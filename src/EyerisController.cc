#include "EyerisController.hh"

#include <unistd.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <functional>
#include <iostream>
#include <stdexcept>

#include "Gobbledegook.h"

namespace {
// Number of distance sensors. If changed, make sure to adjust pins and
// addresses accordingly (i.e. to match size).
constexpr int SENSOR_COUNT = 3;
constexpr std::array<uint8_t, SENSOR_COUNT> PINS{4, 20, 27};
constexpr std::array<uint8_t, SENSOR_COUNT> ADDRESSES{
    VL53L0X_ADDRESS_DEFAULT + 2, VL53L0X_ADDRESS_DEFAULT + 4,
    VL53L0X_ADDRESS_DEFAULT + 6};
constexpr uint16_t sensTimeout = 200;
constexpr uint32_t sensMeasureTimingBudgetUs = 200000;
constexpr float sensSigRateLim = 0.1;              // default is 0.25 MCPS
constexpr uint8_t sensPulsePeriodPreRange = 18;    // default is 14 PCLKs
constexpr uint8_t sensPulsePeriodFinalRange = 14;  // default is 10 PCLKs
constexpr uint16_t sensErrorDistVal = UINT16_MAX;
constexpr uint16_t sensMaxDist = 280;
constexpr uint8_t maxDelay = 50;

//
// Constants
//

// Maximum time to wait for any single async process to timeout during
// initialization
static const int kMaxAsyncInitTimeoutMS = 30 * 1000;

//
// Server data values
//

// Eyeris test string
static std::string eyerisHello("Hello from Eyeris!");

//
// Logging
//

enum LogLevel { Debug, Verbose, Normal, ErrorsOnly };

// Our log level - defaulted to 'Normal' but can be modified via
// command-line options
LogLevel logLevel = Debug;
}  // namespace

EyerisController::EyerisController() : running(false) {
  for (const uint8_t& pin : PINS) {
    distSensors.emplace_back(std::make_unique<VL53L0X>(pin));
    distSensors.back()->powerOff();
  }

  // Init sensor, set timeout and address
  for (size_t ii = 0; ii < distSensors.size(); ii++) {
    try {
      distSensors[ii]->initialize();
      distSensors[ii]->setTimeout(sensTimeout);
      distSensors[ii]->setMeasurementTimingBudget(sensMeasureTimingBudgetUs);
      distSensors[ii]->setAddress(ADDRESSES[ii]);
      distSensors[ii]->setSignalRateLimit(sensSigRateLim);
      distSensors[ii]->setVcselPulsePeriod(VcselPeriodPreRange,
                                           sensPulsePeriodPreRange);
      distSensors[ii]->setVcselPulsePeriod(VcselPeriodFinalRange,
                                           sensPulsePeriodFinalRange);
    } catch (const std::exception& error) {
      std::cout << "Failed to initialize sensor " << ii << ", removing...";
      distSensors[ii].reset();
    }
  }

  for (size_t ii = 0; ii < distSensors.size(); ii++) {
    distances.emplace_back(
        std::make_unique<std::atomic_uint16_t>(sensErrorDistVal));
  }

  haptics.enableLRAEffects();

  // Register our loggers
  ggkLogRegisterDebug(LogDebug);
  ggkLogRegisterInfo(LogInfo);
  ggkLogRegisterStatus(LogStatus);
  ggkLogRegisterWarn(LogWarn);
  ggkLogRegisterError(LogError);
  ggkLogRegisterFatal(LogFatal);
  ggkLogRegisterAlways(LogAlways);
  ggkLogRegisterTrace(LogTrace);

  // Start the server's async processing
  //
  // This starts the server on a thread and begins the initialization process
  //
  // !!!IMPORTANT!!!
  //
  //     This first parameter (the service name) must match tha name configured
  //     in the D-Bus permissions. See the Readme.md file for more information.
  //

  std::function<const void*(const char* pName)> dataGetterFunc =
      [this](const char* pName) -> const void* {
    return this->dataGetter(pName);
  };
  std::function<int(const char* pName, const void* pData)> dataSetterFunc =
      [this](const char* pName, const void* pData) -> int {
    return this->dataSetter(pName, pData);
  };

  if (!ggkStart("eyeris",
                "Nordic UART Test Server",  // dashes in name not allowed
                "Nordic UART Test",
                dataGetterFunc.target<const void*(const char*)>(),
                dataSetterFunc.target<int(const char*, const void*)>(),
                kMaxAsyncInitTimeoutMS)) {
    std::cout << "Failed to start BLE server" << std::endl;
  }
}

EyerisController::~EyerisController() { stop(); }

void EyerisController::start() {
  if (!running) {
    running = true;
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      try {
        if (distSensors[ii]) distSensors[ii]->startContinuous();
      } catch (const std::exception& error) {
        std::cerr << "Failed to start continuous read mode on sensor " << ii
                  << " with reason: " << error.what();
        throw std::runtime_error("Cannot read from sensors");
      }
    }

    try {
      hapticsThread = std::thread(&EyerisController::hapticsThreadFunc, this);
      distSenseThread =
          std::thread(&EyerisController::distSenseThreadFunc, this);
      audioAlertThread =
          std::thread(&EyerisController::audioAlertThreadFunc, this);
      bluetoothThread =
          std::thread(&EyerisController::bluetoothThreadFunc, this);
    } catch (std::system_error& ee) {
      std::cerr << "Failed to start threads";
      stop();
    }
  } else {
    std::cerr << "Eyeris already started!";
  }
}

void EyerisController::stop() {
  std::cout << "Shutting down Eyeris" << std::endl;
  running = false;
  if (hapticsThread.joinable()) hapticsThread.join();
  if (distSenseThread.joinable()) distSenseThread.join();
  if (audioAlertThread.joinable()) audioAlertThread.join();
  if (bluetoothThread.joinable()) bluetoothThread.join();
}

uint16_t EyerisController::getDistance(size_t ii) {
  try {
    return *distances.at(ii);
  } catch (std::out_of_range& ee) {
    return 0;
  }
}

void EyerisController::hapticsThreadFunc() {
  while (running) {
    for (uint8_t ii = 0; ii < distances.size(); ii++) {
      uint16_t distance = *distances[ii];
      auto startTime = std::chrono::steady_clock::now();
      if (distance < sensMaxDist) {
        // Direction indicator
        bool waveRunning;
        uint8_t indicatorEffect = 64;
        switch (ii) {
          case 0:
            indicatorEffect = 4;  // Sharp click, 100%
            break;
          case 1:
            indicatorEffect = 10;  // Double click, 100%;
            break;
          case 2:
            indicatorEffect = 12;  // Triple click, 100%
            break;
          default:
            std::cout << "Unsupported sensor " << ii << std::endl;
        }
        haptics.setWaveform(indicatorEffect, 0);
        haptics.setWaveform(50, 1,
                            true);  // wait 500ms before starting distance wave
        haptics.setWaveform(0, 2);
        haptics.startPlayback();
        do {
          waveRunning = haptics.isPlaying();
          usleep(1);
        } while (waveRunning);

        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - startTime)
                       .count() < 4 &&
               distance < sensMaxDist) {
          haptics.setPulseWave(distance * maxDelay / sensMaxDist);
          haptics.startPlayback();
          do {
            waveRunning = haptics.isPlaying();
            usleep(1);
          } while (waveRunning);

          distance = *distances[ii];  // Fetch new distance info
        }

        usleep(500000);  // wait 500ms before moving to next direction
      } else {
        usleep(1);
      }
    }
  }
}

void EyerisController::distSenseThreadFunc() {
  while (running) {
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      uint16_t distance;
      try {
        distance = distSensors[ii]
                       ? distSensors[ii]->readRangeContinuousMillimeters()
                       : sensErrorDistVal;
      } catch (const std::exception& error) {
        std::cerr << "Failed to read sensor " << ii
                  << " with reason: " << error.what();
        distance = sensErrorDistVal;
      }

      if (distance == sensErrorDistVal)
        continue;
      else if (distSensors[ii]->timeoutOccurred())
        std::cerr << "Timeout occurred on sensor " << ii;
      else
        *distances[ii] = distance;
    }
  }

  for (std::unique_ptr<VL53L0X>& ss : distSensors) {
    ss->stopContinuous();
  }
}

void EyerisController::audioAlertThreadFunc() {
  while (running) {
  }
}

void EyerisController::bluetoothThreadFunc() {
  while (running) {
    if (ggkGetServerRunState() < EStopping) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      ggkNofifyUpdatedCharacteristic("/com/eyeris/Nordic_UART_Service/UART_TX");
    }
  }

  ggkTriggerShutdown();
}

// Our full set of logging methods (we just log to stdout)
//
// NOTE: Some methods will only log if the appropriate `logLevel` is set
void EyerisController::LogDebug(const char* pText) {
  if (logLevel <= Debug) {
    std::cout << "  DEBUG: " << pText << std::endl;
  }
}
void EyerisController::LogInfo(const char* pText) {
  if (logLevel <= Verbose) {
    std::cout << "   INFO: " << pText << std::endl;
  }
}
void EyerisController::LogStatus(const char* pText) {
  if (logLevel <= Normal) {
    std::cout << " STATUS: " << pText << std::endl;
  }
}
void EyerisController::LogWarn(const char* pText) {
  std::cout << "WARNING: " << pText << std::endl;
}
void EyerisController::LogError(const char* pText) {
  std::cout << "!!ERROR: " << pText << std::endl;
}
void EyerisController::LogFatal(const char* pText) {
  std::cout << "**FATAL: " << pText << std::endl;
}
void EyerisController::LogAlways(const char* pText) {
  std::cout << "..EyerisController::Log..: " << pText << std::endl;
}
void EyerisController::LogTrace(const char* pText) {
  std::cout << "-Trace-: " << pText << std::endl;
}

//
// Server data management
//

// Called by the server when it wants to retrieve a named value
//
// This method conforms to `GGKServerDataGetter` and is passed to the server via
// our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our
// implementation is thread-safe. In our case, we're simply sending over stored
// values, so we don't need to take any additional steps to ensure
// thread-safety.
const void* EyerisController::dataGetter(const char* pName) {
  if (nullptr == pName) {
    LogError("NULL name sent to server data getter");
    return nullptr;
  }

  std::string strName = pName;

  if (strName == "uart_tx") {
    return eyerisHello.c_str();
  }

  LogWarn((std::string("Unknown name for server data getter request: '") +
           pName + "'")
              .c_str());
  return nullptr;
}

// Called by the server when it wants to update a named value
//
// This method conforms to `GGKServerDataSetter` and is passed to the server via
// our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our
// implementation is thread-safe. In our case, we're simply sending over stored
// values, so we don't need to take any additional steps to ensure
// thread-safety.
int EyerisController::dataSetter(const char* pName, const void* pData) {
  if (nullptr == pName) {
    LogError("NULL name sent to server data setter");
    return 0;
  }
  if (nullptr == pData) {
    LogError("NULL pData sent to server data setter");
    return 0;
  }

  std::string strName = pName;

  if (strName == "uart_rx") {
    const char* dataStr = static_cast<const char*>(pData);
    LogDebug((std::string("Server received: ") + std::string(dataStr)).c_str());
    return 1;
  }

  LogWarn((std::string("Unknown name for server data setter request: '") +
           pName + "'")
              .c_str());

  return 0;
}