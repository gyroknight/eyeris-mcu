#include "EyerisController.hh"

#include <array>
#include <iostream>

namespace {
// Number of distance sensors. If changed, make sure to adjust pins and
// addresses accordingly (i.e. to match size).
constexpr int SENSOR_COUNT = 3;
constexpr std::array<uint8_t, SENSOR_COUNT> PINS{25, 27, 21};
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
}  // namespace

EyerisController::EyerisController() : running(false) {
  for (const uint8_t& pin : PINS) {
    distSensors.emplace_back(std::make_unique<VL53L0X>(pin));
    distSensors.back()->powerOff();
  }

  try {
    // Init sensor, set timeout and address
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      distSensors[ii]->initialize();
      distSensors[ii]->setTimeout(sensTimeout);
      distSensors[ii]->setMeasurementTimingBudget(sensMeasureTimingBudgetUs);
      distSensors[ii]->setAddress(ADDRESSES[ii]);
      distSensors[ii]->setSignalRateLimit(sensSigRateLim);
      distSensors[ii]->setVcselPulsePeriod(VcselPeriodPreRange,
                                           sensPulsePeriodPreRange);
      distSensors[ii]->setVcselPulsePeriod(VcselPeriodFinalRange,
                                           sensPulsePeriodFinalRange);
    }
  } catch (const std::exception& error) {
    std::cerr << "Sensor init error: " << error.what();
    throw std::runtime_error("Failed to intialize all sensors");
  }

  for (size_t ii = 0; ii < distSensors.size(); ii++) {
    distances.emplace_back(
        std::make_unique<std::atomic_uint16_t>(sensErrorDistVal));
  }

  haptics.enableLRAEffects();
}

EyerisController::~EyerisController() { stop(); }

void EyerisController::start() {
  if (!running) {
    running = true;
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      try {
        distSensors[ii]->startContinuous();
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
}

uint16_t EyerisController::getDistance(size_t ii) { return *distances[ii]; }

void EyerisController::hapticsThreadFunc() {
  while (running) {
    uint16_t distance = *distances[0];
    if (distance < sensMaxDist) {
      haptics.setPulseWave(distance * maxDelay / sensMaxDist);
      haptics.fireWaveform();
    } else {
      haptics.stopPlayback();
    }
  }
}

void EyerisController::distSenseThreadFunc() {
  while (running) {
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      uint16_t distance;
      try {
        distance = distSensors[ii]->readRangeContinuousMillimeters();
      } catch (const std::exception& error) {
        std::cerr << "Failed to read sensor " << ii
                  << " with reason: " << error.what();
        distance = sensErrorDistVal;
      }

      if (distSensors[ii]->timeoutOccurred())
        std::cerr << "Timeout occurred on sensor " << ii;
      else if (distance != sensErrorDistVal)
        *distances[ii] = distance;
    }
  }

  for (std::unique_ptr<VL53L0X>& ss : distSensors) {
    ss->stopContinuous();
  }
}