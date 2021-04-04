#include <Gobbledegook.h>
#include <unistd.h>
#include <wiringPi.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <thread>

#include "EyerisController.hh"

namespace {
constexpr int buckEnable = 13;
constexpr int speakerEnable = 25;
constexpr int SENSOR_COUNT = 3;
constexpr uint16_t sensErrorDistVal = UINT16_MAX;

//
// Constants
//

// Maximum time to wait for any single async process to timeout during
// initialization
static const int kMaxAsyncInitTimeoutMS = 30 * 1000;

//
// Server data values
//

static std::array<uint16_t, SENSOR_COUNT> distances{
    sensErrorDistVal, sensErrorDistVal, sensErrorDistVal};
static std::stringstream alertStream;
static std::string alertString;

//
// Logging
//

enum LogLevel { Debug, Verbose, Normal, ErrorsOnly };

// Our log level - defaulted to 'Normal' but can be modified via
// command-line options
LogLevel logLevel = Debug;
}  // namespace

// Our full set of logging methods (we just log to stdout)
//
// NOTE: Some methods will only log if the appropriate `logLevel` is set
void LogDebug(const char* pText) {
  if (logLevel <= Debug) {
    std::cout << "  DEBUG: " << pText << std::endl;
  }
}
void LogInfo(const char* pText) {
  if (logLevel <= Verbose) {
    std::cout << "   INFO: " << pText << std::endl;
  }
}
void LogStatus(const char* pText) {
  if (logLevel <= Normal) {
    std::cout << " STATUS: " << pText << std::endl;
  }
}
void LogWarn(const char* pText) {
  std::cout << "WARNING: " << pText << std::endl;
}
void LogError(const char* pText) {
  std::cout << "!!ERROR: " << pText << std::endl;
}
void LogFatal(const char* pText) {
  std::cout << "**FATAL: " << pText << std::endl;
}
void LogAlways(const char* pText) {
  std::cout << "..Log..: " << pText << std::endl;
}
void LogTrace(const char* pText) {
  std::cout << "-Trace-: " << pText << std::endl;
}

//
// Signal handling
//

// We setup a couple Unix signals to perform graceful shutdown in the case of
// SIGTERM or get an SIGING (CTRL-C)
void signalHandler(int signum) {
  switch (signum) {
    case SIGINT:
      LogStatus("SIGINT recieved, shutting down");
      ggkTriggerShutdown();
      break;
    case SIGTERM:
      LogStatus("SIGTERM recieved, shutting down");
      ggkTriggerShutdown();
      break;
  }
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
const void* dataGetter(const char* pName) {
  if (nullptr == pName) {
    LogError("NULL name sent to server data getter");
    return nullptr;
  }

  std::string strName = pName;

  if (strName == "uart_tx") {
    std::stringstream().swap(alertStream);  // Clear previous output
    for (uint16_t distance : distances) {
      alertStream << distance << " ";
    }

    // TODO: Add battery reading info
    alertString = alertStream.str();
    return alertString.c_str();
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
int dataSetter(const char* pName, const void* pData) {
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

int main(int, char*[]) {
  // Set up enable pins
  wiringPiSetup();
  pinMode(buckEnable, OUTPUT);
  pinMode(speakerEnable, OUTPUT);
  // Reset sequence (in case they're still on)
  digitalWrite(buckEnable, false);
  digitalWrite(speakerEnable, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  digitalWrite(buckEnable, true);
  digitalWrite(speakerEnable, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  EyerisController controller;

  controller.start();

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

  // if (!ggkStart("eyeris",
  //               "Nordic UART Test Server",  // dashes in name not allowed
  //               "Nordic UART Test", dataGetter, dataSetter,
  //               kMaxAsyncInitTimeoutMS)) {
  //   std::cout << "Failed to start BLE server" << std::endl;
  // }

  while (true) {
    for (short ii = 0; ii < SENSOR_COUNT; ii++) {
      distances[ii] = controller.getDistance(ii);
    }

    // if (ggkGetServerRunState() < EStopping) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    //   for (uint16_t distance : distances) {
    //     if (distance > sensErrorDistVal) {
    //       ggkNofifyUpdatedCharacteristic(
    //           "/com/eyeris/Nordic_UART_Service/UART_TX");
    //       break;
    //     }
    //   }
    // }

    // if (ggkGetServerRunState() == EStopped) break;
  }

  // Return the final server health status as a success (0) or error (-1)
  return ggkGetServerHealth() == EOk ? 0 : 1;
}
