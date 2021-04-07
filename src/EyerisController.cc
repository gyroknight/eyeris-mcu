#include "EyerisController.hh"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include "VL53L0X.hpp"

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
constexpr uint16_t sensMaxDist = 2000;
constexpr uint16_t sensMinDist = 1000;
constexpr uint8_t maxDelay = 50;
constexpr uint8_t numFeedback = 2;
const std::vector<std::string> soundFilenames{
    "resources/1.5m.wav", "resources/2m.wav", "resources/above.wav",
    "resources/left.wav", "resources/right.wav"};
const std::vector<std::string> soundFilenames1{
    "resources/1.5m_1.wav", "resources/2m_1.wav", "resources/above_1.wav",
    "resources/left_1.wav", "resources/right_1.wav"};
constexpr size_t oneFiveMSoundIdx = 0;
constexpr size_t twoMSoundIdx = 1;
constexpr size_t aboveSoundIdx = 2;
constexpr size_t leftSoundIdx = 3;
constexpr size_t rightSoundIdx = 4;
std::unordered_map<SoundSet, std::vector<std::string>> soundFilenamesMap{
    {Default, soundFilenames}, {Variant1, soundFilenames1}};
}  // namespace

EyerisController::EyerisController()
    : running(false), feedbackBarrier(numFeedback), soundSet(Default) {
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
    sensorEnables.emplace_back(std::make_unique<std::atomic_bool>(true));
  }

  lastAlert.assign({None, None, None});
  lastUpdate.assign({std::chrono::steady_clock::now(),
                     std::chrono::steady_clock::now(),
                     std::chrono::steady_clock::now()});

  haptics.enableLRAEffects();
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
  for (uint8_t ii = 0; ii < numFeedback; ii++) {
    // Trigger waits to unblock any pending threads
    feedbackBarrier.wait();
  }
  if (hapticsThread.joinable()) hapticsThread.join();
  if (distSenseThread.joinable()) distSenseThread.join();
  if (audioAlertThread.joinable()) audioAlertThread.join();
}

uint16_t EyerisController::getDistance(size_t ii) {
  try {
    return *distances.at(ii);
  } catch (std::out_of_range& ee) {
    return 0;
  }
}

void EyerisController::setSoundSet(SoundSet newSoundSet) {
  soundSet = newSoundSet;
}

SoundSet EyerisController::numToSoundSet(int num) {
  switch (num) {
    case 0:
      return Default;
    case 1:
      return Variant1;
      break;
    default:
      std::cout << "Unrecognized SoundSet value" << std::endl;
      return Default;
  }
}

void EyerisController::enableSensor(size_t idx, bool enabled) {
  if (idx < SENSOR_COUNT)
    *sensorEnables[idx] = enabled;
  else
    std::cout << "Unknown sensor" << std::endl;
}

void EyerisController::hapticsThreadFunc() {
  while (running) {
    for (uint8_t ii = 0; ii < distances.size(); ii++) {
      feedbackBarrier.wait();
      if (*sensorEnables[ii]) {
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
          haptics.setWaveform(
              50, 1,
              true);  // wait 500ms before starting distance wave
          haptics.setWaveform(0, 2);
          haptics.startPlayback();
          do {
            waveRunning = haptics.isPlaying();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
          } while (waveRunning);

          while (std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::steady_clock::now() - startTime)
                         .count() < 4 &&
                 distance < sensMaxDist) {
            distance = std::max(distance, sensMinDist);
            haptics.setPulseWave((distance - sensMinDist) * maxDelay /
                                 (sensMaxDist - sensMinDist));
            haptics.startPlayback();
            do {
              waveRunning = haptics.isPlaying();
              std::this_thread::sleep_for(std::chrono::microseconds(500));
            } while (waveRunning);

            distance = *distances[ii];  // Fetch new distance info
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(
              500));  // wait 500ms before moving to next direction
        }
      }
    }
  }
}

void EyerisController::distSenseThreadFunc() {
  while (running) {
    for (size_t ii = 0; ii < distSensors.size(); ii++) {
      if (*sensorEnables[ii]) {
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

    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  for (std::unique_ptr<VL53L0X>& ss : distSensors) {
    ss->stopContinuous();
  }
}

void EyerisController::audioAlertThreadFunc() {
  for (const std::string& filename : soundFilenames) {
    audioController.loadFile(filename);
  }

  for (const std::string& filename : soundFilenames1) {
    audioController.loadFile(filename);
  }

  while (running) {
    for (uint8_t ii = 0; ii < distances.size(); ii++) {
      feedbackBarrier.wait();
      if (*sensorEnables[ii]) {
        uint16_t distance = *distances[ii];
        Alert nextAlert = getAlert(distance);
        if (nextAlert != None &&
            (nextAlert != lastAlert[ii] ||
             std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::steady_clock::now() - lastUpdate[ii])
                     .count() > 5)) {
          playAlert(ii, nextAlert);
        }

        lastAlert[ii] = nextAlert;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

Alert EyerisController::getAlert(uint16_t distance) {
  if (distance <= 1500) {
    return OneFiveM;
  } else if (distance <= 2000) {
    return TwoM;
  } else {
    return None;
  }
}

void EyerisController::playAlert(uint8_t idx, Alert alert) {
  switch (idx) {
    case 0:
      audioController.playFile(soundFilenamesMap[soundSet][rightSoundIdx]);
      break;
    case 1:
      audioController.playFile(soundFilenamesMap[soundSet][aboveSoundIdx]);
      break;
    case 2:
      audioController.playFile(soundFilenamesMap[soundSet][leftSoundIdx]);
      break;
    default:
      return;
  }

  switch (alert) {
    case OneFiveM:
      audioController.playFile(soundFilenamesMap[soundSet][oneFiveMSoundIdx]);
      break;
    case TwoM:
      audioController.playFile(soundFilenamesMap[soundSet][twoMSoundIdx]);
      break;
    default:
      std::cout << "Invalid alert to play" << std::endl;
  }

  lastUpdate[idx] = std::chrono::steady_clock::now();
}