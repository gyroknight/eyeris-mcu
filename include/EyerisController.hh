#include <bits/stdint-uintn.h>

#include <cstddef>
#if !defined(__EYERIS_H__)
#define __EYERIS_H__

#include <atomic>
#include <boost/fiber/barrier.hpp>
#include <chrono>
#include <thread>
#include <vector>

#include "AudioController.hh"
#include "VL53L0X.hpp"
#include "drv2605lDriver.hh"

enum SoundSet { Default, Variant1 };

enum Alert { None, OneFiveM, TwoM };

class EyerisController {
 public:
  EyerisController();
  ~EyerisController();

  void start();
  void stop();
  uint16_t getDistance(size_t ii);
  void setSoundSet(SoundSet newSoundSet);
  void enableSensor(size_t idx, bool enabled);

 private:
  void hapticsThreadFunc();
  void distSenseThreadFunc();
  void audioAlertThreadFunc();
  Alert getAlert(uint16_t distance);
  void playAlert(uint8_t idx, Alert alert);

  std::vector<std::unique_ptr<VL53L0X>> distSensors;
  std::vector<std::unique_ptr<std::atomic_bool>> sensorEnables;
  std::vector<std::unique_ptr<std::atomic_uint16_t>> distances;
  std::vector<Alert> lastAlert;
  std::vector<std::chrono::time_point<std::chrono::steady_clock>> lastUpdate;
  drv2605l::Driver haptics;
  std::atomic_bool running;
  std::thread hapticsThread;
  std::thread distSenseThread;
  std::thread audioAlertThread;
  boost::fibers::barrier feedbackBarrier;
  AudioController audioController;
  std::atomic<SoundSet> soundSet;
};

#endif  // __EYERIS_H__