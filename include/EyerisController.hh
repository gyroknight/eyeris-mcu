#if !defined(__EYERIS_H__)
#define __EYERIS_H__

#include <atomic>
#include <boost/fiber/barrier.hpp>
#include <thread>
#include <vector>

#include "AudioController.hh"
#include "VL53L0X.hpp"
#include "drv2605lDriver.hh"

enum SoundSet { Default, Variant1 };

class EyerisController {
 public:
  EyerisController();
  ~EyerisController();

  void start();
  void stop();
  uint16_t getDistance(size_t ii);
  void setSoundSet(SoundSet soundSet);

 private:
  void hapticsThreadFunc();
  void distSenseThreadFunc();
  void audioAlertThreadFunc();

  std::vector<std::unique_ptr<VL53L0X>> distSensors;
  std::vector<std::unique_ptr<std::atomic_uint16_t>> distances;
  drv2605l::Driver haptics;
  std::atomic_bool running;
  std::thread hapticsThread;
  std::thread distSenseThread;
  std::thread audioAlertThread;
  boost::fibers::barrier feedbackBarrier;
  AudioController audioController;
  SoundSet soundSet;
};

#endif  // __EYERIS_H__