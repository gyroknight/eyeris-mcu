#if !defined(__EYERIS_H__)
#define __EYERIS_H__

#include <atomic>
#include <thread>
#include <vector>

#include "VL53L0X.hpp"
#include "drv2605lDriver.hh"

class EyerisController {
 public:
  EyerisController();
  ~EyerisController();

  void start();
  void stop();
  uint16_t getDistance(size_t ii);

 private:
  void hapticsThreadFunc();
  void distSenseThreadFunc();

  std::vector<std::unique_ptr<VL53L0X>> distSensors;
  std::vector<std::unique_ptr<std::atomic_uint16_t>> distances;
  drv2605l::Driver haptics;
  std::atomic_bool running;
  std::thread hapticsThread;
  std::thread distSenseThread;
};

#endif  // __EYERIS_H__