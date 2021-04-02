#if !defined(__EYERIS_H__)
#define __EYERIS_H__

#include <atomic>
#include <thread>
#include <vector>

#include "AudioController.hh"
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
  void audioAlertThreadFunc();
  void bluetoothThreadFunc();

  // Bluetooth stuff
  static void LogDebug(const char* pText);
  static void LogInfo(const char* pText);
  static void LogStatus(const char* pText);
  static void LogWarn(const char* pText);
  static void LogError(const char* pText);
  static void LogFatal(const char* pText);
  static void LogAlways(const char* pText);
  static void LogTrace(const char* pText);
  const void* dataGetter(const char* pName);
  int dataSetter(const char* pName, const void* pData);

  std::vector<std::unique_ptr<VL53L0X>> distSensors;
  std::vector<std::unique_ptr<std::atomic_uint16_t>> distances;
  drv2605l::Driver haptics;
  std::atomic_bool running;
  std::thread hapticsThread;
  std::thread distSenseThread;
  std::thread audioAlertThread;
  std::thread bluetoothThread;
  AudioController audioController;
};

#endif  // __EYERIS_H__