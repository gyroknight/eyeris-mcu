#if !defined(__DRV2605L_DRIVER_H__)
#define __DRV2605L_DRIVER_H__

#include <cstdint>

constexpr long DRV2605L_ADDR = 0x5a;
constexpr uint32_t G1040003D_VOLTAGE = 2500;  // mV

namespace drv2605l {
enum Mode {
  IntTrig,
  ExtTrigEdge,
  ExtTrigLvl,
  PWMAnalog,
  AudioToVibe,
  RealTimePlayback,
  Diag,
  AutoCalib
};

class Driver {
 public:
  Driver(long address = DRV2605L_ADDR,
         uint32_t ratedVoltage = G1040003D_VOLTAGE,
         uint32_t overdriveVoltage = G1040003D_VOLTAGE,
         bool runAutoCalib = false);
  ~Driver();

  void enablePWMInput();
  void enableLRAEffects();
  void setMode(Mode mode);
  void setWaveform(uint8_t value, uint8_t idx, bool delay = false);
  void printWaveform();
  void setPulseWave(uint8_t delay);
  void startPlayback();
  void stopPlayback();
  bool isPlaying();

 private:
  void initHaptics(bool runAutoCalib);
  uint8_t readReg(uint8_t addr, bool* readSuccess = nullptr);
  bool writeReg(uint8_t addr, uint8_t value);
  uint8_t readRegField(uint8_t addr, uint8_t startBit, uint8_t endBit,
                       bool* readSuccess = nullptr);
  bool writeRegField(uint8_t addr, uint8_t value, uint8_t startBit,
                     uint8_t endBit);
  static uint8_t getBitmask(uint8_t startBit, uint8_t endBit);
  static uint8_t calculateVoltage(uint32_t voltage);

  int i2cBusFD;
  long address;
  uint32_t ratedVoltage;
  uint32_t overdriveVoltage;
  Mode currentMode;
};
}  // namespace drv2605l

#endif  // __DRV2605L_DRIVER_H__