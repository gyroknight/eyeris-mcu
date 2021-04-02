#include "drv2605lDriver.hh"

#include <fcntl.h>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr const char* I2C_DEV_PATH = "/dev/i2c-%d";

// clang-format off
// Register addresses
constexpr uint8_t DRV260X_STATUS           = 0x0;
constexpr uint8_t DRV260X_MODE             = 0x1;
constexpr uint8_t DRV260X_RT_PB_IN         = 0x2;
constexpr uint8_t DRV260X_LIB_SEL          = 0x3;
constexpr uint8_t DRV260X_WV_SEQ_1         = 0x4;
constexpr uint8_t DRV260X_WV_SEQ_2         = 0x5;
constexpr uint8_t DRV260X_WV_SEQ_3         = 0x6;
constexpr uint8_t DRV260X_WV_SEQ_4         = 0x7;
constexpr uint8_t DRV260X_WV_SEQ_5         = 0x8;
constexpr uint8_t DRV260X_WV_SEQ_6         = 0x9;
constexpr uint8_t DRV260X_WV_SEQ_7         = 0xa;
constexpr uint8_t DRV260X_WV_SEQ_8         = 0xb;
constexpr uint8_t DRV260X_GO               = 0xc;
constexpr uint8_t DRV260X_OVERDRIVE_OFF    = 0xd;
constexpr uint8_t DRV260X_SUSTAIN_P_OFF    = 0xe;
constexpr uint8_t DRV260X_SUSTAIN_N_OFF    = 0xf;
constexpr uint8_t DRV260X_BRAKE_OFF        = 0x10;
constexpr uint8_t DRV260X_A_TO_V_CTRL      = 0x11;
constexpr uint8_t DRV260X_A_TO_V_MIN_INPUT = 0x12;
constexpr uint8_t DRV260X_A_TO_V_MAX_INPUT = 0x13;
constexpr uint8_t DRV260X_A_TO_V_MIN_OUT   = 0x14;
constexpr uint8_t DRV260X_A_TO_V_MAX_OUT   = 0x15;
constexpr uint8_t DRV260X_RATED_VOLT       = 0x16;
constexpr uint8_t DRV260X_OD_CLAMP_VOLT    = 0x17;
constexpr uint8_t DRV260X_CAL_COMP         = 0x18;
constexpr uint8_t DRV260X_CAL_BACK_EMF     = 0x19;
constexpr uint8_t DRV260X_FEEDBACK_CTRL    = 0x1a;
constexpr uint8_t DRV260X_CTRL1            = 0x1b;
constexpr uint8_t DRV260X_CTRL2            = 0x1c;
constexpr uint8_t DRV260X_CTRL3            = 0x1d;
constexpr uint8_t DRV260X_CTRL4            = 0x1e;
constexpr uint8_t DRV260X_CTRL5            = 0x1f;
constexpr uint8_t DRV260X_LRA_LOOP_PERIOD  = 0x20;
constexpr uint8_t DRV260X_VBAT_MON         = 0x21;
constexpr uint8_t DRV260X_LRA_RES_PERIOD   = 0x22;
constexpr uint8_t DRV260X_MAX_REG          = 0x23;

constexpr uint8_t DRV260X_GO_BIT           = 0x01;

// Library selection
constexpr uint8_t DRV260X_LIB_SEL_MASK     = 0x07;
constexpr uint8_t DRV260X_LIB_SEL_RAM      = 0x0;
constexpr uint8_t DRV260X_LIB_SEL_OD       = 0x1;
constexpr uint8_t DRV260X_LIB_SEL_40_60    = 0x2;
constexpr uint8_t DRV260X_LIB_SEL_60_80    = 0x3;
constexpr uint8_t DRV260X_LIB_SEL_100_140  = 0x4;
constexpr uint8_t DRV260X_LIB_SEL_140_PLUS = 0x5;
constexpr uint8_t DRV260X_LIB_SEL_LRA      = 0x6;
constexpr uint8_t DRV260X_LIB_SEL_5V_35_45 = 0x7;

constexpr uint8_t DRV260X_LIB_SEL_HIZ_MASK = 0x10;
constexpr uint8_t DRV260X_LIB_SEL_HIZ_EN   = 0x01;
constexpr uint8_t DRV260X_LIB_SEL_HIZ_DIS  = 0;

// Mode register
constexpr uint8_t DRV260X_STANDBY           = (1 << 6);
constexpr uint8_t DRV260X_STANDBY_MASK      = 0x40;
constexpr uint8_t DRV260X_INTERNAL_TRIGGER  = 0x00;
constexpr uint8_t DRV260X_EXT_TRIGGER_EDGE  = 0x01;
constexpr uint8_t DRV260X_EXT_TRIGGER_LEVEL = 0x02;
constexpr uint8_t DRV260X_PWM_ANALOG_IN     = 0x03;
constexpr uint8_t DRV260X_AUDIOHAPTIC       = 0x04;
constexpr uint8_t DRV260X_RT_PLAYBACK       = 0x05;
constexpr uint8_t DRV260X_DIAGNOSTICS       = 0x06;
constexpr uint8_t DRV260X_AUTO_CAL          = 0x07;

// Feedback register
constexpr uint8_t DRV260X_FB_REG_ERM_MODE = 0x7f;
constexpr uint8_t DRV260X_FB_REG_LRA_MODE = (1 << 7);

constexpr uint8_t DRV260X_BRAKE_FACTOR_MASK = 0x1f;
constexpr uint8_t DRV260X_BRAKE_FACTOR_2X   = (1 << 0);
constexpr uint8_t DRV260X_BRAKE_FACTOR_3X   = (2 << 4);
constexpr uint8_t DRV260X_BRAKE_FACTOR_4X   = (3 << 4);
constexpr uint8_t DRV260X_BRAKE_FACTOR_6X   = (4 << 4);
constexpr uint8_t DRV260X_BRAKE_FACTOR_8X   = (5 << 4);
constexpr uint8_t DRV260X_BRAKE_FACTOR_16   = (6 << 4);
constexpr uint8_t DRV260X_BRAKE_FACTOR_DIS  = (7 << 4);

constexpr uint8_t DRV260X_LOOP_GAIN_LOW       = 0xf3;
constexpr uint8_t DRV260X_LOOP_GAIN_MED       = (1 << 2);
constexpr uint8_t DRV260X_LOOP_GAIN_HIGH      = (2 << 2);
constexpr uint8_t DRV260X_LOOP_GAIN_VERY_HIGH = (3 << 2);

constexpr uint8_t DRV260X_BEMF_GAIN_0 = 0xfc;
constexpr uint8_t DRV260X_BEMF_GAIN_1 = (1 << 0);
constexpr uint8_t DRV260X_BEMF_GAIN_2 = (2 << 0);
constexpr uint8_t DRV260X_BEMF_GAIN_3 = (3 << 0);

// Control 1 register
constexpr uint8_t DRV260X_AC_CPLE_EN      = (1 << 5);
constexpr uint8_t DRV260X_STARTUP_BOOST   = (1 << 7);
constexpr uint8_t DRV260X_DRIVE_TIME_MASK = 0x1F;

// Control 3 register
constexpr uint8_t DRV260X_LRA_OPEN_LOOP     = (1 << 0);
constexpr uint8_t DRV260X_ANALOG_IN        = (1 << 1);
constexpr uint8_t DRV260X_LRA_DRV_MODE      = (1 << 2);
constexpr uint8_t DRV260X_RTP_UNSIGNED_DATA = (1 << 3);
constexpr uint8_t DRV260X_SUPPLY_COMP_DIS   = (1 << 4);
constexpr uint8_t DRV260X_ERM_OPEN_LOOP     = (1 << 5);
constexpr uint8_t DRV260X_NG_THRESH_0       = (0 << 6);
constexpr uint8_t DRV260X_NG_THRESH_2       = (1 << 6);
constexpr uint8_t DRV260X_NG_THRESH_4       = (2 << 6);
constexpr uint8_t DRV260X_NG_THRESH_8       = (3 << 6);
// clang-format on

std::vector<std::pair<uint8_t, uint8_t>> drv2605lLRACalRegs = {
    {DRV260X_MODE, DRV260X_AUTO_CAL},
    {DRV260X_CTRL3, DRV260X_NG_THRESH_2},
    {DRV260X_FEEDBACK_CTRL, DRV260X_FB_REG_LRA_MODE | DRV260X_BRAKE_FACTOR_4X |
                                DRV260X_LOOP_GAIN_MED}};
}  // namespace

using namespace drv2605l;

Driver::Driver(long address, uint32_t ratedVoltage, uint32_t overdriveVoltage,
               uint8_t driveTime, bool runAutoCalib)
    : address(address),
      ratedVoltage(ratedVoltage),
      overdriveVoltage(overdriveVoltage),
      currentMode(IntTrig) {
  char filename[20];
  int size;
  bool foundBus = false;

  // Assumes drv2605l is on first available I2C bus, limited to 256 minor
  // numbers
  for (int ii = 0; ii < 256; ii++) {
    size = snprintf(filename, 20, I2C_DEV_PATH, ii);
    if (size < 0 || size >= 20) {
      throw std::runtime_error("Failed to format I2C device path");
    }
    i2cBusFD = open(filename, O_RDWR);
    if (i2cBusFD >= 0) {
      std::cout << "I2C bus " << ii << " found" << std::endl;
      foundBus = true;
      break;
    }
  }

  if (!foundBus) throw std::runtime_error("Failed to find I2C bus");

  initHaptics(runAutoCalib, driveTime);
}

Driver::~Driver() { close(i2cBusFD); }

void Driver::enablePWMInput() {
  setMode(PWMAnalog);
  writeReg(DRV260X_MODE, DRV260X_PWM_ANALOG_IN);
  // Disable common-mode drive
  writeReg(DRV260X_CTRL1, readReg(DRV260X_CTRL1) & ~DRV260X_AC_CPLE_EN);
  // Set IN/TRIG input mode to PWM
  writeReg(DRV260X_CTRL3, readReg(DRV260X_CTRL3) & ~DRV260X_ANALOG_IN);
}

void Driver::enableLRAEffects() {
  // Default to internally-triggered waveform
  setMode(IntTrig);
  writeReg(DRV260X_LIB_SEL, DRV260X_LIB_SEL_LRA);
}

void Driver::setMode(Mode mode) {
  if (mode != currentMode) {
    switch (mode) {
      case IntTrig:
        writeReg(DRV260X_MODE, DRV260X_INTERNAL_TRIGGER);
        break;
      case ExtTrigEdge:
        writeReg(DRV260X_MODE, DRV260X_EXT_TRIGGER_EDGE);
        break;
      case ExtTrigLvl:
        writeReg(DRV260X_MODE, DRV260X_EXT_TRIGGER_LEVEL);
        break;
      case PWMAnalog:
        writeReg(DRV260X_MODE, DRV260X_PWM_ANALOG_IN);
        break;
      case AudioToVibe:
        writeReg(DRV260X_MODE, DRV260X_AUDIOHAPTIC);
        break;
      case RealTimePlayback:
        writeReg(DRV260X_MODE, DRV260X_RT_PLAYBACK);
        break;
      case Diag:
        writeReg(DRV260X_MODE, DRV260X_DIAGNOSTICS);
        break;
      case AutoCalib:
        writeReg(DRV260X_MODE, DRV260X_AUTO_CAL);
        break;
      default:
        std::cout << "Tried to set unsupported mode" << std::endl;
        return;
    }
    currentMode = mode;
  }
}

void Driver::setWaveform(uint8_t value, uint8_t idx, bool delay) {
  if ((!delay && value > 123) || value > 0x7F)
    throw std::runtime_error("Invalid effect or delay value");
  if (idx > 7) throw std::runtime_error("Invalid waveform register index");
  if (delay) {
    value |= (1 << 7);
  }
  writeReg(DRV260X_WV_SEQ_1 + idx, value);
}

void Driver::printWaveform() {
  for (uint8_t ii = 0; ii < 8; ii++) {
    std::cout << "Waveform register " << +ii << ": "
              << +readReg(DRV260X_WV_SEQ_1 + ii) << std::endl;
  }
}

void Driver::setPulseWave(uint8_t delay) {
  for (uint8_t ii = 0; ii < 3; ii += 2) {
    setWaveform(3, ii);
  }

  for (uint8_t ii = 1; ii < 4; ii += 2) {
    // NOTE: 10s of ms
    setWaveform(delay, ii, true);
  }
  setWaveform(4, 0);
  // setWaveform(delay, 1, true);
  // setWaveform(0, 2);
}

void Driver::startPlayback() {
  writeRegField(DRV260X_GO, DRV260X_GO_BIT, 0, 0);
}

void Driver::stopPlayback() { writeRegField(DRV260X_GO, 0, 0, 0); }

bool Driver::isPlaying() { return readRegField(DRV260X_GO, 0, 0) == 1; }

void Driver::initHaptics(bool runAutoCalib, uint8_t driveTime) {
  if (ioctl(i2cBusFD, I2C_SLAVE, address) < 0) {
    throw std::runtime_error("Failed to set DRV2605L I2C address");
  }

  uint8_t value = readReg(DRV260X_STATUS);

  if ((value >> 5) != 7) {
    throw std::runtime_error("Device on bus is not DRV2605L");
  }

  // Reset takes too long, needs investigation
  // bool writeSuccess = writeRegField(DRV260X_MODE, 1, 7, 7);
  // if (!writeSuccess) throw std::runtime_error("Failed to reset haptics
  // driver");

  // usleep(250);  // Driver doesn't accept command for at least 250us from the
  //               // datasheet

  // sleep(30);  // this thing takes forever to boot

  // int timeout = 0;
  // bool readSuccess = false;

  // while ((!readRegField(DRV260X_MODE, 7, 7, &readSuccess) || !readSuccess) &&
  //        timeout < 5000000) {
  //   usleep(1000);
  //   timeout += 1000;
  // }

  // if (timeout >= 5000)
  //   throw std::runtime_error("Driver did not finish resetting in time");

  // Set Rated and Overdrive voltage
  bool writeSuccess = writeReg(DRV260X_RATED_VOLT, ratedVoltage);
  if (!writeSuccess)
    throw std::runtime_error(std::string("Failed to write rated voltage ")
                                 .append(std::to_string(ratedVoltage)));

  writeSuccess = writeReg(DRV260X_OD_CLAMP_VOLT, overdriveVoltage);
  if (!writeSuccess)
    throw std::runtime_error(std::string("Failed to write overdrive voltage ")
                                 .append(std::to_string(overdriveVoltage)));

  // Set initial drive time guess
  writeSuccess = writeRegField(DRV260X_CTRL1, driveTime, 0, 4);

  // Set actuator to LRA
  writeSuccess = writeRegField(DRV260X_FEEDBACK_CTRL, 1, 7, 7);

  // Run LRA autocalibration sequence (requires explicit support by the
  // actuator)
  if (runAutoCalib) {
    for (std::pair<uint8_t, uint8_t>& regVal : drv2605lLRACalRegs) {
      writeSuccess =
          writeSuccess ? writeReg(regVal.first, regVal.second) : false;
    }

    if (!writeSuccess)
      throw std::runtime_error("Failed to write LRA calibration sequence");

    writeSuccess = writeReg(DRV260X_GO, DRV260X_GO_BIT);
    if (!writeSuccess) throw std::runtime_error("Failed to write GO register");

    uint8_t calBuf;

    do {
      calBuf = readReg(DRV260X_GO);
    } while (calBuf == DRV260X_GO_BIT);

    currentMode = AutoCalib;

    if (readRegField(DRV260X_STATUS, 3, 3))
      std::cout << "Warning: actuator auto-calibration failed" << std::endl;
  }

  // Disable ROM effects library
  writeSuccess = writeReg(DRV260X_LIB_SEL, DRV260X_LIB_SEL_RAM);

  // Kick it out of standby
  writeSuccess = writeReg(DRV260X_MODE, 0);

  // Enable internal trigger mode
  setMode(IntTrig);
}

uint8_t Driver::readReg(uint8_t addr, bool* readSuccess) {
  int32_t res = i2c_smbus_read_byte_data(i2cBusFD, addr);
  if (readSuccess) *readSuccess = res >= 0;
  if (res < 0) {
    std::cerr << "Failed to read register" << std::hex << +addr << std::endl;
    return 0;
  } else {
    return static_cast<uint8_t>(res);
  }
}

bool Driver::writeReg(uint8_t addr, uint8_t value) {
  int32_t res = i2c_smbus_write_byte_data(i2cBusFD, addr, value);
  if (res) {
    std::cerr << "Failed to write register " << std::hex << addr << " value "
              << value << std::endl;
  }

  return res == 0;
}

uint8_t Driver::readRegField(uint8_t addr, uint8_t startBit, uint8_t endBit,
                             bool* readSuccess) {
  uint8_t currentValue = readReg(addr, readSuccess);
  uint8_t mask = getBitmask(startBit, endBit);
  return (currentValue & mask) >> startBit;
}

bool Driver::writeRegField(uint8_t addr, uint8_t value, uint8_t startBit,
                           uint8_t endBit) {
  uint8_t currentValue = readReg(addr);
  uint8_t mask = getBitmask(startBit, endBit);
  value = ((value << startBit) & mask);
  value = currentValue ^ ((currentValue ^ value) & mask);
  return writeReg(addr, value);
}

uint8_t Driver::getBitmask(uint8_t startBit, uint8_t endBit) {
  if (startBit > endBit || endBit > 7)
    throw std::runtime_error("Invalid field params");
  uint8_t mask = 0xFF;
  mask >>= (7 - (endBit - startBit));
  mask <<= startBit;
  return mask;
}