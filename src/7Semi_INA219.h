#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * 7Semi INA219 Driver
 * - No external dependencies except Wire.h
 * - Full control of config: bus range, shunt PGA, ADC resolutions/averaging, mode
 * - Flexible calibration:
 *    - Direct: set current LSB and shunt ohms
 *    - Auto: provide max expected current and shunt ohms
 * - Readings:
 *    - Shunt voltage (mV, signed)
 *    - Bus voltage (V)
 *    - Current (mA, signed)
 *    - Power (mW)
 * - Utilities:
 *    - Reset device
 *    - Check conversion ready / overflow flags
 *    - Change I2C address at runtime
 */
class INA219_7Semi {
public:
  /**
   * Create driver instance
   * - address defaults to 0x40, valid range 0x40–0x4F
   */
  explicit INA219_7Semi(uint8_t i2c_addr = 0x40);

  /**
   * Initialize I2C and probe device
   * - returns true on ACK
   */
  // bool begin(TwoWire* theWire = &Wire);
  bool begin(TwoWire* theWire, int sda=-1, int scl=-1, uint32_t freqHz = 400000);

  /**
   * Soft reset via CONFIG.RST
   * - reverts config/calibration to defaults
   */
  void reset();

  /**
   * Set full configuration in one shot
   * - busRange16V: false=32V, true=16V
   * - pga: 0=±40mV, 1=±80mV, 2=±160mV, 3=±320mV
   * - badc & sadc: ADC res/avg codes (0..15)
   *   - 0..3 → 9/10/11/12-bit single sample
   *   - 8..15 → 12-bit with 2/4/8/16/32/64/128 samples averaging
   * - mode: 0..7 (power-down/shunt/bus/cont modes)
   */
  void configure(bool busRange16V, uint8_t pga, uint8_t badc, uint8_t sadc, uint8_t mode);

  /**
   * Quick helpers for common fields
   * - bus range: false=32V, true=16V
   * - pga: 0=±40mV, 1=±80mV, 2=±160mV, 3=±320mV
   * - badc/sadc: 0..15 (as datasheet)
   * - mode: 0..7
   */
  void setBusVoltageRange16V(bool enable16V);
  void setPGAGain(uint8_t pga);
  void setBusADC(uint8_t badc);
  void setShuntADC(uint8_t sadc);
  void setMode(uint8_t mode);

  /**
   * Calibration (choose one style)
   * - Direct: set current LSB (A per bit) and shunt resistance (ohms)
   * - Auto: set max expected current (A) and shunt resistance (ohms)
   *   - chooses current LSB near maxExpected/32767, rounded up to a clean value
   */
  void calibrateDirect(float currentLSB_A, float shuntOhms);
  void calibrateAuto(float maxExpectedCurrent_A, float shuntOhms);

  /**
   * Read raw registers (signed where applicable)
   * - shunt voltage LSB = 10 µV
   * - bus voltage LSB = 4 mV, stored in bits [15:3]
   * - current/power depend on calibration
   */
  int16_t readShuntVoltageRaw();
  uint16_t readBusVoltageRaw();
  int16_t readCurrentRaw();
  int16_t readPowerRaw();

  /**
   * Read converted engineering units
   * - Shunt mV (signed)
   * - Bus V
   * - Current mA (signed)
   * - Power mW
   */
  float readShuntVoltage();
  float readBusVoltage();
  float readCurrent();
  float readPower();

  /**
   * Status helpers
   * - overflow: true if math overflow occurred
   * - conversionReady: true if a new bus voltage conversion available
   */
  bool overflow();
  bool conversionReady();

  /**
   * Change I2C address at runtime
   */
  void setAddress(uint8_t addr);

private:
  // Register map
  static constexpr uint8_t REG_CONFIG = 0x00;
  static constexpr uint8_t REG_SHUNT_VOLT = 0x01;
  static constexpr uint8_t REG_BUS_VOLT = 0x02;
  static constexpr uint8_t REG_POWER = 0x03;
  static constexpr uint8_t REG_CURRENT = 0x04;
  static constexpr uint8_t REG_CALIB = 0x05;

  // CONFIG bit fields
  static constexpr uint16_t CFG_RST = 0x8000;
  static constexpr uint16_t CFG_BRNG_32V = 0x0000;
  static constexpr uint16_t CFG_BRNG_16V = 0x2000;
  static constexpr uint16_t CFG_PGA_SHIFT = 11;  // 2 bits
  static constexpr uint16_t CFG_BADC_SHIFT = 7;  // 4 bits
  static constexpr uint16_t CFG_SADC_SHIFT = 3;  // 4 bits
  static constexpr uint16_t CFG_MODE_MASK = 0x0007;

  // Bus status bits
  static constexpr uint16_t BUS_CNVR = 0x0002;
  static constexpr uint16_t BUS_OVF = 0x0001;

  // Fixed LSBs
  static constexpr float SHUNT_LSB_V = 10e-6;  // 10 µV
  static constexpr float BUS_LSB_V = 4e-3;     // 4 mV

  // Calibration constant from datasheet
  static constexpr float CALIB_CONST = 0.04096f;

  TwoWire* _wire;
  uint8_t _addr;
  uint16_t _configCache;

  // Dynamic scaling determined by calibration
  float _currentLSB_A;  // A/bit
  float _powerLSB_W;    // W/bit
  uint16_t _calib;

  void i2cWrite(uint8_t reg, uint16_t val);
  uint16_t i2cRead(uint8_t reg);
  int16_t i2cReadSigned(uint8_t reg);

  void applyCalibration();
  uint16_t makeConfig(bool range16V, uint8_t pga, uint8_t badc, uint8_t sadc, uint8_t mode);
};
