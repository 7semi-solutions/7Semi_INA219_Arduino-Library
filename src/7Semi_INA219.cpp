#include "7Semi_INA219.h"

/** ctor */
INA219_7Semi::INA219_7Semi(uint8_t i2c_addr)
  : _wire(&Wire),
    _addr(i2c_addr),
    _configCache(0),
    _currentLSB_A(0.0f),
    _powerLSB_W(0.0f),
    _calib(0) {}

bool INA219_7Semi::begin(TwoWire* theWire, int sda, int scl, uint32_t freqHz) {
  // Select wire instance
  _wire = theWire ? theWire : &Wire;

/**
   * I2C bus bring-up
   * - If custom SDA/SCL are provided and supported, use them.
   * - Otherwise call default begin().
   * - Set bus clock if requested.
   */
#if defined(ESP32) || defined(ESP8266)
  if (sda >= 0 && scl >= 0) {
    _wire->begin((int)sda, (int)scl);
  } else {
    _wire->begin();  // use board defaults
  }
#else
  (void)sda;
  (void)scl;  // pins fixed on many MCUs (AVR/SAMD/etc.)
  _wire->begin();
#endif

  if (freqHz > 0) {
    _wire->setClock(freqHz);
  }
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission(false) == 0)
    return true;
  return false;
}

/** reset */
void INA219_7Semi::reset() {
  i2cWrite(REG_CONFIG, CFG_RST);
  delay(1);

  // Default: 32V range, PGA x8 (±320mV), 12-bit single-shot, power-down mode
  configure(false, 3, 0x03, 0x03, 0);  // user should set desired mode later
  // Clear calibration scaling
  _currentLSB_A = 0.0f;
  _powerLSB_W = 0.0f;
  _calib = 0;
}

/** configure */
void INA219_7Semi::configure(bool range16V, uint8_t pga, uint8_t badc, uint8_t sadc, uint8_t mode) {
  _configCache = makeConfig(range16V, pga, badc, sadc, mode);
  i2cWrite(REG_CONFIG, _configCache);
}

/** quick setters */
void INA219_7Semi::setBusVoltageRange16V(bool enable16V) {
  // Keep other fields; update BRNG bit
  bool range16V = enable16V;
  uint8_t pga = (uint8_t)((_configCache >> CFG_PGA_SHIFT) & 0x03);
  uint8_t badc = (uint8_t)((_configCache >> CFG_BADC_SHIFT) & 0x0F);
  uint8_t sadc = (uint8_t)((_configCache >> CFG_SADC_SHIFT) & 0x0F);
  uint8_t mode = (uint8_t)(_configCache & CFG_MODE_MASK);
  configure(range16V, pga, badc, sadc, mode);
}

void INA219_7Semi::setPGAGain(uint8_t pga) {
  bool range16V = (_configCache & CFG_BRNG_16V) != 0;
  uint8_t badc = (uint8_t)((_configCache >> CFG_BADC_SHIFT) & 0x0F);
  uint8_t sadc = (uint8_t)((_configCache >> CFG_SADC_SHIFT) & 0x0F);
  uint8_t mode = (uint8_t)(_configCache & CFG_MODE_MASK);
  configure(range16V, pga, badc, sadc, mode);
}

void INA219_7Semi::setBusADC(uint8_t badc) {
  bool range16V = (_configCache & CFG_BRNG_16V) != 0;
  uint8_t pga = (uint8_t)((_configCache >> CFG_PGA_SHIFT) & 0x03);
  uint8_t sadc = (uint8_t)((_configCache >> CFG_SADC_SHIFT) & 0x0F);
  uint8_t mode = (uint8_t)(_configCache & CFG_MODE_MASK);
  configure(range16V, pga, badc, sadc, mode);
}

void INA219_7Semi::setShuntADC(uint8_t sadc) {
  bool range16V = (_configCache & CFG_BRNG_16V) != 0;
  uint8_t pga = (uint8_t)((_configCache >> CFG_PGA_SHIFT) & 0x03);
  uint8_t badc = (uint8_t)((_configCache >> CFG_BADC_SHIFT) & 0x0F);
  uint8_t mode = (uint8_t)(_configCache & CFG_MODE_MASK);
  configure(range16V, pga, badc, sadc, mode);
}

void INA219_7Semi::setMode(uint8_t mode) {
  bool range16V = (_configCache & CFG_BRNG_16V) != 0;
  uint8_t pga = (uint8_t)((_configCache >> CFG_PGA_SHIFT) & 0x03);
  uint8_t badc = (uint8_t)((_configCache >> CFG_BADC_SHIFT) & 0x0F);
  uint8_t sadc = (uint8_t)((_configCache >> CFG_SADC_SHIFT) & 0x0F);
  configure(range16V, pga, badc, sadc, mode);
}

/** calibration - direct */
void INA219_7Semi::calibrateDirect(float currentLSB_A, float shuntOhms) {
  if (currentLSB_A <= 0.0f || shuntOhms <= 0.0f) return;

  _currentLSB_A = currentLSB_A;
  _powerLSB_W = 20.0f * _currentLSB_A;  // per datasheet
  float cal_f = CALIB_CONST / (_currentLSB_A * shuntOhms);
  _calib = (uint16_t)(cal_f + 0.5f);

  applyCalibration();
}

/** calibration - automatic */
void INA219_7Semi::calibrateAuto(float maxExpectedCurrent_A, float shuntOhms) {
  if (maxExpectedCurrent_A <= 0.0f || shuntOhms <= 0.0f) return;

  // Choose a tidy current LSB slightly larger than max/32767
  float rawLSB = maxExpectedCurrent_A / 32767.0f;

  // Round up to a "nice" number to avoid tiny floats:
  // - use 1/10000 A steps (0.1 mA) minimum granularity
  float step = 0.0001f;
  float rounded = ceilf(rawLSB / step) * step;

  calibrateDirect(rounded, shuntOhms);
}

/** raw register reads */
int16_t INA219_7Semi::readShuntVoltageRaw() {
  return i2cReadSigned(REG_SHUNT_VOLT);
}

uint16_t INA219_7Semi::readBusVoltageRaw() {
  return i2cRead(REG_BUS_VOLT);
}

int16_t INA219_7Semi::readCurrentRaw() {
  return i2cReadSigned(REG_CURRENT);
}

int16_t INA219_7Semi::readPowerRaw() {
  return (int16_t)i2cRead(REG_POWER);
}

/** engineering unit conversions */
float INA219_7Semi::readShuntVoltage() {
  int16_t raw = readShuntVoltageRaw();
  return (float)raw * (SHUNT_LSB_V * 1000.0f);
}

float INA219_7Semi::readBusVoltage() {
  uint16_t raw = readBusVoltageRaw();
  uint16_t value = (raw >> 3);  // bits [15:3]
  return (float)value * BUS_LSB_V;
}

float INA219_7Semi::readCurrent() {
  if (_calib == 0) return NAN;
  int16_t raw = readCurrentRaw();
  float current_A = (float)raw * _currentLSB_A;
  return current_A * 1000.0f;
}

float INA219_7Semi::readPower() {
  if (_calib == 0) return NAN;
  int16_t raw = readPowerRaw();
  float power_W = (float)raw * _powerLSB_W;
  return power_W * 1000.0f;
}

/** status helpers */
bool INA219_7Semi::overflow() {
  uint16_t raw = readBusVoltageRaw();
  return (raw & BUS_OVF) != 0;
}

bool INA219_7Semi::conversionReady() {
  uint16_t raw = readBusVoltageRaw();
  return (raw & BUS_CNVR) != 0;
}

/** address */
void INA219_7Semi::setAddress(uint8_t addr) {
  _addr = addr;
}

/** internals */
void INA219_7Semi::i2cWrite(uint8_t reg, uint16_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write((uint8_t)(val >> 8));
  _wire->write((uint8_t)(val & 0xFF));
  _wire->endTransmission();
}

uint16_t INA219_7Semi::i2cRead(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom((int)_addr, 2);
  uint16_t msb = _wire->read();
  uint16_t lsb = _wire->read();
  return (uint16_t)((msb << 8) | lsb);
}

int16_t INA219_7Semi::i2cReadSigned(uint8_t reg) {
  return (int16_t)i2cRead(reg);
}

void INA219_7Semi::applyCalibration() {
  if (_calib == 0) return;
  i2cWrite(REG_CALIB, _calib);
}

uint16_t INA219_7Semi::makeConfig(bool range16V, uint8_t pga, uint8_t badc, uint8_t sadc, uint8_t mode) {
  uint16_t cfg = 0;
  cfg |= range16V ? CFG_BRNG_16V : CFG_BRNG_32V;
  cfg |= ((uint16_t)(pga & 0x03)) << CFG_PGA_SHIFT;
  cfg |= ((uint16_t)(badc & 0x0F)) << CFG_BADC_SHIFT;
  cfg |= ((uint16_t)(sadc & 0x0F)) << CFG_SADC_SHIFT;
  cfg |= (uint16_t)(mode & 0x07);
  return cfg;
}