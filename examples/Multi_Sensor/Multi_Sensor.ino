/***************************************************************
 * @file    Multi_Sensor.ino
 * @brief   Example showing use of a for-loop to manage three
 *          7Semi INA219 sensors on one I2C bus.
 *
 * Features demonstrated:
 * - Single I2C bus shared by three INA219 devices
 * - Initialization, configuration and calibration in loop
 * - Unified read-and-print cycle for all sensors
 *
 * Sensor Configuration:
 * - Bus Range : 32 V (BRNG=0)
 * - Shunt PGA : ±320 mV (PGA=3)
 * - ADC       : 12-bit x16 averaging (code 0x0B)
 * - Mode      : Continuous Shunt + Bus (0x07)
 *
 * Library   : 7Semi_INA219
 * Author    : 7Semi
 * Version   : 1.0
 * Date      : 04 October 2025
 * License   : MIT
 ***************************************************************/

#include <Wire.h>
#include <7Semi_INA219.h>

// three INA219 addresses — change to match your ADDR straps
uint8_t addresses[3] = { 0x40, 0x41, 0x42 };

// create 3 device objects
INA219_7Semi ina[3] = { INA219_7Semi(addresses[0]),
                        INA219_7Semi(addresses[1]),
                        INA219_7Semi(addresses[2]) };

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n7Semi INA219 Triple-Loop Demo"));

  Wire.begin();

  // common config parameters
  bool range16V = false;  // 32V range
  uint8_t pga = 3;        // ±320 mV
  uint8_t badc = 0x0B;    // 12-bit x16 avg
  uint8_t sadc = 0x0B;    // 12-bit x16 avg
  uint8_t mode = 0x07;    // continuous shunt+bus
  float shunt_Ohms = 0.1f;
  float maxExpected_A = 2.0f;

  // initialize, configure, and calibrate all sensors
  for (uint8_t i = 0; i < 3; i++) {
    if (!ina[i].begin(&Wire)) {
      Serial.print(F("ERROR: INA219 @0x"));
      Serial.println(addresses[i], HEX);
      while (1) { delay(1000); }
    }
    ina[i].configure(range16V, pga, badc, sadc, mode);
    ina[i].calibrateAuto(maxExpected_A, shunt_Ohms);
    Serial.print(F("INA219 @0x"));
    Serial.print(addresses[i], HEX);
    Serial.println(F(" ready."));
  }
  Serial.println();
}

void loop() {
  // read and print all three devices in a loop
  for (uint8_t index = 0; index < 3; index++) {
    INA219_7Semi& dev = ina[index];
    if (!dev.conversionReady()) return;

    float vBus_V = dev.readBusVoltage();
    float vShunt_mV = dev.readShuntVoltage();
    float current_mA = dev.readCurrent();
    float power_mW = dev.readPower();
    bool ovf = dev.overflow();

    Serial.print(F("[INA"));
    Serial.print(index + 1);
    Serial.print(F(" @0x"));
    Serial.print(addresses[index], HEX);
    Serial.print(F("]  Bus: "));
    Serial.print(vBus_V, 3);
    Serial.print(F(" V  Shunt: "));
    Serial.print(vShunt_mV, 3);
    Serial.print(F(" mV  Curr: "));
    Serial.print(current_mA, 2);
    Serial.print(F(" mA  Power: "));
    Serial.print(power_mW, 1);
    Serial.print(F(" mW"));
    if (ovf) Serial.print(F("  [OVF]"));
    Serial.println();
  }
  Serial.println();
  delay(500);
}
