/***************************************************************
 * @file    Basic.ino
 * @brief   Single-device example for the 7Semi INA219 current/
 *          power monitor over I2C.
 *
 * Features demonstrated:
 * - Initialization of one INA219
 * - Configuration of bus range, PGA, ADC res/averaging and mode
 * - Auto calibration using shunt ohms and max expected current
 * - Reading Bus V (V), Shunt V (mV), Current (mA), Power (mW)
 * - Overflow and conversion-ready checks
 *
 * Sensor Configuration:
 * - Bus Range : 32 V (BRNG=0)
 * - Shunt PGA : ±320 mV (PGA=3)
 * - ADC       : 12-bit with 16x averaging (0x0B) for bus & shunt
 * - Mode      : Continuous Shunt + Bus (MODE=0b111)
 *
 * ADC Configuration Table (BADC / SADC codes)
 * -------------------------------------------
 * - 0x00 →  9-bit     (84 µs)
 * - 0x01 → 10-bit    (148 µs)
 * - 0x02 → 11-bit    (276 µs)
 * - 0x03 → 12-bit    (532 µs)
 * - 0x08 → 12-bit x2    (1.06 ms)
 * - 0x09 → 12-bit x4    (2.13 ms)
 * - 0x0A → 12-bit x8    (4.26 ms)
 * - 0x0B → 12-bit x16   (8.51 ms)
 * - 0x0C → 12-bit x32   (17.02 ms)
 * - 0x0D → 12-bit x64   (34.05 ms)
 * - 0x0E → 12-bit x128  (68.10 ms)
 *
 * Mode Bits Table (MODE[2:0])
 * ---------------------------
 * - 000 → Power-down
 * - 001 → Shunt voltage, triggered
 * - 010 → Bus voltage, triggered
 * - 011 → Shunt + Bus, triggered
 * - 100 → ADC off (disabled)
 * - 101 → Shunt voltage, continuous
 * - 110 → Bus voltage, continuous
 * - 111 → Shunt + Bus, continuous
 *
 * Connections:
 * - VIN+ / VIN-  → in series with load (observe polarity)
 * - VCC          → 3.3V / 5V
 * - GND          → GND
 * - SDA          → A4 (Uno) / board SDA
 * - SCL          → A5 (Uno) / board SCL
 * - ADDR         → strap for 0x40..0x4F (example uses 0x40)
 *
 * Library   : 7Semi_INA219
 * Author    : 7Semi
 * Version   : 1.0
 * Date      : 04 October 2025
 * License   : MIT
 ***************************************************************/

#include <7Semi_INA219.h>

INA219_7Semi ina(0x40);   // single device @ 0x40

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n7Semi INA219 Single Device Demo"));

  if (!ina.begin(&Wire)) {
    // if (!ina.beginStrict(&Wire, 21, 22, 400000)) {
    Serial.println(F("ERROR: INA219 @0x40 not found"));
    while (1) { delay(1000); }
  }

  // Config: 32V, ±320mV, 12-bit x16 avg, continuous shunt+bus
  bool    range16V = false; // true: 16V, false: 32V
  uint8_t pga      = 3;
  uint8_t badc     = 0x0B;
  uint8_t sadc     = 0x0B;
  uint8_t mode     = 0x07;
  ina.configure(range16V, pga, badc, sadc, mode);

  // Auto calibration: set per your shunt & range
  float maxExpected_A = 2.0f;
  float shunt_Ohms    = 0.1f;
  ina.calibrateAuto(maxExpected_A, shunt_Ohms);

  Serial.println(F("Single INA219 ready.\n"));
}

void loop() {
  if (ina.conversionReady()) {
    float vBus_V     = ina.readBusVoltage();
    float vShunt_mV  = ina.readShuntVoltage();
    float current_mA = ina.readCurrent();
    float power_mW   = ina.readPower();
    bool  ovf        = ina.overflow();

    Serial.print(F("Bus: "));
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

  delay(500);
}
