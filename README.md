# 7Semi_INA219_Arduino_Library

This Arduino library provides support for the **7Semi INA219 Current, Voltage & Power Monitor** — a high-accuracy I²C sensor designed for bidirectional current, voltage, and power measurement.  
It is ideal for energy monitoring, battery management, and power analysis in embedded and IoT systems.

![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## 🌐 Features

- Supports **I²C** communication (standard 100kHz and fast 400kHz)
- Measures:
  - **Bus Voltage (V)**
  - **Shunt Voltage (mV)**
  - **Current (mA)**
  - **Power (mW)**
- **Configurable ranges** for 16V/32V, programmable gain amplifier (PGA), and ADC resolution
- **Status flags** for conversion-ready and overflow
- Compatible with:
  - **3.3V and 5V** systems
  - **AVR**, **ESP32/ESP8266** 
- Lightweight, efficient, and type-safe API

---

## 🔧 Hardware Required

| Component | Description |
|------------|-------------|
| 7Semi INA219 Current/Voltage/Power Sensor | Core sensor module |
| Arduino-compatible board | UNO, MEGA, ESP32, ESP8266 |
| I²C connections | SDA, SCL |
| Power supply | 3.3V or 5V |

### Typical Wiring

| INA219 Pin | Arduino UNO | ESP32 |
|-------------|--------------|-------|
| VCC | 5V | 5V |
| GND | GND | GND |
| SDA | A4 | D21 |
| SCL | A5 | D22 |

---

## 🚀 Getting Started

### 1. Installation via Arduino Library Manager

1. Open the Arduino IDE  
2. Navigate to:  
   - **Sketch → Include Library → Manage Libraries…** (IDE 1.x)  
   - or use the **Library Manager sidebar** in IDE 2.x  
3. Search for: **7Semi_INA219_Arduino_Library**  
4. Click **Install**

Alternatively, clone or download this repository and place it inside your Arduino **libraries/** folder.

---

