# Intelligent-Vehicle-Speed-Limiting-And-Monitoring-System-Using-Hybrid-IR-LiFi-LoRa-Communication

> An IoT-based system that automatically enforces vehicle speed limits in restricted zones — without driver compliance or network infrastructure.

---

## 📌 Overview

This project implements a smart roadside-to-vehicle communication system that detects approaching vehicles, transmits the permissible speed limit via optical LiFi, enforces it automatically on the vehicle, and reports real-time telemetry to a remote monitoring dashboard via LoRa.

Designed for restricted zones such as **schools, hospitals, and construction sites**, the system operates entirely without cellular networks or internet infrastructure.

---

## 🏫 Academic Details

| Field | Details |
|---|---|
| Institution | Manakula Vinayagar Institute of Technology (MVIT) |
| University | Pondicherry University |
| Department | Information Technology |
| Academic Year | 2025–26 |
| Guide | Mrs. V. Vimala Dheekshanya, Asst. Professor, Dept. of IT |

### Team Members

| Name | Roll Number |
|---|---|
| Dharshnamoorthy R | 22TH0223 |
| Gogulakannan M | 22TH0228 |
| Henry Daniel Didace D | 22TH0232 |
| Prashanth Arjun GK | 22TH0276 |

---

## 🔧 System Architecture

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│         ROAD UNIT           │         │        VEHICLE UNIT          │
│                             │         │                              │
│  IR Sensor → ESP32          │──LiFi──▶│  Phototransistor ×3 → ESP32 │
│  LED ×5 + BC547 (LiFi TX)  │         │  L298N → DC Motors ×4        │
│  LoRa SX1278 (RX)          │◀─LoRa──│  LoRa SX1278 (TX)           │
│  ESP32 Web Server           │         │  I2C LCD 16×2                │
│  Blynk IoT (Zone Select)    │         │  Bluetooth RC Control        │
│  12V DC Adapter             │         │  4S 18650 Battery Pack       │
└─────────────────────────────┘         └─────────────────────────────┘
         │                                          │
       WiFi                                     Bluetooth
         │                                          │
   Browser Dashboard                          Mobile App (BLE)
```

### Communication Channels

| Channel | Technology | Direction | Purpose |
|---|---|---|---|
| Detection | IR Sensor | Sensor → ESP32 | Vehicle presence detection |
| Zone Data | LiFi (OOK) | Road → Vehicle | Speed limit transmission |
| Telemetry | LoRa 433MHz | Vehicle → Road | Real-time speed/status reporting |
| Control | Bluetooth | App → Vehicle | Manual RC car control |
| Zone Select | Blynk / WiFi | App → Road unit | Zone configuration |
| Dashboard | HTTP (WebServer) | Road unit → Browser | Live monitoring |

---

## ⚙️ How It Works

1. Operator selects active zone (A / B / C) via **Blynk app**
2. **IR sensor** detects approaching vehicle → triggers LiFi TX
3. **LiFi transmitter** (5× LEDs + BC547) sends zone code via OOK pulse encoding
4. **Phototransistor array** on vehicle decodes the optical signal via ISR
5. Vehicle ESP32 looks up zone code → enforces speed limit via **PWM capping**
6. Driver can slow down freely but **cannot exceed zone limit**
7. Emergency **override** available for 20 seconds (with cooldown + lock-out)
8. **LoRa** transmits telemetry every 3 seconds to road unit
9. Road unit hosts a **live web dashboard** showing speed, zone, override status

---

## 📁 Repository Structure

```
intelligent-speed-limiting-system/
│
├── firmware/
│   ├── vehicle_unit.cpp        # Vehicle ESP32 — BT + LiFi RX + LoRa TX
│   ├── road_unit.cpp           # Road ESP32 — IR + LiFi TX + LoRa RX + Dashboard
│   └── lifi_tx_test.cpp        # Standalone LiFi TX test sketch
│
├── docs/
│   ├── project_summary.docx    # Full project summary document
│   └── phase2_report.pdf       # Phase 2 academic report
│
├── dashboard/
│   └── index.html              # Web dashboard (also embedded in road unit)
│
├── diagrams/
│   ├── architecture.png        # System architecture diagram
│   └── workflow.png            # System workflow diagram
│
├── .gitignore
├── LICENSE
└── README.md
```

---

## 🛠️ Tech Stack

**Firmware & Embedded**
- Language: C++ (Arduino Framework)
- IDE: VS Code + PlatformIO
- PWM: ESP32 LEDC
- ISR: ESP32 IRAM_ATTR interrupt (LiFi RX decoding)

**Communication**
- LiFi: OOK (On-Off Keying) pulse encoding — 433nm optical
- LoRa: SX1278 @ 433MHz, SF7, 125kHz BW
- Bluetooth: Classic SPP via BluetoothSerial
- WiFi: 802.11 b/g/n (HTTP WebServer + Blynk)

**Libraries**
- `LoRa` — SX1278 packet communication
- `BluetoothSerial` — Classic BT serial
- `LiquidCrystal_I2C` — I2C LCD control
- `BlynkSimpleEsp32` — Blynk IoT platform
- `WebServer` — ESP32 HTTP server

**Frontend**
- HTML · CSS (embedded in ESP32 flash)

**IoT Platform**
- Blynk IoT (zone selection — road unit)

---

## 🔌 Pin Mapping

### Vehicle Unit (ESP32)

| GPIO | Connected To |
|---|---|
| GPIO 32 | L298N ENA (PWM — left motors) |
| GPIO 33 | L298N IN1 |
| GPIO 25 | L298N IN2 |
| GPIO 12 | L298N ENB (PWM — right motors) |
| GPIO 26 | L298N IN3 |
| GPIO 27 | L298N IN4 |
| GPIO 34 | Phototransistor array (LiFi RX) |
| GPIO 5  | LoRa NSS/CS |
| GPIO 14 | LoRa RST |
| GPIO 2  | LoRa DIO0 |
| GPIO 18 | LoRa SCK |
| GPIO 19 | LoRa MISO |
| GPIO 23 | LoRa MOSI |
| GPIO 21 | LCD SDA |
| GPIO 22 | LCD SCL |

### Road Unit (ESP32)

| GPIO | Connected To |
|---|---|
| GPIO 4  | IR Sensor OUT |
| GPIO 13 | BC547 Base (LiFi TX via 1kΩ) |
| GPIO 5  | LoRa NSS/CS |
| GPIO 14 | LoRa RST |
| GPIO 2  | LoRa DIO0 |
| GPIO 18 | LoRa SCK |
| GPIO 19 | LoRa MISO |
| GPIO 23 | LoRa MOSI |

---

## 🚀 Getting Started

### Prerequisites
- VS Code + PlatformIO extension installed
- ESP32 board support added in PlatformIO
- Required libraries installed (see `platformio.ini`)

### Flash Vehicle Unit
```bash
cd firmware
pio run -e vehicle_unit -t upload
```

### Flash Road Unit
```bash
cd firmware
pio run -e road_unit -t upload
```

### Access Dashboard
1. Power on road unit
2. Connect your device to the same WiFi hotspot
3. Open Serial Monitor → note the IP address printed
4. Open browser → `http://<IP_ADDRESS>`

---

## 📱 Bluetooth Commands (Vehicle Unit)

| Command | Action |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Left |
| `R` | Right |
| `S` | Stop |
| `U` | Speed up (+5) |
| `D` | Speed down (-5) |
| `ZA` | Enter Zone A (limit 75) |
| `ZB` | Enter Zone B (limit 102) |
| `ZC` | Enter Zone C (limit 128) |
| `ZX` | Exit zone |
| `O` | Emergency override (20s) |
| `ST` | Toggle status screen |

---

## 🔒 Override State Machine

```
IDLE ──(O cmd)──▶ ACTIVE (20s)
                      │
                  (expires)
                      │
                      ▼
                COOLDOWN (100s)
                      │
                  (clears)
                      │
                      ▼
                    IDLE
                      │
              (5 uses total)
                      │
                      ▼
                LOCKED (20 min)
                      │
                  (expires)
                      │
                      ▼
                    IDLE
```

---

## 🌍 SDG Mapping

| SDG | Goal | Relevance |
|---|---|---|
| SDG 3 | Good Health & Well-Being | Reduces road accidents in high-risk zones |
| SDG 9 | Industry, Innovation & Infrastructure | Affordable IoT-based smart road system |
| SDG 11 | Sustainable Cities & Communities | Safer, smarter urban mobility |

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

*Manakula Vinayagar Institute of Technology · Department of Information Technology · 2025–26*
