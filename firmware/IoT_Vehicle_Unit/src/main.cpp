// =====================================================
// INTELLIGENT VEHICLE SPEED LIMITING SYSTEM
// Vehicle Unit — Bluetooth RC Car + LiFi RX Decoder
// Team: Dharshnamoorthy R, Gogulakannan M,
//       Henry Daniel Didace D, Prashanth Arjun GK
// MVIT, Pondicherry University | 2025-26
// =====================================================
// CHANGES FROM PREVIOUS VERSION:
//   - checkLiFiZone() replaced with interrupt-based
//     OOK pulse decoder on GPIO 34
//   - Lookup table maps zone codes to speed limits
//   - ZA/ZB/ZC/ZX Bluetooth commands retained as
//     manual fallback during demo
//   - 2s boot guard retained
//   - 10kΩ pull-up on GPIO 34 required in hardware
// =====================================================

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

BluetoothSerial SerialBT;

// =====================================================
// LCD
// =====================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

String lastLCDLine0 = "";
String lastLCDLine1 = "";

// =====================================================
// MOTOR PINS (L298N)
// =====================================================
#define ENA 32
#define IN1 33
#define IN2 25

#define ENB 12
#define IN3 26
#define IN4 27

// =====================================================
// LIFI PIN
// =====================================================
#define LIFI_PIN 34

// =====================================================
// LORA PINS (SPI)
// =====================================================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2

// =====================================================
// LIFI — ZONE CODES (must match road unit TX)
// =====================================================
#define ZONE_EXIT 0x00
#define ZONE_A    0xA1
#define ZONE_B    0xB2
#define ZONE_C    0xC3

// =====================================================
// LIFI — PULSE TIMING THRESHOLDS (microseconds)
// START pulse  : > 700µs HIGH
// Bit 1        : > 350µs HIGH
// Bit 0        : ≤ 350µs HIGH
// Noise filter : < 50µs ignored
// =====================================================
#define LIFI_THRESH_START  700
#define LIFI_THRESH_BIT1   350
#define LIFI_THRESH_NOISE   50

// =====================================================
// LIFI — DECODER STATE (volatile — used in ISR)
// =====================================================
volatile unsigned long risingTime   = 0;   // when pulse went HIGH
volatile unsigned long pulseWidth   = 0;   // measured HIGH duration
volatile bool          pulseReady   = false; // new pulse to process

volatile byte   rxByte       = 0;    // byte being assembled
volatile int    rxBitCount   = 0;    // bits received so far
volatile bool   rxActive     = false; // START pulse seen
volatile byte   lifiReceived = 0xFF; // last complete valid byte (0xFF = none)
volatile bool   lifiNewData  = false; // flag for main loop

// =====================================================
// LIFI — INTERRUPT SERVICE ROUTINE
// Fires on every CHANGE (RISING + FALLING edge)
// =====================================================
void IRAM_ATTR lifiISR() {

  unsigned long now = micros();

  // RISING edge — record start time
  if (digitalRead(LIFI_PIN) == HIGH) {
    risingTime = now;
    return;
  }

  // FALLING edge — measure pulse width
  pulseWidth = now - risingTime;

  // Ignore noise
  if (pulseWidth < LIFI_THRESH_NOISE) return;

  // START pulse — begin new byte
  if (pulseWidth > LIFI_THRESH_START) {
    rxByte     = 0;
    rxBitCount = 0;
    rxActive   = true;
    return;
  }

  // Data bit — only if START seen
  if (!rxActive) return;

  // Shift in bit (MSB first)
  rxByte <<= 1;
  if (pulseWidth > LIFI_THRESH_BIT1) {
    rxByte |= 1;   // Bit 1
  }
  // else Bit 0 — already shifted in as 0

  rxBitCount++;

  // Full byte received (8 bits)
  if (rxBitCount == 8) {
    lifiReceived = rxByte;
    lifiNewData  = true;
    rxActive     = false;
    rxBitCount   = 0;
    rxByte       = 0;
  }
}

// =====================================================
// SPEED SETTINGS
// =====================================================
int manualSpeed = 180;
int speedValue  = 180;
int zoneSpeed   = 75;

// =====================================================
// MOVEMENT FLAGS
// =====================================================
bool fwd   = false;
bool back  = false;
bool left  = false;
bool right = false;

// =====================================================
// ZONE & TX STATE
// =====================================================
bool inZone   = false;
bool txActive = false;

// =====================================================
// LIFI TIMER (for fail-safe exit)
// =====================================================
unsigned long lastLiFiTime = 0;
const unsigned long LIFI_TIMEOUT = 3000;

// =====================================================
// LORA TIMER
// =====================================================
unsigned long lastLoRaSend = 0;
const unsigned long LORA_INTERVAL = 3000;

// =====================================================
// OVERRIDE SYSTEM
// =====================================================
bool overrideActive         = false;
bool overrideCooldownActive = false;
bool overrideLocked         = false;

unsigned long overrideStart         = 0;
unsigned long overrideCooldownStart = 0;
unsigned long overrideLockStart     = 0;

const unsigned long OVERRIDE_DURATION = 20000;
const unsigned long OVERRIDE_COOLDOWN = 100000;
const unsigned long OVERRIDE_LOCKTIME = 1200000;

int overrideCount = 0;
const int OVERRIDE_MAX = 5;

// =====================================================
// LCD SCREEN TOGGLE
// =====================================================
bool showStatusScreen = false;

// =====================================================
// LCD UPDATE — NO FLICKER
// =====================================================
void updateLCD() {

  String line0 = "";
  String line1 = "";

  if (showStatusScreen) {

    line0 = "OVR:";
    line0 += overrideCount;
    line0 += "/";
    line0 += OVERRIDE_MAX;
    line0 += "  ";
    line0 += overrideActive ? "ACT " : "IDLE";

    if (overrideCooldownActive) {
      unsigned long remain =
        (OVERRIDE_COOLDOWN -
         (millis() - overrideCooldownStart)) / 1000;
      line1 = "CD:";
      line1 += remain;
      line1 += "s          ";
    }
    else if (overrideLocked) {
      unsigned long remain =
        (OVERRIDE_LOCKTIME -
         (millis() - overrideLockStart)) / 60000;
      line1 = "LOCK:";
      line1 += remain;
      line1 += "m         ";
    }
    else {
      line1 = "READY           ";
    }
  }
  else {

    line0 = "SPD:";
    line0 += speedValue;
    line0 += "    ";
    line0 = line0.substring(0, 10);
    line0 += inZone ? "ZONE" : "FREE";
    line0 += "  ";

    line1 = "TX:";
    line1 += txActive ? "ON  " : "OFF ";
    line1 += "        ";
    line1 = line1.substring(0, 8);
    line1 += overrideActive ? "OVR " : "    ";
    line1 += inZone ? "Z" : " ";
  }

  if (line0 != lastLCDLine0) {
    lcd.setCursor(0, 0);
    lcd.print(line0.substring(0, 16));
    lastLCDLine0 = line0;
  }

  if (line1 != lastLCDLine1) {
    lcd.setCursor(0, 1);
    lcd.print(line1.substring(0, 16));
    lastLCDLine1 = line1;
  }
}

// =====================================================
// MOTOR CONTROL
// =====================================================
void moveCar() {

  int leftSpeed  = speedValue;
  int rightSpeed = speedValue;

  int in1 = LOW, in2 = LOW;
  int in3 = LOW, in4 = LOW;

  if (fwd && !back) {
    in1 = HIGH; in2 = LOW;
    in3 = HIGH; in4 = LOW;
  }
  else if (back && !fwd) {
    in1 = LOW; in2 = HIGH;
    in3 = LOW; in4 = HIGH;
  }

  if (left && (fwd || back)) {
    leftSpeed /= 2;
  }
  else if (right && (fwd || back)) {
    rightSpeed /= 2;
  }
  else if (left && !fwd && !back) {
    in1 = LOW;  in2 = HIGH;
    in3 = HIGH; in4 = LOW;
  }
  else if (right && !fwd && !back) {
    in1 = HIGH; in2 = LOW;
    in3 = LOW;  in4 = HIGH;
  }

  if (!fwd && !back && !left && !right) {
    leftSpeed  = 0;
    rightSpeed = 0;
  }

  digitalWrite(IN1, in1);
  digitalWrite(IN2, in2);
  digitalWrite(IN3, in3);
  digitalWrite(IN4, in4);

  ledcWrite(0, leftSpeed);
  ledcWrite(1, rightSpeed);

  updateLCD();
}

// =====================================================
// APPLY SPEED LOGIC
// Priority: Override > Zone (capped) > Manual
// =====================================================
void applySpeedLogic() {

  if (overrideActive) {
    speedValue = manualSpeed;
  }
  else if (inZone) {
    speedValue = min(manualSpeed, zoneSpeed);
    if (manualSpeed > zoneSpeed) {
      Serial.print("[SPD] Capped to zone limit: ");
      Serial.println(zoneSpeed);
    }
  }
  else {
    speedValue = manualSpeed;
  }

  moveCar();
}

// =====================================================
// OVERRIDE — ACTIVATE
// =====================================================
void activateOverride() {

  if (!inZone) {
    Serial.println("[OVR] Only inside zone");
    return;
  }
  if (overrideLocked) {
    Serial.println("[OVR] LOCKED");
    return;
  }
  if (overrideCooldownActive) {
    Serial.println("[OVR] Cooldown active");
    return;
  }

  overrideActive = true;
  overrideStart  = millis();
  overrideCount++;

  Serial.print("[OVR] ACTIVATED (");
  Serial.print(overrideCount);
  Serial.print("/");
  Serial.print(OVERRIDE_MAX);
  Serial.println(")");

  applySpeedLogic();

  if (overrideCount >= OVERRIDE_MAX) {
    overrideLocked    = true;
    overrideLockStart = millis();
    Serial.println("[OVR] SYSTEM LOCKED — 20 min");
  }
}

// =====================================================
// OVERRIDE — TIMER HANDLER
// =====================================================
void handleOverride() {

  if (overrideActive &&
      millis() - overrideStart >= OVERRIDE_DURATION) {
    overrideActive         = false;
    overrideCooldownActive = true;
    overrideCooldownStart  = millis();
    Serial.println("[OVR] Ended — Cooldown started");
    applySpeedLogic();
  }

  if (overrideCooldownActive &&
      millis() - overrideCooldownStart >= OVERRIDE_COOLDOWN) {
    overrideCooldownActive = false;
    Serial.println("[OVR] Cooldown cleared");
  }

  if (overrideLocked &&
      millis() - overrideLockStart >= OVERRIDE_LOCKTIME) {
    overrideLocked = false;
    overrideCount  = 0;
    Serial.println("[OVR] Lock cleared");
  }
}

// =====================================================
// LIFI — ZONE LOOKUP TABLE
// Called from main loop when lifiNewData is set
// =====================================================
void processLiFiData() {

  // Boot guard — ignore first 2 seconds
  if (millis() < 2000) {
    lifiNewData = false;
    return;
  }

  // Read volatile data safely
  noInterrupts();
  byte received = lifiReceived;
  lifiNewData   = false;
  interrupts();

  Serial.print("[LIFI] RX byte: 0x");
  if (received < 0x10) Serial.print("0");
  Serial.println(received, HEX);

  // Lookup table
  switch (received) {

    case ZONE_A:
      zoneSpeed    = 75;
      lastLiFiTime = millis();
      if (!inZone) {
        inZone = true;
        Serial.println("[LIFI] ENTER Zone A — LIM 75");
        applySpeedLogic();
      }
      break;

    case ZONE_B:
      zoneSpeed    = 102;
      lastLiFiTime = millis();
      if (!inZone) {
        inZone = true;
        Serial.println("[LIFI] ENTER Zone B — LIM 102");
        applySpeedLogic();
      }
      break;

    case ZONE_C:
      zoneSpeed    = 128;
      lastLiFiTime = millis();
      if (!inZone) {
        inZone = true;
        Serial.println("[LIFI] ENTER Zone C — LIM 128");
        applySpeedLogic();
      }
      break;

    case ZONE_EXIT:
      if (inZone) {
        inZone         = false;
        overrideActive = false;
        Serial.println("[LIFI] EXIT Zone received");
        applySpeedLogic();
      }
      break;

    default:
      // Unknown byte — noise or corrupt packet, ignore
      Serial.print("[LIFI] Unknown code: 0x");
      Serial.println(received, HEX);
      break;
  }
}

// =====================================================
// LIFI — FAIL-SAFE CHECK
// If signal lost for > 3s while in zone → exit zone
// =====================================================
void checkLiFiFailSafe() {

  if (!inZone) return;
  if (overrideActive) return;  // don't exit during override

  if (millis() - lastLiFiTime > LIFI_TIMEOUT) {
    inZone         = false;
    overrideActive = false;
    Serial.println("[LIFI] Signal lost — EXIT ZONE (fail-safe)");
    applySpeedLogic();
  }
}

// =====================================================
// BLUETOOTH — NON-BLOCKING READ
// =====================================================
void handleBluetooth() {

  if (!SerialBT.available()) return;

  String cmd = "";
  unsigned long timeout = millis();

  while (millis() - timeout < 50) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '\n' || c == '\r') break;
      cmd += c;
    }
  }

  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.println("[BT] CMD: " + cmd);

  // ---- MOVEMENT ----
  if      (cmd == "F") { fwd = true;  back = false; }
  else if (cmd == "B") { back = true; fwd  = false; }
  else if (cmd == "L") { left = true;  }
  else if (cmd == "R") { right = true; }
  else if (cmd == "S") {
    fwd = false; back = false;
    left = false; right = false;
  }

  // ---- SPEED ----
  else if (cmd == "U") {
    manualSpeed += 5;
    if (manualSpeed > 255) manualSpeed = 255;
    Serial.print("[SPD] Manual: "); Serial.println(manualSpeed);
    applySpeedLogic();
    return;
  }
  else if (cmd == "D") {
    manualSpeed -= 5;
    if (manualSpeed < 0) manualSpeed = 0;
    Serial.print("[SPD] Manual: "); Serial.println(manualSpeed);
    applySpeedLogic();
    return;
  }

  // ---- MANUAL ZONE (fallback for demo) ----
  else if (cmd == "ZA") {
    zoneSpeed    = 75;
    inZone       = true;
    lastLiFiTime = millis();
    Serial.println("[ZONE] Manual ENTER Zone A — LIM 75");
    applySpeedLogic();
    return;
  }
  else if (cmd == "ZB") {
    zoneSpeed    = 102;
    inZone       = true;
    lastLiFiTime = millis();
    Serial.println("[ZONE] Manual ENTER Zone B — LIM 102");
    applySpeedLogic();
    return;
  }
  else if (cmd == "ZC") {
    zoneSpeed    = 128;
    inZone       = true;
    lastLiFiTime = millis();
    Serial.println("[ZONE] Manual ENTER Zone C — LIM 128");
    applySpeedLogic();
    return;
  }
  else if (cmd == "ZX") {
    inZone         = false;
    overrideActive = false;
    Serial.println("[ZONE] Manual EXIT Zone");
    applySpeedLogic();
    return;
  }

  // ---- OVERRIDE ----
  else if (cmd == "O") {
    activateOverride();
    return;
  }

  // ---- STATUS SCREEN ----
  else if (cmd == "ST") {
    showStatusScreen = !showStatusScreen;
    updateLCD();
    return;
  }

  else {
    Serial.println("[BT] Unknown cmd: " + cmd);
    return;
  }

  moveCar();
}

// =====================================================
// LORA TRANSMIT
// =====================================================
void sendLoRa() {

  if (millis() - lastLoRaSend < LORA_INTERVAL) return;
  lastLoRaSend = millis();

  txActive = true;
  updateLCD();

  String msg = "";

  if (inZone) {
    msg += "ZONE:IN";
    msg += ",SPD:"; msg += speedValue;
    msg += ",LIM:"; msg += zoneSpeed;
    msg += ",OVR:"; msg += (overrideActive ? "1" : "0");
    msg += ",OVRC:"; msg += overrideCount;
  }
  else {
    msg += "ZONE:OUT";
    msg += ",SPD:"; msg += speedValue;
    msg += ",OVR:0";
  }

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println("[LORA] TX: " + msg);

  txActive = false;
  updateLCD();
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  // ---------------- BLUETOOTH ----------------
  SerialBT.begin("ESP32_RC_CAR");
  Serial.println("[BT] Started: ESP32_RC_CAR");

  // ---------------- LCD ----------------
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SYSTEM INIT...");
  delay(1000);
  lcd.clear();
  lastLCDLine0 = "";
  lastLCDLine1 = "";

  // ---------------- MOTOR PINS ----------------
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // ---------------- LIFI RX ----------------
  // NOTE: 10kΩ pull-up from 3.3V to GPIO 34 required
  // GPIO 34 is input-only, no internal pull-up
  pinMode(LIFI_PIN, INPUT);

  // Attach interrupt — fires on every edge (RISING + FALLING)
  attachInterrupt(digitalPinToInterrupt(LIFI_PIN),
                  lifiISR, CHANGE);

  Serial.println("[LIFI] Interrupt attached on GPIO 34");

  // ---------------- PWM (LEDC) ----------------
  ledcSetup(0, 1000, 8);
  ledcAttachPin(ENA, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(ENB, 1);

  // ---------------- LORA ----------------
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("[LORA] INIT FAILED");
    lcd.setCursor(0, 0);
    lcd.print("LORA FAIL!      ");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("[LORA] Ready at 433 MHz");

  updateLCD();
  Serial.println("[SYS] Vehicle unit online");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  handleBluetooth();      // BT commands — non-blocking

  if (lifiNewData) {      // LiFi — process decoded byte
    processLiFiData();
  }

  checkLiFiFailSafe();    // Fail-safe — exit zone if signal lost

  handleOverride();       // Override timer management

  sendLoRa();             // LoRa telemetry — non-blocking
}