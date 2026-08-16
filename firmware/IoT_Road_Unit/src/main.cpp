// =====================================================
// INTELLIGENT VEHICLE SPEED LIMITING SYSTEM
// Road Unit — IR + LiFi TX + LoRa RX + Web Dashboard
// Team: Dharshnamoorthy R, Gogulakannan M,
//       Henry Daniel Didace D, Prashanth Arjun GK
// MVIT, Pondicherry University | 2025-26
// =====================================================
// PIN SUMMARY:
//   LoRa SX1278 : SS=5, RST=14, DIO0=2
//                 SCK=18, MISO=19, MOSI=23
//   IR Sensor   : GPIO 4
//   LiFi TX     : GPIO 13 (BC547 base via 1kΩ)
//   WiFi        : Mobile Hotspot (Blynk + Web Server)
// =====================================================
// BLYNK VIRTUAL PINS:
//   V0 — Zone selector (0=Exit, 1=A, 2=B, 3=C)
// =====================================================
// FIXES APPLIED:
//   1. Inter-packet gap added to lifiSendByte()
//   2. Dashboard zone limit shows Blynk selection
//      before first LoRa packet arrives
//   3. RSSI logged on every LoRa RX
//   4. WiFi auto-reconnect in loop()
// =====================================================

// -------- BLYNK --------
#define BLYNK_TEMPLATE_ID   "TMPL3WGA3ObfN"
#define BLYNK_TEMPLATE_NAME "Road Unit"
#define BLYNK_AUTH_TOKEN    "ROpTOj95yRzj6UN1nL01mnsQoWp19Cy2"

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <LoRa.h>
#include <WebServer.h>

// =====================================================
// WIFI CREDENTIALS
// =====================================================
char ssid[] = "Embedded";
char pass[] = "12348765";

// =====================================================
// LORA PINS
// =====================================================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2

// =====================================================
// IR + LIFI PINS
// =====================================================
#define IR_PIN   4
#define LIFI_PIN 13

// =====================================================
// LIFI PULSE ENCODING — OOK
// Bit 1 = 500µs HIGH + 500µs LOW
// Bit 0 = 200µs HIGH + 800µs LOW
// Start = 1000µs HIGH + 500µs LOW
// Gap   = 500µs LOW between packets
// =====================================================
#define LIFI_BIT1_HIGH    500
#define LIFI_BIT1_LOW     500
#define LIFI_BIT0_HIGH    200
#define LIFI_BIT0_LOW     800
#define LIFI_START_HIGH  1000
#define LIFI_START_LOW    500
#define LIFI_PKT_GAP      500   // FIX 1: inter-packet gap
#define LIFI_REPEAT_MS    200   // Repeat every 200ms

// =====================================================
// ZONE CODES
// Vehicle lookup table must match these exactly
// =====================================================
#define ZONE_EXIT 0x00   // Exit zone
#define ZONE_A    0xA1   // Speed limit 75
#define ZONE_B    0xB2   // Speed limit 102
#define ZONE_C    0xC3   // Speed limit 128

// =====================================================
// ZONE STATE
// =====================================================
int  selectedZone   = 0;
byte zoneCode       = ZONE_EXIT;
int  zoneLimitValue = 0;

bool vehicleDetected = false;
bool lifiActive      = false;

// =====================================================
// IR DETECTION
// =====================================================
unsigned long irDetectTime  = 0;
const unsigned long IR_HOLD = 5000;

// =====================================================
// LIFI TX TIMER
// =====================================================
unsigned long lastLiFiTx = 0;

// =====================================================
// LORA RX — LATEST DATA
// =====================================================
String loraZone     = "---";
int    loraSpeed    = 0;
int    loraLimit    = 0;
int    loraRssi     = 0;
bool   loraOverride = false;
int    loraOvrCount = 0;
String loraLastMsg  = "---";
unsigned long loraLastRx = 0;

// =====================================================
// WIFI RECONNECT TIMER
// =====================================================
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_MS = 5000;

// =====================================================
// WEB SERVER
// =====================================================
WebServer server(80);

// =====================================================
// DASHBOARD HTML
// =====================================================
String buildDashboard() {

  String uptime = String(millis() / 1000) + "s";

  String zoneLabel = "---";
  if      (selectedZone == 1) zoneLabel = "Zone A";
  else if (selectedZone == 2) zoneLabel = "Zone B";
  else if (selectedZone == 3) zoneLabel = "Zone C";
  else                        zoneLabel = "No Zone";

  String vehicleStatus = vehicleDetected ? "DETECTED" : "CLEAR";
  String lifiStatus    = lifiActive      ? "ACTIVE"   : "IDLE";
  String ovrStatus     = loraOverride    ? "YES"      : "NO";

  // FIX 2: show Blynk zone limit before first LoRa packet
  String limitDisplay = "---";
  if      (loraLimit > 0)      limitDisplay = String(loraLimit);
  else if (zoneLimitValue > 0) limitDisplay = String(zoneLimitValue);

  // Time since last LoRa packet
  String lastSeen = "---";
  if (loraLastRx > 0) {
    unsigned long ago = (millis() - loraLastRx) / 1000;
    lastSeen = String(ago) + "s ago";
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <meta http-equiv='refresh' content='3'>
  <title>Speed Limit Monitor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', sans-serif;
      background: #0f1117;
      color: #e0e0e0;
      padding: 20px;
    }
    h1 {
      text-align: center;
      font-size: 1.4rem;
      color: #ffffff;
      margin-bottom: 6px;
      letter-spacing: 1px;
    }
    .subtitle {
      text-align: center;
      font-size: 0.75rem;
      color: #555;
      margin-bottom: 20px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 14px;
      max-width: 720px;
      margin: 0 auto 20px auto;
    }
    .card {
      background: #1a1d27;
      border: 1px solid #2a2d3e;
      border-radius: 10px;
      padding: 16px 12px;
      text-align: center;
    }
    .card .label {
      font-size: 0.7rem;
      color: #666;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 8px;
    }
    .card .value {
      font-size: 1.6rem;
      font-weight: 700;
      color: #ffffff;
    }
    .card .value.green { color: #4caf82; }
    .card .value.red   { color: #e05252; }
    .card .value.amber { color: #f0a500; }
    .card .value.blue  { color: #4a9eff; }
    .card .value.gray  { color: #888888; }
    .section {
      max-width: 720px;
      margin: 0 auto 14px auto;
      background: #1a1d27;
      border: 1px solid #2a2d3e;
      border-radius: 10px;
      padding: 14px 16px;
    }
    .section h2 {
      font-size: 0.75rem;
      color: #555;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 10px;
    }
    .row {
      display: flex;
      justify-content: space-between;
      padding: 5px 0;
      border-bottom: 1px solid #22253a;
      font-size: 0.88rem;
    }
    .row:last-child { border-bottom: none; }
    .row .k { color: #888; }
    .row .v { color: #ddd; font-weight: 600; }
    .badge {
      display: inline-block;
      padding: 2px 10px;
      border-radius: 20px;
      font-size: 0.75rem;
      font-weight: 700;
    }
    .badge.green { background: #1a3a2a; color: #4caf82; }
    .badge.red   { background: #3a1a1a; color: #e05252; }
    .badge.amber { background: #3a2a0a; color: #f0a500; }
    .footer {
      text-align: center;
      font-size: 0.7rem;
      color: #333;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <h1>&#128246; Speed Zone Monitor</h1>
  <p class='subtitle'>MVIT — Intelligent Vehicle Speed Limiting System</p>

  <div class='grid'>
    <div class='card'>
      <div class='label'>Vehicle Speed</div>
      <div class='value blue'>)rawhtml";
  html += loraSpeed;
  html += R"rawhtml(</div>
    </div>
    <div class='card'>
      <div class='label'>Zone Limit</div>
      <div class='value amber'>)rawhtml";
  html += limitDisplay;   // FIX 2
  html += R"rawhtml(</div>
    </div>
    <div class='card'>
      <div class='label'>Zone Status</div>
      <div class='value )rawhtml";
  html += (loraZone == "IN" ? "green" : "red");
  html += "'>";
  html += loraZone;
  html += R"rawhtml(</div>
    </div>
    <div class='card'>
      <div class='label'>Override</div>
      <div class='value )rawhtml";
  html += (loraOverride ? "red" : "green");
  html += "'>";
  html += ovrStatus;
  html += R"rawhtml(</div>
    </div>
  </div>

  <div class='section'>
    <h2>Road Unit Status</h2>
    <div class='row'>
      <span class='k'>Active Zone</span>
      <span class='v'>)rawhtml";
  html += zoneLabel;
  html += R"rawhtml(</span>
    </div>
    <div class='row'>
      <span class='k'>IR Detection</span>
      <span class='v'>)rawhtml";
  html += "<span class='badge " + String(vehicleDetected ? "green" : "red") + "'>" + vehicleStatus + "</span>";
  html += R"rawhtml(</span>
    </div>
    <div class='row'>
      <span class='k'>LiFi TX</span>
      <span class='v'>)rawhtml";
  html += "<span class='badge " + String(lifiActive ? "green" : "amber") + "'>" + lifiStatus + "</span>";
  html += R"rawhtml(</span>
    </div>
  </div>

  <div class='section'>
    <h2>LoRa Telemetry</h2>
    <div class='row'>
      <span class='k'>Last Packet</span>
      <span class='v'>)rawhtml";
  html += lastSeen;
  html += R"rawhtml(</span>
    </div>
    <div class='row'>
      <span class='k'>Signal (RSSI)</span>
      <span class='v'>)rawhtml";
  html += loraRssi;
  html += " dBm";
  html += R"rawhtml(</span>
    </div>
    <div class='row'>
      <span class='k'>Override Count</span>
      <span class='v'>)rawhtml";
  html += loraOvrCount;
  html += R"rawhtml( / 5</span>
    </div>
    <div class='row'>
      <span class='k'>Raw Message</span>
      <span class='v' style='font-size:0.75rem;color:#666'>)rawhtml";
  html += loraLastMsg;
  html += R"rawhtml(</span>
    </div>
  </div>

  <div class='footer'>
    Auto-refresh every 3s &nbsp;|&nbsp;
    Uptime: )rawhtml";
  html += uptime;
  html += R"rawhtml( &nbsp;|&nbsp; )rawhtml";
  html += WiFi.localIP().toString();
  html += R"rawhtml(
  </div>
</body>
</html>
)rawhtml";

  return html;
}

// =====================================================
// WEB SERVER HANDLERS
// =====================================================
void handleRoot() {
  server.send(200, "text/html", buildDashboard());
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// =====================================================
// LIFI — TRANSMIT ONE BIT
// =====================================================
void lifiSendBit(bool bit) {
  if (bit) {
    digitalWrite(LIFI_PIN, HIGH);
    delayMicroseconds(LIFI_BIT1_HIGH);
    digitalWrite(LIFI_PIN, LOW);
    delayMicroseconds(LIFI_BIT1_LOW);
  } else {
    digitalWrite(LIFI_PIN, HIGH);
    delayMicroseconds(LIFI_BIT0_HIGH);
    digitalWrite(LIFI_PIN, LOW);
    delayMicroseconds(LIFI_BIT0_LOW);
  }
}

// =====================================================
// LIFI — TRANSMIT ONE BYTE
// Start pulse + 8 bits MSB first + inter-packet gap
// =====================================================
void lifiSendByte(byte data) {

  // Start pulse
  digitalWrite(LIFI_PIN, HIGH);
  delayMicroseconds(LIFI_START_HIGH);
  digitalWrite(LIFI_PIN, LOW);
  delayMicroseconds(LIFI_START_LOW);

  // 8 data bits MSB first
  for (int i = 7; i >= 0; i--) {
    lifiSendBit((data >> i) & 0x01);
  }

  // FIX 1: inter-packet gap — ensures receiver can
  // cleanly separate consecutive packets
  digitalWrite(LIFI_PIN, LOW);
  delayMicroseconds(LIFI_PKT_GAP);
}

// =====================================================
// LIFI — HANDLE TX
// Only fires if zone selected AND vehicle detected
// =====================================================
void handleLiFiTx() {

  if (selectedZone == 0) {
    lifiActive = false;
    digitalWrite(LIFI_PIN, LOW);
    return;
  }

  if (!vehicleDetected) {
    lifiActive = false;
    digitalWrite(LIFI_PIN, LOW);
    return;
  }

  if (millis() - lastLiFiTx < LIFI_REPEAT_MS) return;
  lastLiFiTx = millis();

  lifiActive = true;
  lifiSendByte(zoneCode);

  Serial.print("[LIFI] TX: 0x");
  Serial.println(zoneCode, HEX);
}

// =====================================================
// IR SENSOR — VEHICLE DETECTION
// =====================================================
void handleIR() {

  if (digitalRead(IR_PIN) == LOW) {
    irDetectTime = millis();
    if (!vehicleDetected) {
      vehicleDetected = true;
      Serial.println("[IR] Vehicle detected");
    }
  }

  if (vehicleDetected &&
      millis() - irDetectTime > IR_HOLD) {
    vehicleDetected = false;
    lifiActive      = false;
    digitalWrite(LIFI_PIN, LOW);
    Serial.println("[IR] Vehicle cleared");
  }
}

// =====================================================
// LORA RX — PARSE PACKET
// =====================================================
void parseLoRaPacket(String msg) {

  loraLastMsg = msg;
  loraLastRx  = millis();
  loraRssi    = LoRa.packetRssi();   // FIX 3: log RSSI

  auto extract = [&](String key) -> String {
    int idx = msg.indexOf(key + ":");
    if (idx == -1) return "---";
    int start = idx + key.length() + 1;
    int end   = msg.indexOf(",", start);
    if (end == -1) end = msg.length();
    return msg.substring(start, end);
  };

  loraZone     = extract("ZONE");
  loraSpeed    = extract("SPD").toInt();
  loraLimit    = extract("LIM").toInt();
  loraOverride = (extract("OVR") == "1");
  loraOvrCount = extract("OVRC").toInt();

  Serial.print("[LORA] RX: ");
  Serial.print(msg);
  Serial.print(" | RSSI: ");
  Serial.print(loraRssi);
  Serial.println(" dBm");
}

// =====================================================
// LORA RX — CHECK FOR INCOMING PACKET
// =====================================================
void handleLoRaRx() {

  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String msg = "";
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }

  parseLoRaPacket(msg);
}

// =====================================================
// WIFI RECONNECT — FIX 4
// =====================================================
void handleWiFi() {

  if (millis() - lastWifiCheck < WIFI_CHECK_MS) return;
  lastWifiCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected — reconnecting...");
    WiFi.reconnect();
  }
}

// =====================================================
// BLYNK — ZONE SELECTOR
// V0: 0=Exit, 1=Zone A, 2=Zone B, 3=Zone C
// =====================================================
BLYNK_WRITE(V0) {

  selectedZone = param.asInt();

  switch (selectedZone) {
    case 1:
      zoneCode       = ZONE_A;
      zoneLimitValue = 75;
      Serial.println("[BLYNK] Zone A — LIM 75");
      break;
    case 2:
      zoneCode       = ZONE_B;
      zoneLimitValue = 102;
      Serial.println("[BLYNK] Zone B — LIM 102");
      break;
    case 3:
      zoneCode       = ZONE_C;
      zoneLimitValue = 128;
      Serial.println("[BLYNK] Zone C — LIM 128");
      break;
    default:
      zoneCode       = ZONE_EXIT;
      zoneLimitValue = 0;
      lifiActive     = false;
      digitalWrite(LIFI_PIN, LOW);
      Serial.println("[BLYNK] Zone EXIT — LiFi OFF");
      break;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  // ---------------- PINS ----------------
  pinMode(IR_PIN,   INPUT_PULLUP);
  pinMode(LIFI_PIN, OUTPUT);
  digitalWrite(LIFI_PIN, LOW);

  // ---------------- WIFI ----------------
  Serial.print("[WIFI] Connecting");
  WiFi.begin(ssid, pass);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected");
    Serial.print("[WIFI] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI] Failed — continuing without WiFi");
  }

  // ---------------- BLYNK ----------------
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  // ---------------- WEB SERVER ----------------
  server.on("/",     handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[WEB] Server started");
  Serial.print("[WEB] Dashboard: http://");
  Serial.println(WiFi.localIP());

  // ---------------- LORA ----------------
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("[LORA] INIT FAILED");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("[LORA] Ready at 433 MHz");
  Serial.println("[SYS] Road unit online");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  Blynk.run();           // Blynk zone selection

  server.handleClient(); // Web dashboard

  handleWiFi();          // FIX 4: WiFi auto-reconnect

  handleIR();            // IR vehicle detection

  handleLiFiTx();        // LiFi zone code TX

  handleLoRaRx();        // LoRa telemetry RX
}
