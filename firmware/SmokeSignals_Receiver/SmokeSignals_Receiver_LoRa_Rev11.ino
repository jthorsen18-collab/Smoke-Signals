// Kisew_TC_Receiver_LoRa_Rev11.ino
//
// Heltec WiFi LoRa 32 V3 (ESP32-S3) receiver
// - Parses:
//    1) Power node:   P,A,RUN,<V>,<A>,<extra>,<hits>
//    2) Thermocouple: KIS_TEMP:<id>:<seq>:<tempC>
// - Fast RX using DIO1 interrupt + startReceive/readData loop
// - OLED dashboard (fits 128x64):
//    SmokeSignals RX
//    TC 24.25C
//    A  9.15V  7.10A
//    B  --.--V --.--A
//
// Button (PGM) toggles DASHBOARD <-> STATUS

#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "RTClib.h"

// ---------------- Pins ----------------
#define BUTTON_PIN   0
#define Vext         36

#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18
#define OLED_RST_PIN 21

#define RTC_SDA_PIN  41
#define RTC_SCL_PIN  40

#define LORA_NSS     8
#define LORA_SCK     9
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_DIO1   14

#define VBAT_ADC_PIN   1
#define ADC_CTRL_PIN   37

// ---------------- OLED ----------------
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  OLED_SCL_PIN,
  OLED_SDA_PIN,
  U8X8_PIN_NONE
);

// ---------------- LoRa ----------------
SX1262 lora = SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
bool loraOK = false;

volatile bool rxFlag = false;
volatile bool rxEnable = true;

static void setRxFlag(void) {
  if (!rxEnable) return;
  rxFlag = true;
}

// ---------------- RTC ----------------
RTC_DS3231 rtc;
bool g_rtcTimeValid = false;

// ---------------- UI state ----------------
enum ScreenMode { SCREEN_DASHBOARD = 0, SCREEN_STATUS = 1 };
ScreenMode currentScreen = SCREEN_DASHBOARD;
bool lastButtonState = HIGH;

// ---------------- Data state ----------------
float lastTempC = NAN;
String lastTC_ID = "T--";
uint32_t lastTC_seq = 0;
unsigned long lastTCms = 0;

float lastA_V = NAN;
float lastA_I = NAN;
unsigned long lastAms = 0;

float lastB_V = NAN;
float lastB_I = NAN;
unsigned long lastBms = 0;

// timeouts
const unsigned long NODE_TIMEOUT_MS = 5000;

// battery (status screen only)
float g_battVoltage = 0.0f;
int   g_battPercent = 0;

// ---------------- Helpers ----------------
void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);   // LOW = Vext ON (Heltec)
}
void OLED_resetRelease() {
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, HIGH);
}

bool isNodeConnected(unsigned long lastPacketMillis) {
  if (lastPacketMillis == 0) return false;
  return (millis() - lastPacketMillis) < NODE_TIMEOUT_MS;
}

// Heltec V3 divider 390k/100k => gain 4.9
float readBatteryVoltage() {
  const int samples = 8;
  uint32_t acc = 0;
  for (int i = 0; i < samples; i++) acc += analogRead(VBAT_ADC_PIN);
  float raw = acc / (float)samples;

  const float ADC_REF_V    = 3.3f;
  const float ADC_COUNTS   = 4095.0f;
  const float DIVIDER_GAIN = (390.0f + 100.0f) / 100.0f; // 4.9

  float v_adc = raw * (ADC_REF_V / ADC_COUNTS);
  return v_adc * DIVIDER_GAIN;
}

int voltageToPercent(float vbat) {
  const float V_MAX = 4.20f;
  const float V_MIN = 3.30f;
  if (vbat >= V_MAX) return 100;
  if (vbat <= V_MIN) return 0;
  float pct = (vbat - V_MIN) * 100.0f / (V_MAX - V_MIN);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

// Parse: KIS_TEMP:<id>:<seq>:<tempC>
bool parseKisTemp(const String& packet, String& outId, uint32_t& outSeq, float& outTempC) {
  const String prefix = "KIS_TEMP:";
  if (!packet.startsWith(prefix)) return false;

  String rest = packet.substring(prefix.length());
  rest.trim();

  int p1 = rest.indexOf(':');
  if (p1 < 0) return false;
  int p2 = rest.indexOf(':', p1 + 1);
  if (p2 < 0) return false;

  String idStr   = rest.substring(0, p1);
  String seqStr  = rest.substring(p1 + 1, p2);
  String tempStr = rest.substring(p2 + 1);

  idStr.trim(); seqStr.trim(); tempStr.trim();

  uint32_t seq = (uint32_t)seqStr.toInt();
  float tempC  = tempStr.toFloat();

  if (tempC < -100.0f || tempC > 1000.0f) return false;

  outId = idStr;
  outSeq = seq;
  outTempC = tempC;
  return true;
}

// Parse: P,<node>,<mode>,<V>,<A>,<extra>,<hits>
bool parsePowerNode(const String& packet, char& outNode, float& outV, float& outA) {
  if (!packet.startsWith("P,")) return false;

  int c1 = packet.indexOf(',', 2);          if (c1 < 0) return false; // after node
  int c2 = packet.indexOf(',', c1 + 1);     if (c2 < 0) return false; // after mode
  int c3 = packet.indexOf(',', c2 + 1);     if (c3 < 0) return false; // after V
  int c4 = packet.indexOf(',', c3 + 1);     if (c4 < 0) return false; // after A

  String nodeStr = packet.substring(2, c1);
  nodeStr.trim();
  if (nodeStr.length() < 1) return false;
  char node = nodeStr.charAt(0);

  String vStr = packet.substring(c2 + 1, c3);
  String aStr = packet.substring(c3 + 1, c4);
  vStr.trim(); aStr.trim();

  float v = vStr.toFloat();
  float a = aStr.toFloat();

  outNode = node;
  outV = v;
  outA = a;
  return true;
}

// ---------------- Drawing ----------------
void drawDashboard() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  u8g2.drawStr(0, 10, "SmokeSignals RX");

  // TC line
  char tcLine[24];
  if (isnan(lastTempC)) {
    snprintf(tcLine, sizeof(tcLine), "TC --.--C");
  } else {
    snprintf(tcLine, sizeof(tcLine), "TC %.2fC", lastTempC);
  }
  u8g2.drawStr(0, 28, tcLine);

  // A line
  char aLine[28];
  if (isnan(lastA_V) || isnan(lastA_I)) {
    snprintf(aLine, sizeof(aLine), "A  --.--V  --.--A");
  } else {
    snprintf(aLine, sizeof(aLine), "A  %.2fV  %.2fA", lastA_V, lastA_I);
  }
  u8g2.drawStr(0, 46, aLine);

  // B line
  char bLine[28];
  if (isnan(lastB_V) || isnan(lastB_I)) {
    snprintf(bLine, sizeof(bLine), "B  --.--V  --.--A");
  } else {
    snprintf(bLine, sizeof(bLine), "B  %.2fV  %.2fA", lastB_V, lastB_I);
  }
  u8g2.drawStr(0, 62, bLine);

  u8g2.sendBuffer();
}

void drawStatus() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  char line[32];

  snprintf(line, sizeof(line), "TC: %s", isNodeConnected(lastTCms) ? "OK" : "NO");
  u8g2.drawStr(0, 12, line);

  snprintf(line, sizeof(line), "A : %s", isNodeConnected(lastAms) ? "OK" : "NO");
  u8g2.drawStr(0, 24, line);

  snprintf(line, sizeof(line), "B : %s", isNodeConnected(lastBms) ? "OK" : "NO");
  u8g2.drawStr(0, 36, line);

  snprintf(line, sizeof(line), "Batt: %.2fV %d%%", g_battVoltage, g_battPercent);
  u8g2.drawStr(0, 48, line);

  u8g2.drawStr(0, 60, g_rtcTimeValid ? "RTC: OK" : "RTC: Check");

  u8g2.sendBuffer();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("SmokeSignals RX Rev11 starting...");

  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, HIGH);
  analogReadResolution(12);

  VextON();
  delay(50);
  OLED_resetRelease();
  delay(50);
  u8g2.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      g_rtcTimeValid = false;
    } else {
      g_rtcTimeValid = true;
    }
  } else {
    g_rtcTimeValid = false;
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = lora.begin(915.0);
  if (st == RADIOLIB_ERR_NONE) {
    loraOK = true;
    Serial.println("LoRa init OK.");

    lora.setDio1Action(setRxFlag);
    rxFlag = false;
    rxEnable = true;

    int rxSt = lora.startReceive();
    if (rxSt != RADIOLIB_ERR_NONE) {
      Serial.print("startReceive FAIL ");
      Serial.println(rxSt);
    }
  } else {
    loraOK = false;
    Serial.print("LoRa init FAIL code ");
    Serial.println(st);
  }

  g_battVoltage = readBatteryVoltage();
  g_battPercent = voltageToPercent(g_battVoltage);

  drawDashboard();
}

void loop() {
  // button toggle
  bool btn = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && btn == LOW) {
    currentScreen = (currentScreen == SCREEN_DASHBOARD) ? SCREEN_STATUS : SCREEN_DASHBOARD;
  }
  lastButtonState = btn;

  // battery update ~1 Hz
  static unsigned long lastBattMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastBattMs >= 1000) {
    g_battVoltage = readBatteryVoltage();
    g_battPercent = voltageToPercent(g_battVoltage);
    lastBattMs = nowMs;
  }

  bool changed = false;

  // RX handling
  if (loraOK && rxFlag) {
    rxEnable = false;
    rxFlag = false;

    String pkt;
    int st = lora.readData(pkt);

    rxEnable = true;

    if (st == RADIOLIB_ERR_NONE) {
      // Serial.println(pkt);

      // TC?
      String id;
      uint32_t seq;
      float tempC;
      if (parseKisTemp(pkt, id, seq, tempC)) {
        lastTempC = tempC;
        lastTC_ID = id;
        lastTC_seq = seq;
        lastTCms = millis();
        changed = true;
      } else {
        // Power node?
        char node;
        float v, a;
        if (parsePowerNode(pkt, node, v, a)) {
          if (node == 'A') {
            lastA_V = v;
            lastA_I = a;
            lastAms = millis();
            changed = true;
          } else if (node == 'B') {
            lastB_V = v;
            lastB_I = a;
            lastBms = millis();
            changed = true;
          }
        }
      }
    }

    lora.startReceive();
  }

  // redraw throttling
  static unsigned long lastDrawMs = 0;
  if (changed || (nowMs - lastDrawMs >= 150)) {
    lastDrawMs = nowMs;
    if (currentScreen == SCREEN_DASHBOARD) drawDashboard();
    else drawStatus();
  }
}
