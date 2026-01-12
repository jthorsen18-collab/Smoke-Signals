// SmokeSignals_Receiver_Lora_Rev27_Average_2_Screens.ino
//
// Heltec WiFi LoRa 32 V3 (ESP32-S3) receiver
// - Parses:
//    1) Power node:   P,A,RUN,<V>,<A>(,<extra>,<hits>)
//    2) Thermocouple: KIS_TEMP:<id>:<seq>:<tempC>
// - Fast RX using DIO1 interrupt + startReceive/readData loop
// - OLED dashboard (fits 128x64):
//    SmokeSignals RX Rev 27
//    TC 24.25C
//    A  9.15V  7.10A
//    B  --.--V --.--A
//
// Button (PGM) toggles DASHBOARD <-> AVERAGE (average 2 screens)

struct WeldStats;

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
enum ScreenMode { SCREEN_DASHBOARD = 0, SCREEN_AVERAGE = 1 };
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

// welding averages (node A/B)
const float WELD_I_THRESHOLD = 15.0f;
const unsigned long WELD_START_MS = 1000;
const unsigned long WELD_STOP_MS = 1000;

struct WeldStats {
  bool weldActive = false;
  bool averaging = false;
  unsigned long weldStartMs = 0;
  unsigned long lastAboveMs = 0;
  double sumV = 0.0;
  double sumA = 0.0;
  uint32_t count = 0;
  float avgV = NAN;
  float avgA = NAN;
  float lastAvgV = NAN;
  float lastAvgA = NAN;
  float lastTimeSec = 0.0f;
  float archiveAvgV = NAN;
  float archiveAvgA = NAN;
  float archiveTimeSec = 0.0f;
};

WeldStats weldA;
WeldStats weldB;

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
  int c4 = packet.indexOf(',', c3 + 1);     // after A (optional fields follow)

  String nodeStr = packet.substring(2, c1);
  nodeStr.trim();
  if (nodeStr.length() < 1) return false;
  char node = nodeStr.charAt(0);

  String vStr = packet.substring(c2 + 1, c3);
  String aStr = (c4 < 0) ? packet.substring(c3 + 1) : packet.substring(c3 + 1, c4);
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
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 8, "SmokeSignals RX Rev 27");
  u8g2.setFont(u8g2_font_6x12_tf);

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

void updateWeldStats(WeldStats& stats, float v, float a, unsigned long nowMs) {
  if (a >= WELD_I_THRESHOLD) {
    if (!stats.weldActive) {
      if (!isnan(stats.lastAvgV) && !isnan(stats.lastAvgA)) {
        stats.archiveAvgV = stats.lastAvgV;
        stats.archiveAvgA = stats.lastAvgA;
        stats.archiveTimeSec = stats.lastTimeSec;
      }
      stats.lastAvgV = NAN;
      stats.lastAvgA = NAN;
      stats.lastTimeSec = 0.0f;
      stats.weldActive = true;
      stats.averaging = false;
      stats.weldStartMs = nowMs;
      stats.lastAboveMs = nowMs;
      stats.sumV = 0.0;
      stats.sumA = 0.0;
      stats.count = 0;
      stats.avgV = NAN;
      stats.avgA = NAN;
    } else {
      stats.lastAboveMs = nowMs;
    }

    if (!stats.averaging && (nowMs - stats.weldStartMs >= WELD_START_MS)) {
      stats.averaging = true;
    }
    if (stats.averaging) {
      stats.sumV += v;
      stats.sumA += a;
      stats.count += 1;
      stats.avgV = stats.sumV / (double)stats.count;
      stats.avgA = stats.sumA / (double)stats.count;
    }
  } else if (stats.weldActive && (nowMs - stats.lastAboveMs >= WELD_STOP_MS)) {
    if (stats.averaging && stats.count > 0) {
      stats.lastAvgV = stats.avgV;
      stats.lastAvgA = stats.avgA;
      stats.lastTimeSec = (stats.lastAboveMs - stats.weldStartMs) / 1000.0f;
    }
    stats.weldActive = false;
    stats.averaging = false;
  }
}

void drawAverage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);

  char line[32];

  float aDisplayAvgV = weldA.lastAvgV;
  float aDisplayAvgA = weldA.lastAvgA;
  float aDisplayTimeSec = weldA.lastTimeSec;

  if (weldA.weldActive && weldA.averaging && weldA.count > 0) {
    aDisplayAvgV = weldA.avgV;
    aDisplayAvgA = weldA.avgA;
    aDisplayTimeSec = (millis() - weldA.weldStartMs) / 1000.0f;
  }

  if (isnan(aDisplayAvgV) || isnan(aDisplayAvgA)) {
    snprintf(line, sizeof(line), "AVG A - T=--.-s");
  } else {
    snprintf(line, sizeof(line), "AVG A - T=%.1fs", aDisplayTimeSec);
  }
  u8g2.drawStr(0, 8, line);

  if (isnan(aDisplayAvgV) || isnan(aDisplayAvgA)) {
    snprintf(line, sizeof(line), "V=--.- A=--.-");
  } else {
    snprintf(line, sizeof(line), "V=%.1f A=%.1f", aDisplayAvgV, aDisplayAvgA);
  }
  u8g2.drawStr(0, 16, line);

  if (isnan(weldA.archiveAvgV) || isnan(weldA.archiveAvgA)) {
    snprintf(line, sizeof(line), "Arc A - T=--.-s");
  } else {
    snprintf(line, sizeof(line), "Arc A - T=%.1fs", weldA.archiveTimeSec);
  }
  u8g2.drawStr(0, 24, line);

  if (isnan(weldA.archiveAvgV) || isnan(weldA.archiveAvgA)) {
    snprintf(line, sizeof(line), "V=--.- A=--.-");
  } else {
    snprintf(line, sizeof(line), "V=%.1f A=%.1f", weldA.archiveAvgV, weldA.archiveAvgA);
  }
  u8g2.drawStr(0, 32, line);

  float bDisplayAvgV = weldB.lastAvgV;
  float bDisplayAvgA = weldB.lastAvgA;
  float bDisplayTimeSec = weldB.lastTimeSec;

  if (weldB.weldActive && weldB.averaging && weldB.count > 0) {
    bDisplayAvgV = weldB.avgV;
    bDisplayAvgA = weldB.avgA;
    bDisplayTimeSec = (millis() - weldB.weldStartMs) / 1000.0f;
  }

  if (isnan(bDisplayAvgV) || isnan(bDisplayAvgA)) {
    snprintf(line, sizeof(line), "AVG B - T=--.-s");
  } else {
    snprintf(line, sizeof(line), "AVG B - T=%.1fs", bDisplayTimeSec);
  }
  u8g2.drawStr(0, 40, line);

  if (isnan(bDisplayAvgV) || isnan(bDisplayAvgA)) {
    snprintf(line, sizeof(line), "V=--.- A=--.-");
  } else {
    snprintf(line, sizeof(line), "V=%.1f A=%.1f", bDisplayAvgV, bDisplayAvgA);
  }
  u8g2.drawStr(0, 48, line);

  if (isnan(weldB.archiveAvgV) || isnan(weldB.archiveAvgA)) {
    snprintf(line, sizeof(line), "Arc B - T=--.-s");
  } else {
    snprintf(line, sizeof(line), "Arc B - T=%.1fs", weldB.archiveTimeSec);
  }
  u8g2.drawStr(0, 56, line);

  if (isnan(weldB.archiveAvgV) || isnan(weldB.archiveAvgA)) {
    snprintf(line, sizeof(line), "V=--.- A=--.-");
  } else {
    snprintf(line, sizeof(line), "V=%.1f A=%.1f", weldB.archiveAvgV, weldB.archiveAvgA);
  }
  u8g2.drawStr(0, 64, line);

  u8g2.sendBuffer();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("SmokeSignals RX Rev27 (average 2 screens) starting...");

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
    currentScreen = (currentScreen == SCREEN_DASHBOARD) ? SCREEN_AVERAGE : SCREEN_DASHBOARD;
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

            unsigned long now = millis();
            updateWeldStats(weldA, v, a, now);
            changed = true;
          } else if (node == 'B') {
            lastB_V = v;
            lastB_I = a;
            lastBms = millis();

            unsigned long now = millis();
            updateWeldStats(weldB, v, a, now);
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
    else drawAverage();
  }
}
