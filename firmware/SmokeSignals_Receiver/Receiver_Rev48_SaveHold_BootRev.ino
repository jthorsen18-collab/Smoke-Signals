// Receiver_Rev48_SaveHold_BootRev.ino
//
// Heltec WiFi LoRa 32 V3 (ESP32-S3) receiver
// Built back from Rev26 style receive/display speed.
//
// Rev43 from Rev42 goals:
// - Fast RX like Rev26
// - New simplified power packet from sender Rev43/simple sender:
//      P,<node>,<volts>,<amps>,<mode>
//      Example: P,A,27.10,64.2,DC
// - Keep Rev26-style weld average logic, but archive the last completed weld
// - Receiver applies R-factor correction:
//      Vwork = Vmachine - (amps * Rfactor)
// - R-factor pages for A and B, step = 0.002 ohm
// - Live page shows A/B corrected voltage, amps, mode, active R, and last TC event only
// - Verify page shows machine voltage, work voltage, current, active R, and voltage drop
// - Calc screen from Node A/B average page:
//      Enter distance in mm, calculate travel speed mm/min and heat input kJ/mm
// - No logging, no CSV, no stored records; R/calc pages discard RX while open
// - Rev40: AVG page amps show one decimal place; average time shows whole seconds in a 3-digit field
// - Rev41: AVG page TC event line shows both Celsius and Fahrenheit
// - Rev42: Receives distance from TC sender, creates distance alert, and saves heat input records
// - Rev43: Manual receiver clock at boot, real-time sync to TC, distance auto-save, dismiss-only notification
// - Rev44: Receiver timestamps TC events and distance saves with its own clock
// - Rev46: Manual receiver save from AVG A/B. Single/double press opens save from AVG A/B.
// - Rev47: Save screen controls changed for field use: single changes value, hold moves field, double saves, triple exits
// - Rev48: Boot/set-time screen and visible page labels show RX48
 
 
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "RTClib.h"
#include <math.h>
 
#define DEBUG_SERIAL 0
 
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
 
// ---------------- Receiver master clock ----------------
// Manual clock set at boot. Receiver syncs this real-ish time to TC sender.
bool clockSet = false;
uint32_t clockBaseSeconds = 0;
uint32_t clockBaseMs = 0;
int setTimeDigits[4] = {0, 9, 0, 0}; // HH:MM default 09:00
int setTimeField = 0;
 
// ---------------- UI ----------------
enum ScreenMode {
  SCREEN_LIVE = 0,
  SCREEN_AVG_A,
  SCREEN_AVG_B,
  SCREEN_VERIFY,
  SCREEN_RFA,
  SCREEN_RFB,
  SCREEN_CALC,
  SCREEN_ALERT,
  SCREEN_SAVED,
  SCREEN_MANUAL_SAVE,
  SCREEN_SET_TIME
};
 
ScreenMode currentScreen = SCREEN_LIVE;
 
// Button event integers avoid Arduino preprocessor enum/type issues.
const int BTN_NONE = 0;
const int BTN_SINGLE = 1;
const int BTN_DOUBLE = 2;
const int BTN_LONG = 3;
const int BTN_VERY_LONG = 4;
const int BTN_TRIPLE = 5;
 
bool btnLastRaw = HIGH;
bool btnPressed = false;
bool longFired = false;
bool veryLongFired = false;
unsigned long btnDownMs = 0;
unsigned long lastReleaseMs = 0;
int pendingClickCount = 0;
 
const unsigned long DEBOUNCE_MS = 25;
const unsigned long DOUBLE_MS = 350;
const unsigned long LONG_MS = 900;
const unsigned long VERY_LONG_MS = 2400;
 
// ---------------- R-factor ----------------
const float R_STEP = 0.002f;
const int R_COUNT = 41; // 0.000 to 0.080 ohm
int rIndexA = 0;
int rIndexB = 0;
float activeRFactorA = 0.000f;
float activeRFactorB = 0.000f;
 
float rFactorFromIndex(int idx) {
  if (idx < 0) idx = 0;
  if (idx >= R_COUNT) idx = R_COUNT - 1;
  return (float)idx * R_STEP;
}
 
// ---------------- Power node state ----------------
struct PowerNodeState {
  float vm = NAN;       // raw machine voltage from sender
  float amps = NAN;
  String mode = "--";
  unsigned long lastMs = 0;
};
 
PowerNodeState nodeA;
PowerNodeState nodeB;
 
float getRFactor(char node) {
  return (node == 'B') ? activeRFactorB : activeRFactorA;
}
 
void syncActiveRFactor(char node) {
  if (node == 'B') activeRFactorB = rFactorFromIndex(rIndexB);
  else activeRFactorA = rFactorFromIndex(rIndexA);
}
 
float voltageDropFromR(float amps, float rFactor) {
  if (isnan(amps)) return NAN;
  return fabs(amps) * rFactor;
}
 
float correctedVw(float vm, float amps, float rFactor) {
  if (isnan(vm) || isnan(amps)) return NAN;
  float drop = voltageDropFromR(amps, rFactor);
  if (isnan(drop)) return NAN;
  float vw = vm - drop;
  if (vw < 0.0f) vw = 0.0f;
  return vw;
}
 
float nodeCorrectedVw(char node) {
  PowerNodeState& n = (node == 'B') ? nodeB : nodeA;
  return correctedVw(n.vm, n.amps, getRFactor(node));
}
 
// ---------------- Thermocouple event state ----------------
// Receiver does not need live TC readings. It stores the last captured TC event only.
float lastTempC = NAN;
String lastTC_ID = "T--";
uint32_t lastTC_seq = 0;
unsigned long lastTCms = 0;
 
float lastEventTempC = NAN;
float lastEventTempF = NAN;
char lastEventUnit = 'F';
String lastEventTC_ID = "T--";
uint32_t lastEventNumber = 0;
unsigned long lastEventMs = 0;
char lastEventTime[6] = "--:--";
uint32_t localEventCounter = 0;
 
bool seenNodeA = false;
bool seenNodeB = false;
bool seenTC = false;
uint32_t lastSyncMs = 0;
const uint32_t RARE_SYNC_INTERVAL_MS = 10UL * 60UL * 1000UL;
 
// ---------------- Weld averages ----------------
const float WELD_I_THRESHOLD = 15.0f;
const unsigned long WELD_START_MS = 1000;
const unsigned long WELD_STOP_MS = 1000;
const float MIN_ARCHIVE_TIME_SEC = 2.0f;
 
struct WeldStats {
  bool weldActive = false;
  bool averaging = false;
  unsigned long weldStartMs = 0;
  unsigned long lastAboveMs = 0;
  double sumV = 0.0;
  double sumA = 0.0;
  uint32_t count = 0;
 
  // Last completed weld, held until the next completed weld replaces it.
  float archiveAvgV = NAN;
  float archiveAvgA = NAN;
  float archiveTimeSec = 0.0f;
  String archiveMode = "--";
};
 
WeldStats weldA;
WeldStats weldB;
 
// ---------------- Calc screen ----------------
char calcNode = 'A';
float calcV = NAN;
float calcA = NAN;
float calcTimeSec = 0.0f;
int calcDigits[3] = {0, 0, 0};
int calcField = 0; // 0 hundreds, 1 tens, 2 ones, 3 calc
bool calcDone = false;
float calcTravelSpeed = NAN;
float calcHeatInput = NAN;
 
 
// ---------------- Distance alert / saved records ----------------
struct CompletedWeldSnapshot {
  bool valid = false;
  char node = 'A';
  float avgVm = NAN;
  float avgA = NAN;
  float timeSec = 0.0f;
};
 
CompletedWeldSnapshot lastCompleted;
 
struct ResultRecord {
  bool valid = false;
  char node = 'A';
  float vw = NAN;
  float amps = NAN;
  float timeSec = 0.0f;
  int distanceMm = 0;
  float travelSpeed = NAN;
  float heatInput = NAN;
  char distanceTime[6] = "--:--";
  bool hasTcEvent = false;
  float tcTempC = NAN;
  float tcTempF = NAN;
  char tcUnit = 'C';
  char tcTime[6] = "--:--";
  char rodText[12] = "--";
  char passText[8] = "--";
  char weldType[8] = "--";
};
 
ResultRecord alertRecord;
bool alertHasRecord = false;
ScreenMode returnAfterAlert = SCREEN_AVG_A;
 
const int MAX_SAVED_RECORDS = 10;
ResultRecord savedRecords[MAX_SAVED_RECORDS];
int savedCount = 0;
int savedIndex = 0;
 
// ---------------- Manual receiver save state ----------------
struct ManualSaveDraft {
  bool active = false;
  bool validAvg = false;
  char node = 'A';
  float avgVm = NAN;
  float avgVw = NAN;
  float avgA = NAN;
  float timeSec = 0.0f;
  char saveTime[6] = "--:--";
  bool hasTcEvent = false;
  float tcTempC = NAN;
  float tcTempF = NAN;
  char tcUnit = 'C';
  char tcTime[6] = "--:--";
};
 
ManualSaveDraft saveDraft;
int saveDigits[3] = {0, 0, 0}; // hundreds, tens, ones(0 or 5)
int saveField = 0;             // 0 H, 1 T, 2 ones, 3 rod, 4 pass, 5 type, 6 SAVE
int saveRodIndex = 3;          // default 1/8 3.2
int savePassIndex = 3;         // default A1
int saveTypeIndex = 0;         // default 2G/F
 
const char* ROD_LIST[] = {"5/64 2.0", "3/32 2.4", "3/32 2.5", "1/8 3.2", "5/32 4.0", "3/16 4.8", "1/4 6.0"};
const int ROD_COUNT = sizeof(ROD_LIST) / sizeof(ROD_LIST[0]);
const char* PASS_LIST[] = {"Root", "Fill", "Cap", "A1", "A2", "B1", "C1"};
const int PASS_COUNT = sizeof(PASS_LIST) / sizeof(PASS_LIST[0]);
const char* TYPE_LIST[] = {"2G/F", "5G/F"};
const int TYPE_COUNT = sizeof(TYPE_LIST) / sizeof(TYPE_LIST[0]);
 
 
String lastDistanceId = "";
uint32_t lastDistanceSeq = 0;
 
// Explicit prototypes below prevent Arduino IDE from auto-generating prototypes
// above the ResultRecord struct, which can cause compile errors.
void storeRecord(const ResultRecord& rec);
void openManualSaveForNode(char node);
void drawManualSave();
void handleManualSaveButton(int ev);
void saveManualDraft();
 
float calcTravelSpeedFromDistance(int distanceMm, float timeSec) {
  if (distanceMm <= 0 || timeSec <= 0.0f) return NAN;
  return ((float)distanceMm * 60.0f) / timeSec;
}
 
float calcHeatInputKjMm(float vw, float amps, float travelSpeed) {
  if (isnan(vw) || isnan(amps) || isnan(travelSpeed) || travelSpeed <= 0.0f) return NAN;
  return (vw * amps * 60.0f) / (travelSpeed * 1000.0f);
}
 
// ---------------- Timeouts / battery ----------------
const unsigned long NODE_TIMEOUT_MS = 5000;
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
 
float cToF(float c) {
  return c * 9.0f / 5.0f + 32.0f;
}
 
unsigned long secondsSince(unsigned long eventMs) {
  if (eventMs == 0) return 0;
  return (millis() - eventMs) / 1000UL;
}
 
uint32_t currentClockSeconds() {
  if (!clockSet) return 0;
  uint32_t elapsed = (millis() - clockBaseMs) / 1000UL;
  return (clockBaseSeconds + elapsed) % 86400UL;
}
 
void fmtClock(char* buf, size_t len) {
  if (!clockSet) {
    snprintf(buf, len, "--:--");
    return;
  }
  uint32_t sec = currentClockSeconds();
  uint32_t hh = sec / 3600UL;
  uint32_t mm = (sec % 3600UL) / 60UL;
  snprintf(buf, len, "%02lu:%02lu", (unsigned long)hh, (unsigned long)mm);
}
 
void copyTimeText(char* dst, const char* src) {
  if (!src || strlen(src) < 5) {
    strncpy(dst, "--:--", 6);
  } else {
    strncpy(dst, src, 5);
    dst[5] = '\0';
  }
}
 
void setClockFromDigits() {
  int hh = setTimeDigits[0] * 10 + setTimeDigits[1];
  int mm = setTimeDigits[2] * 10 + setTimeDigits[3];
  if (hh > 23) hh = 23;
  if (mm > 59) mm = 59;
  clockBaseSeconds = (uint32_t)hh * 3600UL + (uint32_t)mm * 60UL;
  clockBaseMs = millis();
  clockSet = true;
}
 
void fmtTCEventLine(char* buf, size_t len) {
  if ((lastEventUnit == 'C' && isnan(lastEventTempC)) ||
      (lastEventUnit != 'C' && isnan(lastEventTempF)) ||
      lastEventMs == 0) {
    snprintf(buf, len, "E--- --.-%c --:--", lastEventUnit == 'C' ? 'C' : 'F');
    return;
  }
 
  float displayTemp = (lastEventUnit == 'C') ? lastEventTempC : lastEventTempF;
  snprintf(buf, len, "E%lu %.1f%c %s",
           (unsigned long)lastEventNumber,
           displayTemp,
           lastEventUnit == 'C' ? 'C' : 'F',
           lastEventTime);
}
 
void fmtTCEventLineBothUnits(char* buf, size_t len) {
  if (isnan(lastEventTempC) || isnan(lastEventTempF) || lastEventMs == 0) {
    snprintf(buf, len, "E--- --.-C --.-F --:--");
    return;
  }
 
  snprintf(buf, len, "E%lu %.1fC %.1fF %s",
           (unsigned long)lastEventNumber,
           lastEventTempC,
           lastEventTempF,
           lastEventTime);
}
 
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
 
  float tempC  = tempStr.toFloat();
  if (tempC < -100.0f || tempC > 1000.0f) return false;
 
  outId = idStr;
  outSeq = (uint32_t)seqStr.toInt();
  outTempC = tempC;
  return true;
}
 
// Parse: KIS_EVENT:<id>:<eventNumber>:<tempC>
bool parseKisEvent(const String& packet, String& outId, uint32_t& outEvent, float& outTempC) {
  const String prefix = "KIS_EVENT:";
  if (!packet.startsWith(prefix)) return false;
 
  String rest = packet.substring(prefix.length());
  rest.trim();
 
  int p1 = rest.indexOf(':');
  if (p1 < 0) return false;
  int p2 = rest.indexOf(':', p1 + 1);
  if (p2 < 0) return false;
 
  String idStr = rest.substring(0, p1);
  String evStr = rest.substring(p1 + 1, p2);
  String tempStr = rest.substring(p2 + 1);
  idStr.trim(); evStr.trim(); tempStr.trim();
 
  float tempC = tempStr.toFloat();
  if (tempC < -100.0f || tempC > 1000.0f) return false;
 
  outId = idStr;
  outEvent = (uint32_t)evStr.toInt();
  outTempC = tempC;
  return true;
}
 
 
// Parse TC event packet from TC Rev5:
// New:    TCE,<id>,<eventNo>,<tempC>,<tempF>,<unit>,<ageMs>
// Legacy: TCE,<id>,<eventNo>,<tempC>,<tempF>,<ageMs>
bool looksLikeTimeText(const String& s) {
  if (s.length() != 5) return false;
  if (s.charAt(2) != ':') return false;
  return isDigit(s.charAt(0)) && isDigit(s.charAt(1)) && isDigit(s.charAt(3)) && isDigit(s.charAt(4));
}
 
// Parse TC event packet from TC Rev7:
// New:    TCE,<id>,<eventNo>,<tempC>,<tempF>,<unit>,<HH:MM>
// Legacy: TCE,<id>,<eventNo>,<tempC>,<tempF>,<unit>,<ageMs>
bool parseTcEventPacket(const String& packet, String& outId, uint32_t& outEvent, float& outTempC, float& outTempF, char& outUnit, uint32_t& outAgeMs, char* outTimeText) {
  if (!packet.startsWith("TCE,")) return false;
 
  String rest = packet.substring(4);
  rest.trim();
 
  int p1 = rest.indexOf(',');
  if (p1 < 0) return false;
  int p2 = rest.indexOf(',', p1 + 1);
  if (p2 < 0) return false;
  int p3 = rest.indexOf(',', p2 + 1);
  if (p3 < 0) return false;
  int p4 = rest.indexOf(',', p3 + 1);
  if (p4 < 0) return false;
 
  String idStr = rest.substring(0, p1);
  String evStr = rest.substring(p1 + 1, p2);
  String cStr = rest.substring(p2 + 1, p3);
  String fStr = rest.substring(p3 + 1, p4);
 
  String unitStr;
  String timeOrAgeStr;
 
  int p5 = rest.indexOf(',', p4 + 1);
  if (p5 < 0) {
    unitStr = "F";
    timeOrAgeStr = rest.substring(p4 + 1);
  } else {
    unitStr = rest.substring(p4 + 1, p5);
    timeOrAgeStr = rest.substring(p5 + 1);
  }
 
  idStr.trim(); evStr.trim(); cStr.trim(); fStr.trim(); unitStr.trim(); timeOrAgeStr.trim();
 
  float tempC = cStr.toFloat();
  float tempF = fStr.toFloat();
  char unit = unitStr.length() > 0 ? unitStr.charAt(0) : 'F';
  if (!(unit == 'C' || unit == 'F')) unit = 'F';
 
  if (tempC < -100.0f || tempC > 1000.0f) return false;
  if (tempF < -148.0f || tempF > 1832.0f) return false;
 
  outId = idStr;
  outEvent = (uint32_t)evStr.toInt();
  outTempC = tempC;
  outTempF = tempF;
  outUnit = unit;
 
  if (looksLikeTimeText(timeOrAgeStr)) {
    copyTimeText(outTimeText, timeOrAgeStr.c_str());
    outAgeMs = 0;
  } else {
    outAgeMs = (uint32_t)timeOrAgeStr.toInt();
    char nowTxt[6];
    fmtClock(nowTxt, sizeof(nowTxt));
    copyTimeText(outTimeText, nowTxt);
  }
  return true;
}
 
// Parse distance packet from TC Rev6:
// New:    DST,<id>,<distanceNo>,<distanceMm>
// Legacy: DST,<id>,<distanceMm>
// Parse distance packet from TC Rev7:
// New:    DST,<id>,<distanceNo>,<distanceMm>,<HH:MM>
// Legacy: DST,<id>,<distanceNo>,<distanceMm>
bool parseDistancePacket(const String& packet, String& outId, uint32_t& outSeq, int& outMm, char* outTimeText) {
  if (!packet.startsWith("DST,")) return false;
 
  String rest = packet.substring(4);
  rest.trim();
 
  int p1 = rest.indexOf(',');
  if (p1 < 0) return false;
  int p2 = rest.indexOf(',', p1 + 1);
 
  String idStr = rest.substring(0, p1);
  String seqStr;
  String mmStr;
  String timeStr = "--:--";
 
  if (p2 < 0) {
    seqStr = "0";
    mmStr = rest.substring(p1 + 1);
  } else {
    int p3 = rest.indexOf(',', p2 + 1);
    seqStr = rest.substring(p1 + 1, p2);
    if (p3 < 0) {
      mmStr = rest.substring(p2 + 1);
    } else {
      mmStr = rest.substring(p2 + 1, p3);
      timeStr = rest.substring(p3 + 1);
    }
  }
 
  idStr.trim(); seqStr.trim(); mmStr.trim(); timeStr.trim();
 
  int mm = mmStr.toInt();
  if (mm <= 0 || mm > 495) return false;
 
  outId = idStr;
  outSeq = (uint32_t)seqStr.toInt();
  outMm = mm;
  if (looksLikeTimeText(timeStr)) copyTimeText(outTimeText, timeStr.c_str());
  else {
    char nowTxt[6];
    fmtClock(nowTxt, sizeof(nowTxt));
    copyTimeText(outTimeText, nowTxt);
  }
  return true;
}
 
// Parse simplified power packet: P,<node>,<V>,<A>,<mode>
// Example: P,A,27.10,64.2,DC
bool parsePowerNodeSimple(const String& packet, char& outNode, float& outV, float& outA, String& outMode) {
  if (!packet.startsWith("P,")) return false;
 
  int c1 = packet.indexOf(',', 2);       // after node
  if (c1 < 0) return false;
  int c2 = packet.indexOf(',', c1 + 1);  // after volts
  if (c2 < 0) return false;
  int c3 = packet.indexOf(',', c2 + 1);  // after amps
  if (c3 < 0) return false;
 
  String nodeStr = packet.substring(2, c1);
  String vStr = packet.substring(c1 + 1, c2);
  String aStr = packet.substring(c2 + 1, c3);
  String modeStr = packet.substring(c3 + 1);
 
  nodeStr.trim(); vStr.trim(); aStr.trim(); modeStr.trim();
  if (nodeStr.length() < 1) return false;
 
  char node = nodeStr.charAt(0);
  if (!(node == 'A' || node == 'B')) return false;
 
  outNode = node;
  outV = vStr.toFloat();
  outA = aStr.toFloat();
  outMode = modeStr;
  return true;
}
 
void updateWeldStats(char node, float vMachine, float amps, const String& modeText, unsigned long nowMs) {
  WeldStats& stats = (node == 'B') ? weldB : weldA;
  if (isnan(vMachine) || isnan(amps)) return;
 
  if (amps >= WELD_I_THRESHOLD) {
    if (!stats.weldActive) {
      stats.weldActive = true;
      stats.averaging = false;
      stats.weldStartMs = nowMs;
      stats.lastAboveMs = nowMs;
      stats.sumV = 0.0;
      stats.sumA = 0.0;
      stats.count = 0;
    } else {
      stats.lastAboveMs = nowMs;
    }
 
    if (!stats.averaging && (nowMs - stats.weldStartMs >= WELD_START_MS)) {
      stats.averaging = true;
    }
 
    if (stats.averaging) {
      // Store raw machine voltage so changing R-factor later changes the corrected average.
      stats.sumV += vMachine;
      stats.sumA += amps;
      stats.count += 1;
    }
  } else if (stats.weldActive && (nowMs - stats.lastAboveMs >= WELD_STOP_MS)) {
    float timeSec = (stats.lastAboveMs - stats.weldStartMs) / 1000.0f;
 
    if (stats.averaging && stats.count > 0 && timeSec >= MIN_ARCHIVE_TIME_SEC) {
      stats.archiveAvgV = stats.sumV / (double)stats.count;
      stats.archiveAvgA = stats.sumA / (double)stats.count;
      stats.archiveTimeSec = timeSec;
      stats.archiveMode = modeText;
 
      lastCompleted.valid = true;
      lastCompleted.node = node;
      lastCompleted.avgVm = stats.archiveAvgV;
      lastCompleted.avgA = stats.archiveAvgA;
      lastCompleted.timeSec = stats.archiveTimeSec;
    }
 
    stats.weldActive = false;
    stats.averaging = false;
  }
}
 
void markLocalTempEvent() {
  if (isnan(lastTempC)) return;
  localEventCounter++;
  lastEventTempC = lastTempC;
  lastEventTempF = cToF(lastTempC);
  lastEventUnit = 'C';
  lastEventTC_ID = "RX";
  lastEventNumber = localEventCounter;
  lastEventMs = millis();
  fmtClock(lastEventTime, sizeof(lastEventTime));
}
 
String shortMode(const String& mode) {
  if (mode == "AC") return "AC";
  if (mode == "DC") return "DC";
  return "--";
}
 
bool nodeHasDisplayValues(char node) {
  PowerNodeState& n = (node == 'B') ? nodeB : nodeA;
  if (n.lastMs == 0) return false;
  if (!isNodeConnected(n.lastMs)) return false;
  if (isnan(n.vm) || isnan(n.amps)) return false;
 
  // If packet is no-load / no values, display blanks.
  if (fabs(n.vm) < 0.05f && fabs(n.amps) < 0.5f && shortMode(n.mode) == "--") return false;
  return true;
}
 
void fmtR(char* buf, size_t len, char node) {
  snprintf(buf, len, "R %.3f", getRFactor(node));
}
 
// ---------------- Drawing ----------------
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 8, title);
}
 
void drawLiveNodeLine(int y, char label) {
  PowerNodeState& n = (label == 'B') ? nodeB : nodeA;
  float r = getRFactor(label);
  char line[32];
 
  if (!nodeHasDisplayValues(label)) {
    snprintf(line, sizeof(line), "%c --.-Vw --.-A --", label);
  } else {
    float vw = correctedVw(n.vm, n.amps, r);
    snprintf(line, sizeof(line), "%c %.1fVw %.1fA %s",
             label, vw, n.amps, shortMode(n.mode).c_str());
  }
 
  u8g2.drawStr(0, y, line);
}
 
void drawLive() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
 
  u8g2.drawStr(0, 8, "RX48 LIVE");
 
  char line[32];
 
  drawLiveNodeLine(20, 'A');
  fmtR(line, sizeof(line), 'A');
  u8g2.drawStr(0, 30, line);
 
  drawLiveNodeLine(42, 'B');
  fmtR(line, sizeof(line), 'B');
  u8g2.drawStr(0, 52, line);
 
  fmtTCEventLine(line, sizeof(line));
  u8g2.drawStr(0, 63, line);
 
  u8g2.sendBuffer();
}
 
void drawNodeAverage(char nodeLabel) {
  PowerNodeState& n = (nodeLabel == 'B') ? nodeB : nodeA;
  WeldStats& w = (nodeLabel == 'B') ? weldB : weldA;
 
  u8g2.clearBuffer();
 
  char line[32];
  float r = getRFactor(nodeLabel);
  float liveVw = correctedVw(n.vm, n.amps, r);
 
  // Bigger two-column average page:
  // Left = live corrected work values. Right = archived average values.
  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(line, sizeof(line), "LIVE %c", nodeLabel);
  u8g2.drawStr(0, 9, line);
  snprintf(line, sizeof(line), "AVG %c", nodeLabel);
  u8g2.drawStr(76, 9, line);
 
  u8g2.setFont(u8g2_font_7x13B_tf);
 
  if (!nodeHasDisplayValues(nodeLabel)) {
    u8g2.drawStr(0, 24, "--.-Vw");
    u8g2.drawStr(0, 39, "---A");
  } else {
    snprintf(line, sizeof(line), "%.1fVw", liveVw);
    u8g2.drawStr(0, 24, line);
    snprintf(line, sizeof(line), "%.1fA", n.amps);
    u8g2.drawStr(0, 39, line);
  }
 
  if (isnan(w.archiveAvgV) || isnan(w.archiveAvgA)) {
    u8g2.drawStr(76, 24, "--.-Vw");
    u8g2.drawStr(76, 39, "---A");
    u8g2.drawStr(76, 54, "---s");
  } else {
    float archiveVw = correctedVw(w.archiveAvgV, w.archiveAvgA, r);
    snprintf(line, sizeof(line), "%.1fVw", archiveVw);
    u8g2.drawStr(76, 24, line);
    snprintf(line, sizeof(line), "%.1fA", w.archiveAvgA);
    u8g2.drawStr(76, 39, line);
    snprintf(line, sizeof(line), "%3.0fs", w.archiveTimeSec);
    u8g2.drawStr(76, 54, line);
  }
 
  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(line, sizeof(line), "R %.3f", r);
  u8g2.drawStr(0, 53, line);
 
  u8g2.setFont(u8g2_font_5x8_tf);
  fmtTCEventLineBothUnits(line, sizeof(line));
  u8g2.drawStr(0, 63, line);
 
  u8g2.sendBuffer();
}
 
void drawVerify() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
 
  u8g2.drawStr(0, 8, "VERIFY RX48");
 
  char line[32];
  float rA = activeRFactorA;
  float rB = activeRFactorB;
  float dropA = voltageDropFromR(nodeA.amps, rA);
  float dropB = voltageDropFromR(nodeB.amps, rB);
  float aVw = correctedVw(nodeA.vm, nodeA.amps, rA);
  float bVw = correctedVw(nodeB.vm, nodeB.amps, rB);
 
  if (!nodeHasDisplayValues('A') || isnan(aVw)) snprintf(line, sizeof(line), "A M--.- W--.-");
  else snprintf(line, sizeof(line), "A M%.1f W%.1f", nodeA.vm, aVw);
  u8g2.drawStr(0, 18, line);
 
  if (!nodeHasDisplayValues('A')) snprintf(line, sizeof(line), "A I--- R%.3f", rA);
  else snprintf(line, sizeof(line), "A I%.1f R%.3f", nodeA.amps, rA);
  u8g2.drawStr(0, 27, line);
 
  if (!nodeHasDisplayValues('A') || isnan(dropA)) snprintf(line, sizeof(line), "A D--.-");
  else snprintf(line, sizeof(line), "A D%.1f", dropA);
  u8g2.drawStr(0, 36, line);
 
  if (!nodeHasDisplayValues('B') || isnan(bVw)) snprintf(line, sizeof(line), "B M--.- W--.-");
  else snprintf(line, sizeof(line), "B M%.1f W%.1f", nodeB.vm, bVw);
  u8g2.drawStr(0, 45, line);
 
  if (!nodeHasDisplayValues('B')) snprintf(line, sizeof(line), "B I--- R%.3f", rB);
  else snprintf(line, sizeof(line), "B I%.1f R%.3f", nodeB.amps, rB);
  u8g2.drawStr(0, 54, line);
 
  if (!nodeHasDisplayValues('B') || isnan(dropB)) snprintf(line, sizeof(line), "B D--.-");
  else snprintf(line, sizeof(line), "B D%.1f", dropB);
  u8g2.drawStr(0, 63, line);
 
  u8g2.sendBuffer();
}
 
void drawRFactorPage(char nodeLabel, int selectedIndex) {
  u8g2.clearBuffer();
 
  char title[20];
  snprintf(title, sizeof(title), "R FACTOR %c", nodeLabel);
  drawHeader(title);
  u8g2.setFont(u8g2_font_6x10_tf);
 
  int start = selectedIndex - 2;
  if (start < 0) start = 0;
  if (start > R_COUNT - 5) start = R_COUNT - 5;
  if (start < 0) start = 0;
 
  for (int i = 0; i < 5; i++) {
    int idx = start + i;
    if (idx >= R_COUNT) break;
    char line[24];
    snprintf(line, sizeof(line), "%c %.3f", (idx == selectedIndex) ? '-' : ' ', rFactorFromIndex(idx));
    u8g2.drawStr(0, 22 + i * 10, line);
  }
 
  u8g2.sendBuffer();
}
 
int calcDistanceMm() {
  return calcDigits[0] * 100 + calcDigits[1] * 10 + calcDigits[2];
}
 
void runCalc() {
  int dist = calcDistanceMm();
  if (dist <= 0 || calcTimeSec <= 0.0f || isnan(calcV) || isnan(calcA)) {
    calcTravelSpeed = NAN;
    calcHeatInput = NAN;
    calcDone = true;
    return;
  }
 
  calcTravelSpeed = ((float)dist * 60.0f) / calcTimeSec; // mm/min
  if (calcTravelSpeed > 0.0f) {
    calcHeatInput = (calcV * calcA * 60.0f) / (calcTravelSpeed * 1000.0f); // kJ/mm
  } else {
    calcHeatInput = NAN;
  }
  calcDone = true;
}
 
 
void storeRecord(const ResultRecord& rec) {
  if (!rec.valid) return;
 
  if (savedCount < MAX_SAVED_RECORDS) {
    savedRecords[savedCount] = rec;
    savedIndex = savedCount;
    savedCount++;
  } else {
    for (int i = 1; i < MAX_SAVED_RECORDS; i++) {
      savedRecords[i - 1] = savedRecords[i];
    }
    savedRecords[MAX_SAVED_RECORDS - 1] = rec;
    savedIndex = MAX_SAVED_RECORDS - 1;
  }
}
 
 
int saveDistanceMm() {
  return saveDigits[0] * 100 + saveDigits[1] * 10 + saveDigits[2];
}
 
void copySmallText(char* dst, size_t len, const char* src) {
  if (!dst || len == 0) return;
  if (!src) src = "--";
  strncpy(dst, src, len - 1);
  dst[len - 1] = '\0';
}
 
void openManualSaveForNode(char node) {
  WeldStats& w = (node == 'B') ? weldB : weldA;
  if (isnan(w.archiveAvgV) || isnan(w.archiveAvgA)) return;
 
  saveDraft.active = true;
  saveDraft.validAvg = true;
  saveDraft.node = node;
  saveDraft.avgVm = w.archiveAvgV;
  saveDraft.avgA = w.archiveAvgA;
  saveDraft.timeSec = (w.archiveTimeSec > 0.0f) ? w.archiveTimeSec : 0.0f;
  saveDraft.avgVw = correctedVw(w.archiveAvgV, w.archiveAvgA, getRFactor(node));
  fmtClock(saveDraft.saveTime, sizeof(saveDraft.saveTime));
 
  saveDraft.hasTcEvent = (lastEventMs != 0 && !isnan(lastEventTempC) && !isnan(lastEventTempF));
  saveDraft.tcTempC = lastEventTempC;
  saveDraft.tcTempF = lastEventTempF;
  saveDraft.tcUnit = lastEventUnit;
  copyTimeText(saveDraft.tcTime, lastEventTime);
 
  saveDigits[0] = 0;
  saveDigits[1] = 0;
  saveDigits[2] = 0;
  saveField = 0;
  currentScreen = SCREEN_MANUAL_SAVE;
}
 
void saveManualDraft() {
  if (!saveDraft.active || !saveDraft.validAvg) return;
  int distanceMm = saveDistanceMm();
  if (distanceMm <= 0) return;
 
  ResultRecord rec;
  rec.valid = true;
  rec.node = saveDraft.node;
  rec.vw = saveDraft.avgVw;
  rec.amps = saveDraft.avgA;
  rec.timeSec = saveDraft.timeSec;
  rec.distanceMm = distanceMm;
  rec.travelSpeed = calcTravelSpeedFromDistance(distanceMm, saveDraft.timeSec);
  rec.heatInput = calcHeatInputKjMm(rec.vw, rec.amps, rec.travelSpeed);
  copyTimeText(rec.distanceTime, saveDraft.saveTime);
 
  rec.hasTcEvent = saveDraft.hasTcEvent;
  rec.tcTempC = saveDraft.tcTempC;
  rec.tcTempF = saveDraft.tcTempF;
  rec.tcUnit = saveDraft.tcUnit;
  copyTimeText(rec.tcTime, saveDraft.tcTime);
  copySmallText(rec.rodText, sizeof(rec.rodText), ROD_LIST[saveRodIndex]);
  copySmallText(rec.passText, sizeof(rec.passText), PASS_LIST[savePassIndex]);
  copySmallText(rec.weldType, sizeof(rec.weldType), TYPE_LIST[saveTypeIndex]);
 
  alertRecord = rec;
  storeRecord(rec);
  alertHasRecord = true;
  returnAfterAlert = (saveDraft.node == 'B') ? SCREEN_AVG_B : SCREEN_AVG_A;
  saveDraft.active = false;
  currentScreen = SCREEN_ALERT;
}
 
void buildDistanceAlert(int distanceMm, const char* distanceTimeText) {
  if (!lastCompleted.valid) return;
 
  char node = lastCompleted.node;
  float r = getRFactor(node);
  float vw = correctedVw(lastCompleted.avgVm, lastCompleted.avgA, r);
  float ts = calcTravelSpeedFromDistance(distanceMm, lastCompleted.timeSec);
  float hi = calcHeatInputKjMm(vw, lastCompleted.avgA, ts);
 
  alertRecord.valid = true;
  alertRecord.node = node;
  alertRecord.vw = vw;
  alertRecord.amps = lastCompleted.avgA;
  alertRecord.timeSec = lastCompleted.timeSec;
  alertRecord.distanceMm = distanceMm;
  alertRecord.travelSpeed = ts;
  alertRecord.heatInput = hi;
  copyTimeText(alertRecord.distanceTime, distanceTimeText);
 
  alertRecord.hasTcEvent = (lastEventMs != 0 && !isnan(lastEventTempC) && !isnan(lastEventTempF));
  alertRecord.tcTempC = lastEventTempC;
  alertRecord.tcTempF = lastEventTempF;
  alertRecord.tcUnit = lastEventUnit;
  copyTimeText(alertRecord.tcTime, lastEventTime);
  copySmallText(alertRecord.rodText, sizeof(alertRecord.rodText), "TC");
  copySmallText(alertRecord.passText, sizeof(alertRecord.passText), "DIST");
  copySmallText(alertRecord.weldType, sizeof(alertRecord.weldType), "--");
 
  storeRecord(alertRecord);
 
  alertHasRecord = true;
  returnAfterAlert = currentScreen;
  currentScreen = SCREEN_ALERT;
}
 
void drawAlert() {
  u8g2.clearBuffer();
  char line[64];
 
  u8g2.setFont(u8g2_font_5x8_tf);
 
  if (!alertHasRecord || !alertRecord.valid) {
    u8g2.drawStr(0, 10, "SAVED");
    u8g2.drawStr(0, 26, "NO AVG READY");
    u8g2.drawStr(0, 62, "double dismiss");
    u8g2.sendBuffer();
    return;
  }
 
  snprintf(line, sizeof(line), "SAVED %s %c %s %s", alertRecord.distanceTime, alertRecord.node, alertRecord.passText, alertRecord.weldType);
  u8g2.drawStr(0, 8, line);
 
  snprintf(line, sizeof(line), "A%.1f V%.1f L%03d T%03.0f", alertRecord.amps, alertRecord.vw, alertRecord.distanceMm, alertRecord.timeSec);
  u8g2.drawStr(0, 18, line);
 
  if (isnan(alertRecord.travelSpeed) || isnan(alertRecord.heatInput)) {
    snprintf(line, sizeof(line), "%s --mm/m --kJ/mm", alertRecord.rodText);
  } else {
    snprintf(line, sizeof(line), "%s %.0fmm/m %.2fkJ/mm", alertRecord.rodText, alertRecord.travelSpeed, alertRecord.heatInput);
  }
  u8g2.drawStr(0, 28, line);
 
  if (alertRecord.hasTcEvent) {
    snprintf(line, sizeof(line), "TC%.1fC %.1fF %s", alertRecord.tcTempC, alertRecord.tcTempF, alertRecord.tcTime);
  } else {
    snprintf(line, sizeof(line), "TC --.-C --.-F --:--");
  }
  u8g2.drawStr(0, 38, line);
 
  u8g2.drawStr(0, 62, "double dismiss");
  u8g2.sendBuffer();
}
 
void drawSaved() {
  u8g2.clearBuffer();
  char line[64];
 
  u8g2.setFont(u8g2_font_5x8_tf);
 
  if (savedCount <= 0) {
    u8g2.drawStr(0, 9, "SAVED");
    u8g2.drawStr(0, 28, "NO RECORDS");
    u8g2.sendBuffer();
    return;
  }
 
  if (savedIndex < 0) savedIndex = 0;
  if (savedIndex >= savedCount) savedIndex = savedCount - 1;
  ResultRecord& r = savedRecords[savedIndex];
 
  snprintf(line, sizeof(line), "%d/%d %s %c %s %s", savedIndex + 1, savedCount, r.distanceTime, r.node, r.passText, r.weldType);
  u8g2.drawStr(0, 8, line);
 
  snprintf(line, sizeof(line), "A%.1f V%.1f L%03d T%03.0f", r.amps, r.vw, r.distanceMm, r.timeSec);
  u8g2.drawStr(0, 18, line);
 
  if (isnan(r.travelSpeed) || isnan(r.heatInput)) {
    snprintf(line, sizeof(line), "%s --mm/m --kJ/mm", r.rodText);
  } else {
    snprintf(line, sizeof(line), "%s %.0fmm/m %.2fkJ/mm", r.rodText, r.travelSpeed, r.heatInput);
  }
  u8g2.drawStr(0, 28, line);
 
  if (r.hasTcEvent) {
    snprintf(line, sizeof(line), "TC%.1fC %.1fF %s", r.tcTempC, r.tcTempF, r.tcTime);
  } else {
    snprintf(line, sizeof(line), "TC --.-C --.-F --:--");
  }
  u8g2.drawStr(0, 38, line);
 
  u8g2.drawStr(0, 62, "clk next  hold page");
  u8g2.sendBuffer();
}
 
void drawManualSave() {
  u8g2.clearBuffer();
  char line[64];
  u8g2.setFont(u8g2_font_5x8_tf);
 
  if (!saveDraft.active || !saveDraft.validAvg) {
    u8g2.drawStr(0, 9, "SAVE");
    u8g2.drawStr(0, 28, "NO AVG READY");
    u8g2.drawStr(0, 62, "triple exit");
    u8g2.sendBuffer();
    return;
  }
 
  snprintf(line, sizeof(line), "SAVE RX48 %c %s", saveDraft.node, saveDraft.saveTime);
  u8g2.drawStr(0, 8, line);
 
  snprintf(line, sizeof(line), "A%.1f V%.1f", saveDraft.avgA, saveDraft.avgVw);
  u8g2.drawStr(0, 18, line);
 
  snprintf(line, sizeof(line), "L%c%d%c%d%c%d  T%03.0f",
           (saveField == 0) ? '>' : ' ', saveDigits[0],
           (saveField == 1) ? '>' : ' ', saveDigits[1],
           (saveField == 2) ? '>' : ' ', saveDigits[2],
           saveDraft.timeSec);
  u8g2.drawStr(0, 28, line);
 
  snprintf(line, sizeof(line), "%cRod %s", (saveField == 3) ? '>' : ' ', ROD_LIST[saveRodIndex]);
  u8g2.drawStr(0, 38, line);
 
  snprintf(line, sizeof(line), "%c%s  %c%s",
           (saveField == 4) ? '>' : ' ', PASS_LIST[savePassIndex],
           (saveField == 5) ? '>' : ' ', TYPE_LIST[saveTypeIndex]);
  u8g2.drawStr(0, 48, line);
 
  snprintf(line, sizeof(line), "%cSAVE  clk+ hold> dbl save tri exit", (saveField == 6) ? '>' : ' ');
  u8g2.drawStr(0, 62, line);
  u8g2.sendBuffer();
}
 
void drawCalc() {
  u8g2.clearBuffer();
 
  char title[16];
  snprintf(title, sizeof(title), "CALC %c", calcNode);
  drawHeader(title);
  u8g2.setFont(u8g2_font_5x8_tf);
 
  char line[40];
  snprintf(line, sizeof(line), "V%.1f A%.1f", calcV, calcA);
  u8g2.drawStr(0, 18, line);
 
  snprintf(line, sizeof(line), "T%.1fs D%c%d%c%d%c%dmm",
           calcTimeSec,
           (calcField == 0) ? '>' : ' ', calcDigits[0],
           (calcField == 1) ? '>' : ' ', calcDigits[1],
           (calcField == 2) ? '>' : ' ', calcDigits[2]);
  u8g2.drawStr(0, 28, line);
 
  fmtTCEventLine(line, sizeof(line));
  u8g2.drawStr(0, 38, line);
 
  snprintf(line, sizeof(line), "%sRUN", (calcField == 3) ? ">" : " ");
  u8g2.drawStr(0, 48, line);
 
  if (calcDone) {
    if (isnan(calcTravelSpeed) || isnan(calcHeatInput)) {
      u8g2.drawStr(0, 60, "TS -- HI --");
    } else {
      snprintf(line, sizeof(line), "TS%.0f HI%.3f", calcTravelSpeed, calcHeatInput);
      u8g2.drawStr(0, 60, line);
    }
  } else {
    u8g2.drawStr(0, 60, "LONG ON RUN");
  }
 
  u8g2.sendBuffer();
}
 
void drawSetTime() {
  u8g2.clearBuffer();
  char line[24];
 
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(0, 13, "SET TIME RX48");
 
  u8g2.setFont(u8g2_font_logisoso24_tf);
  snprintf(line, sizeof(line), "%d%d:%d%d", setTimeDigits[0], setTimeDigits[1], setTimeDigits[2], setTimeDigits[3]);
  u8g2.drawStr(0, 43, line);
 
  u8g2.setFont(u8g2_font_6x10_tf);
  int markerX[4] = {8, 25, 52, 69};
  u8g2.drawStr(markerX[setTimeField], 55, "^");
  u8g2.drawStr(0, 63, "clk+ dbl> hold ok");
  u8g2.sendBuffer();
}
 
void handleSetTimeButton(int ev) {
  if (ev == BTN_SINGLE) {
    setTimeDigits[setTimeField]++;
    if (setTimeField == 0) {
      if (setTimeDigits[0] > 2) setTimeDigits[0] = 0;
      if (setTimeDigits[0] == 2 && setTimeDigits[1] > 3) setTimeDigits[1] = 3;
    } else if (setTimeField == 1) {
      int maxOnes = (setTimeDigits[0] == 2) ? 3 : 9;
      if (setTimeDigits[1] > maxOnes) setTimeDigits[1] = 0;
    } else if (setTimeField == 2) {
      if (setTimeDigits[2] > 5) setTimeDigits[2] = 0;
    } else {
      if (setTimeDigits[3] > 9) setTimeDigits[3] = 0;
    }
  } else if (ev == BTN_DOUBLE) {
    setTimeField++;
    if (setTimeField > 3) setTimeField = 0;
  } else if (ev == BTN_LONG) {
    setClockFromDigits();
    currentScreen = SCREEN_LIVE;
    if (loraOK) sendSyncPacket("timeSet");
  }
}
 
void drawCurrentScreen() {
  if (currentScreen == SCREEN_LIVE) drawLive();
  else if (currentScreen == SCREEN_AVG_A) drawNodeAverage('A');
  else if (currentScreen == SCREEN_AVG_B) drawNodeAverage('B');
  else if (currentScreen == SCREEN_VERIFY) drawVerify();
  else if (currentScreen == SCREEN_RFA) drawRFactorPage('A', rIndexA);
  else if (currentScreen == SCREEN_RFB) drawRFactorPage('B', rIndexB);
  else if (currentScreen == SCREEN_CALC) drawCalc();
  else if (currentScreen == SCREEN_ALERT) drawAlert();
  else if (currentScreen == SCREEN_SAVED) drawSaved();
  else if (currentScreen == SCREEN_MANUAL_SAVE) drawManualSave();
  else if (currentScreen == SCREEN_SET_TIME) drawSetTime();
}
 
void sendSyncPacket(const char* reason) {
  if (!loraOK) return;
 
  char packet[40];
  char tbuf[6];
  fmtClock(tbuf, sizeof(tbuf));
  snprintf(packet, sizeof(packet), "SYNC,%s", tbuf);
  int st = lora.transmit(packet);
  lastSyncMs = millis();
#if DEBUG_SERIAL
  Serial.print("SYNC ");
  Serial.print(reason);
  Serial.print(" state=");
  Serial.println(st);
#endif
  lora.startReceive();
}
 
// ---------------- Button handling ----------------
int pollPgmButton() {
  bool raw = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
 
  int event = BTN_NONE;
 
  if (raw != btnLastRaw) {
    delay(DEBOUNCE_MS);
    raw = digitalRead(BUTTON_PIN);
    btnLastRaw = raw;
  }
 
  if (!btnPressed && raw == LOW) {
    btnPressed = true;
    longFired = false;
    veryLongFired = false;
    btnDownMs = now;
  }
 
  if (btnPressed && raw == LOW) {
    unsigned long held = now - btnDownMs;
    if (!veryLongFired && held >= VERY_LONG_MS) {
      veryLongFired = true;
      longFired = true;
      pendingClickCount = 0;
      event = BTN_VERY_LONG;
    } else if (!longFired && held >= LONG_MS) {
      longFired = true;
      pendingClickCount = 0;
      event = BTN_LONG;
    }
  }
 
  if (btnPressed && raw == HIGH) {
    btnPressed = false;
    if (!longFired && !veryLongFired) {
      if (pendingClickCount == 0 || (now - lastReleaseMs <= DOUBLE_MS)) {
        pendingClickCount++;
        lastReleaseMs = now;
        if (pendingClickCount >= 3) {
          pendingClickCount = 0;
          event = BTN_TRIPLE;
        }
      } else {
        pendingClickCount = 1;
        lastReleaseMs = now;
      }
    }
  }
 
  if (pendingClickCount > 0 && (now - lastReleaseMs > DOUBLE_MS)) {
    if (pendingClickCount == 1) event = BTN_SINGLE;
    else if (pendingClickCount == 2) event = BTN_DOUBLE;
    pendingClickCount = 0;
  }
 
  return event;
}
 
void nextNormalScreen() {
  if (currentScreen == SCREEN_CALC || currentScreen == SCREEN_ALERT) return;
 
  if (currentScreen == SCREEN_LIVE) currentScreen = SCREEN_AVG_A;
  else if (currentScreen == SCREEN_AVG_A) currentScreen = SCREEN_AVG_B;
  else if (currentScreen == SCREEN_AVG_B) currentScreen = SCREEN_SAVED;
  else if (currentScreen == SCREEN_SAVED) currentScreen = SCREEN_VERIFY;
  else if (currentScreen == SCREEN_VERIFY) currentScreen = SCREEN_RFA;
  else if (currentScreen == SCREEN_RFA) currentScreen = SCREEN_RFB;
  else currentScreen = SCREEN_LIVE;
}
 
void openCalcForNode(char node) {
  WeldStats& w = (node == 'B') ? weldB : weldA;
  if (isnan(w.archiveAvgV) || isnan(w.archiveAvgA)) return;
 
  calcNode = node;
  calcV = correctedVw(w.archiveAvgV, w.archiveAvgA, getRFactor(node));
  calcA = w.archiveAvgA;
  calcTimeSec = w.archiveTimeSec;
  calcDigits[0] = 0;
  calcDigits[1] = 0;
  calcDigits[2] = 0;
  calcField = 0;
  calcDone = false;
  calcTravelSpeed = NAN;
  calcHeatInput = NAN;
  currentScreen = SCREEN_CALC;
}
 
void handleManualSaveButton(int ev) {
  if (ev == BTN_TRIPLE) {
    saveDraft.active = false;
    currentScreen = (saveDraft.node == 'B') ? SCREEN_AVG_B : SCREEN_AVG_A;
    return;
  }
 
  if (ev == BTN_SINGLE) {
    if (saveField == 0) {
      saveDigits[0]++;
      if (saveDigits[0] > 4) saveDigits[0] = 0;
    } else if (saveField == 1) {
      saveDigits[1]++;
      if (saveDigits[1] > 9) saveDigits[1] = 0;
    } else if (saveField == 2) {
      saveDigits[2] = (saveDigits[2] == 0) ? 5 : 0;
    } else if (saveField == 3) {
      saveRodIndex++;
      if (saveRodIndex >= ROD_COUNT) saveRodIndex = 0;
    } else if (saveField == 4) {
      savePassIndex++;
      if (savePassIndex >= PASS_COUNT) savePassIndex = 0;
    } else if (saveField == 5) {
      saveTypeIndex++;
      if (saveTypeIndex >= TYPE_COUNT) saveTypeIndex = 0;
    }
  } else if (ev == BTN_LONG) {
    saveField++;
    if (saveField > 6) saveField = 0;
  } else if (ev == BTN_DOUBLE) {
    saveManualDraft();
  }
}
 
void handleButtonEvent(int ev) {
  if (ev == BTN_NONE) return;
 
  if (currentScreen == SCREEN_SET_TIME) {
    handleSetTimeButton(ev);
    return;
  }
 
  if (currentScreen == SCREEN_MANUAL_SAVE) {
    handleManualSaveButton(ev);
    return;
  }
 
  if (currentScreen == SCREEN_ALERT) {
    if (ev == BTN_DOUBLE || ev == BTN_LONG) {
      alertHasRecord = false;
      currentScreen = returnAfterAlert;
    }
    return;
  }
 
  if (currentScreen == SCREEN_SAVED) {
    if (ev == BTN_SINGLE && savedCount > 0) {
      savedIndex++;
      if (savedIndex >= savedCount) savedIndex = 0;
    } else if (ev == BTN_LONG) {
      nextNormalScreen();
    }
    return;
  }
 
  if (currentScreen == SCREEN_CALC) {
    if (ev == BTN_SINGLE) {
      if (calcField >= 0 && calcField <= 2) {
        calcDigits[calcField]++;
        if (calcDigits[calcField] > 9) calcDigits[calcField] = 0;
        calcDone = false;
      }
    } else if (ev == BTN_DOUBLE) {
      calcField++;
      if (calcField > 3) calcField = 0;
    } else if (ev == BTN_LONG) {
      if (calcField == 3) runCalc();
    } else if (ev == BTN_VERY_LONG) {
      currentScreen = (calcNode == 'B') ? SCREEN_AVG_B : SCREEN_AVG_A;
    }
    return;
  }
 
  if (ev == BTN_LONG) {
    nextNormalScreen();
    return;
  }
 
  if (ev == BTN_VERY_LONG) {
    if (currentScreen == SCREEN_LIVE) sendSyncPacket("manual");
    return;
  }
 
  if (currentScreen == SCREEN_RFA && ev == BTN_SINGLE) {
    rIndexA++;
    if (rIndexA >= R_COUNT) rIndexA = 0;
    syncActiveRFactor('A');
  } else if (currentScreen == SCREEN_RFB && ev == BTN_SINGLE) {
    rIndexB++;
    if (rIndexB >= R_COUNT) rIndexB = 0;
    syncActiveRFactor('B');
  } else if (currentScreen == SCREEN_AVG_A && (ev == BTN_SINGLE || ev == BTN_DOUBLE)) {
    openManualSaveForNode('A');
  } else if (currentScreen == SCREEN_AVG_B && (ev == BTN_SINGLE || ev == BTN_DOUBLE)) {
    openManualSaveForNode('B');
  }
}
 
// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("SmokeSignals RX Rev48 SaveHold BootRev starting...");
 
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
 
  syncActiveRFactor('A');
  syncActiveRFactor('B');
 
  currentScreen = SCREEN_SET_TIME;
  drawCurrentScreen();
}
 
void loop() {
  unsigned long nowMs = millis();
  bool changed = false;
 
  int ev = pollPgmButton();
  if (ev != BTN_NONE) {
    handleButtonEvent(ev);
    changed = true;
  }
 
  // Battery update ~1 Hz, same style as Rev26.
  static unsigned long lastBattMs = 0;
  if (nowMs - lastBattMs >= 1000) {
    g_battVoltage = readBatteryVoltage();
    g_battPercent = voltageToPercent(g_battVoltage);
    lastBattMs = nowMs;
  }
 
  if (loraOK && lastSyncMs > 0 && nowMs - lastSyncMs >= RARE_SYNC_INTERVAL_MS) {
    sendSyncPacket("rare");
  }
 
  // Fast RX handling, based on Rev26.
  if (loraOK && rxFlag) {
    rxEnable = false;
    rxFlag = false;
 
    String pkt;
    int st = lora.readData(pkt);
 
    rxEnable = true;
 
    if (st == RADIOLIB_ERR_NONE) {
      // Uncomment for packet debug:
      // Serial.println(pkt);
 
      bool processRx =
        currentScreen == SCREEN_LIVE ||
        currentScreen == SCREEN_AVG_A ||
        currentScreen == SCREEN_AVG_B ||
        currentScreen == SCREEN_VERIFY ||
        currentScreen == SCREEN_ALERT ||
        currentScreen == SCREEN_SAVED ||
        currentScreen == SCREEN_MANUAL_SAVE;
 
      if (processRx) {
        String id;
        uint32_t seq;
        float tempC;
        uint32_t evNum;
        float tempF;
        char eventUnit;
        uint32_t eventAgeMs;
        char eventTimeText[6];
        uint32_t distSeq;
        int distMm;
        char distanceTimeText[6];
 
        if (parseDistancePacket(pkt, id, distSeq, distMm, distanceTimeText)) {
          bool duplicateDistance = (id == lastDistanceId && distSeq == lastDistanceSeq);
          if (!duplicateDistance && currentScreen != SCREEN_ALERT && currentScreen != SCREEN_MANUAL_SAVE) {
            lastDistanceId = id;
            lastDistanceSeq = distSeq;
            // Receiver is the recordkeeper. Use receiver clock for the save time,
            // not the TC packet time, so stored records stay consistent.
            char rxDistanceTime[6];
            fmtClock(rxDistanceTime, sizeof(rxDistanceTime));
            buildDistanceAlert(distMm, rxDistanceTime);
            changed = true;
          }
          if (!seenTC) {
            seenTC = true;
            sendSyncPacket("newTC");
          }
        } else if (parseTcEventPacket(pkt, id, evNum, tempC, tempF, eventUnit, eventAgeMs, eventTimeText)) {
          lastEventTempC = tempC;
          lastEventTempF = tempF;
          lastEventUnit = eventUnit;
          lastEventTC_ID = id;
          lastEventNumber = evNum;
          lastEventMs = (eventAgeMs > 0) ? (millis() - eventAgeMs) : millis();
          // Receiver is the recordkeeper. Stamp TC events with receiver clock
          // at packet receipt instead of trusting TC packet time.
          char rxEventTime[6];
          fmtClock(rxEventTime, sizeof(rxEventTime));
          copyTimeText(lastEventTime, rxEventTime);
          if (!seenTC) {
            seenTC = true;
            sendSyncPacket("newTC");
          }
          changed = true;
        } else if (parseKisTemp(pkt, id, seq, tempC)) {
          // Legacy live TC packet. Keep support, but do not display live TC.
          lastTempC = tempC;
          lastTC_ID = id;
          lastTC_seq = seq;
          lastTCms = millis();
          changed = true;
        } else if (parseKisEvent(pkt, id, evNum, tempC)) {
          // Legacy event packet support. Time is receiver receive time.
          lastEventTempC = tempC;
          lastEventTempF = cToF(tempC);
          lastEventUnit = 'C';
          lastEventTC_ID = id;
          lastEventNumber = evNum;
          lastEventMs = millis();
          fmtClock(lastEventTime, sizeof(lastEventTime));
          if (!seenTC) {
            seenTC = true;
            sendSyncPacket("newTC");
          }
          changed = true;
        } else {
          char node;
          float v, a;
          String mode;
          if (parsePowerNodeSimple(pkt, node, v, a, mode)) {
            PowerNodeState& n = (node == 'B') ? nodeB : nodeA;
            bool newNode = false;
            if (node == 'A' && !seenNodeA) { seenNodeA = true; newNode = true; }
            if (node == 'B' && !seenNodeB) { seenNodeB = true; newNode = true; }
 
            n.vm = v;
            n.amps = a;
            n.mode = mode;
            n.lastMs = millis();
 
            updateWeldStats(node, v, a, mode, millis());
            if (newNode) sendSyncPacket("newPower");
            changed = true;
          }
        }
      }
    }
 
    lora.startReceive();
  }
 
  // Rev26-style redraw on live-style pages.
  // R-factor and calc pages redraw only from button changes/results.
  static unsigned long lastDrawMs = 0;
  bool liveStyle =
    currentScreen == SCREEN_LIVE ||
    currentScreen == SCREEN_AVG_A ||
    currentScreen == SCREEN_AVG_B ||
    currentScreen == SCREEN_VERIFY;
 
  if (changed || (liveStyle && (nowMs - lastDrawMs >= 250))) {
    lastDrawMs = nowMs;
    drawCurrentScreen();
  }
}
