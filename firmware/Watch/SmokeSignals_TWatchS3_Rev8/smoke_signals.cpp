// SmokeSignals_TWatchS3_Rev8
//
// LilyGO T-Watch S3 (ESP32-S3 + onboard SX1262) receiver
// Hardware migration of production Receiver Rev48.
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
 
 
#include <LilyGoLib.h>
#include "ui_define.h"
#include <math.h>
#include "procedure_rules.h"

// ui_define.h aliases lv_point_t for legacy chart code. Touch coordinates use
// LVGL's real integer screen-point type required by lv_indev_get_point().
#ifdef lv_point_t
#undef lv_point_t
#endif
 
#define DEBUG_SERIAL 0
 
// ---------------- Display migration shim ----------------
// Rev48 draws into a 128x64 logical canvas. This small presentation shim keeps
// those screen functions intact while rendering their text with official LVGL
// on the T-Watch's 240x240 display. It is not a hardware driver.
enum LegacyFontId { FONT_5X8, FONT_6X10, FONT_7X13B, FONT_LOGISOSO24 };
#define u8g2_font_5x8_tf       FONT_5X8
#define u8g2_font_6x10_tf      FONT_6X10
#define u8g2_font_7x13B_tf     FONT_7X13B
#define u8g2_font_logisoso24_tf FONT_LOGISOSO24

class Rev48Display {
 public:
  void begin(lv_obj_t *screen) {
    parent = screen;
    lv_obj_clean(parent);
    lv_obj_set_style_bg_color(parent, lv_color_hex(0x07111F), 0);
    lv_obj_set_style_bg_grad_color(parent, lv_color_hex(0x102B3F), 0);
    lv_obj_set_style_bg_grad_dir(parent, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    for (uint8_t i = 0; i < MAX_LINES; i++) {
      labels[i] = lv_label_create(parent);
      lv_obj_set_style_text_color(labels[i], i == 0 ? lv_color_hex(0x63C7FF) : lv_color_white(), 0);
      lv_obj_add_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
  void end() {
    parent = nullptr;
    for (uint8_t i = 0; i < MAX_LINES; i++) labels[i] = nullptr;
  }
  void clearBuffer() { count = 0; }
  void setFont(LegacyFontId selected) { font = selected; }
  void drawStr(int x, int y, const char *text) {
    if (count >= MAX_LINES) return;
    lines[count].x = x;
    lines[count].y = y;
    lines[count].font = font;
    strncpy(lines[count].text, text ? text : "", sizeof(lines[count].text) - 1);
    lines[count].text[sizeof(lines[count].text) - 1] = '\0';
    count++;
  }
  void sendBuffer() {
    if (!parent) return;
    for (uint8_t i = 0; i < count; i++) {
      lv_label_set_text(labels[i], lines[i].text);
      lv_obj_set_style_text_font(labels[i], lvFont(lines[i].font), 0);
      int16_t top = (int16_t)(lines[i].y * 3.45f) - fontHeight(lines[i].font);
      if (top < 0) top = 0;
      lv_obj_set_pos(labels[i], (int16_t)(lines[i].x * 1.75f), top);
      lv_obj_remove_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
    }
    for (uint8_t i = count; i < MAX_LINES; i++) {
      lv_obj_add_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_timer_handler();
  }

 private:
  static const uint8_t MAX_LINES = 16;
  struct Line { int16_t x; int16_t y; LegacyFontId font; char text[64]; };
  Line lines[MAX_LINES];
  lv_obj_t *parent = nullptr;
  lv_obj_t *labels[MAX_LINES] = {};
  uint8_t count = 0;
  LegacyFontId font = FONT_5X8;

  const lv_font_t *lvFont(LegacyFontId id) {
    if (id == FONT_LOGISOSO24) return &lv_font_montserrat_24;
    if (id == FONT_7X13B) return &lv_font_montserrat_18;
    if (id == FONT_6X10) return &lv_font_montserrat_16;
    return &lv_font_montserrat_12;
  }
  int16_t fontHeight(LegacyFontId id) {
    if (id == FONT_LOGISOSO24) return 24;
    if (id == FONT_7X13B) return 18;
    if (id == FONT_6X10) return 16;
    return 12;
  }
};

Rev48Display u8g2;
bool smokeUiActive = false;
 
// ---------------- LoRa ----------------
// LilyGoLib owns the Module/pins/SPI instance and exports the onboard radio.
SX1262 &lora = radio;
bool loraOK = false;
 
volatile bool rxFlag = false;
volatile bool rxEnable = true;
 
static void setRxFlag(void) {
  if (!rxEnable) return;
  rxFlag = true;
}
 
// ---------------- RTC ----------------
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
bool touchRaw = HIGH;
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
bool temperaturePopupPending = false;
static void showTemperaturePopup();
 
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
  int16_t procedureIndex = -1;
  uint8_t procedureFlags = 0;
  char wpsText[20] = "--";
  char procedurePass[28] = "--";
  char thicknessText[24] = "--";
  char electrodeText[12] = "--";
};
 
ResultRecord alertRecord;
bool alertHasRecord = false;
ScreenMode returnAfterAlert = SCREEN_AVG_A;
 
const int MAX_SAVED_RECORDS = 10;
ResultRecord savedRecords[MAX_SAVED_RECORDS];
int savedCount = 0;
int savedIndex = 0;
int savedDetailPage = 0;
int alertDetailPage = 0;
 
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
// Distance is deliberately first. Procedure selection then follows the
// workbook Picker hierarchy instead of exposing the 47-row CONFIG table.
// 0 H, 1 T, 2 ones, 3 WPS, 4 pass, 5 thickness, 6 electrode, 7 review/save.
int saveField = 0;
int saveRodIndex = 3;          // default 1/8 3.2
int savePassIndex = 3;         // default A1
int saveTypeIndex = 0;         // default 2G/F
int selectedProcedureIndex = 42; // BW-10 Rev.3 A1; retained until changed.
 
const char* ROD_LIST[] = {"5/64 2.0", "3/32 2.4", "3/32 2.5", "1/8 3.2", "5/32 4.0", "3/16 4.8", "1/4 6.0"};
const int ROD_COUNT = sizeof(ROD_LIST) / sizeof(ROD_LIST[0]);
const char* PASS_LIST[] = {"Root", "Fill", "Cap", "A1", "A2", "B1", "C1"};
const int PASS_COUNT = sizeof(PASS_LIST) / sizeof(PASS_LIST[0]);
const char* TYPE_LIST[] = {"2G/F", "5G/F"};
const int TYPE_COUNT = sizeof(TYPE_LIST) / sizeof(TYPE_LIST[0]);

static uint8_t evaluateProcedure(const ProcedureRule &rule, float volts,
                                 float amps, float travel, float heat) {
  uint8_t flags = 0;
  if (!isnan(amps) && amps >= rule.minAmps && amps <= rule.maxAmps) flags |= 0x01;
  if (!isnan(volts) && volts >= rule.minVolts && volts <= rule.maxVolts) flags |= 0x02;
  if (!isnan(travel) && travel >= rule.minTravel && travel <= rule.maxTravel) flags |= 0x04;
  if (!isnan(heat) && heat >= rule.minHeat && heat <= rule.maxHeat) flags |= 0x08;
  return flags;
}

static const char *procedureStatus(uint8_t flags) {
  return (flags & 0x0F) == 0x0F ? "PASS" : "OUT";
}

static bool sameProcedureText(const char *a, const char *b) {
  if (!a || !b) return false;
  return strcmp(a, b) == 0;
}

static const char *procedureFieldText(const ProcedureRule &rule, int field) {
  if (field == 3) return rule.wps;
  if (field == 4) return rule.pass;
  if (field == 5) return rule.thickness;
  return rule.electrode;
}

static bool procedureRuleEligible(int ruleIndex, int field) {
  const ProcedureRule &candidate = PROCEDURE_RULES[ruleIndex];
  const ProcedureRule &selected = PROCEDURE_RULES[selectedProcedureIndex];
  if (field >= 4 && !sameProcedureText(candidate.wps, selected.wps)) return false;
  if (field >= 5 && !sameProcedureText(candidate.pass, selected.pass)) return false;
  if (field >= 6 && !sameProcedureText(candidate.thickness, selected.thickness)) return false;
  return true;
}

static bool procedureChoiceSeenEarlier(int ruleIndex, int field) {
  const char *value = procedureFieldText(PROCEDURE_RULES[ruleIndex], field);
  for (int i = 0; i < ruleIndex; i++) {
    if (!procedureRuleEligible(i, field)) continue;
    if (sameProcedureText(procedureFieldText(PROCEDURE_RULES[i], field), value)) return true;
  }
  return false;
}

static int procedureChoiceCount(int field) {
  int count = 0;
  for (int i = 0; i < PROCEDURE_RULE_COUNT; i++) {
    if (procedureRuleEligible(i, field) && !procedureChoiceSeenEarlier(i, field)) count++;
  }
  return count;
}

static int procedureChoiceRuleAt(int field, int position) {
  int found = 0;
  for (int i = 0; i < PROCEDURE_RULE_COUNT; i++) {
    if (!procedureRuleEligible(i, field) || procedureChoiceSeenEarlier(i, field)) continue;
    if (found == position) return i;
    found++;
  }
  return selectedProcedureIndex;
}

static int currentProcedureChoicePosition(int field) {
  const char *selectedValue = procedureFieldText(PROCEDURE_RULES[selectedProcedureIndex], field);
  int found = 0;
  for (int i = 0; i < PROCEDURE_RULE_COUNT; i++) {
    if (!procedureRuleEligible(i, field) || procedureChoiceSeenEarlier(i, field)) continue;
    if (sameProcedureText(procedureFieldText(PROCEDURE_RULES[i], field), selectedValue)) return found;
    found++;
  }
  return 0;
}

static bool procedureCandidateMatches(int ruleIndex, int changedField,
                                      const char *changedValue,
                                      const ProcedureRule &oldSelection,
                                      bool preserveLater) {
  const ProcedureRule &candidate = PROCEDURE_RULES[ruleIndex];
  if (changedField >= 4 && !sameProcedureText(candidate.wps, oldSelection.wps)) return false;
  if (changedField >= 5 && !sameProcedureText(candidate.pass, oldSelection.pass)) return false;
  if (changedField >= 6 && !sameProcedureText(candidate.thickness, oldSelection.thickness)) return false;
  if (!sameProcedureText(procedureFieldText(candidate, changedField), changedValue)) return false;

  if (!preserveLater) return true;
  if (changedField < 4 && !sameProcedureText(candidate.pass, oldSelection.pass)) return false;
  if (changedField < 5 && !sameProcedureText(candidate.thickness, oldSelection.thickness)) return false;
  if (changedField < 6 && !sameProcedureText(candidate.electrode, oldSelection.electrode)) return false;
  return true;
}

static void changeProcedureChoice(int field, int delta) {
  if (field < 3 || field > 6 || delta == 0) return;
  selectedProcedureIndex = constrain(selectedProcedureIndex, 0, PROCEDURE_RULE_COUNT - 1);
  int count = procedureChoiceCount(field);
  if (count <= 0) return;

  int position = currentProcedureChoicePosition(field);
  position = (position + delta) % count;
  if (position < 0) position += count;

  int choiceRuleIndex = procedureChoiceRuleAt(field, position);
  const char *changedValue = procedureFieldText(PROCEDURE_RULES[choiceRuleIndex], field);
  ProcedureRule oldSelection = PROCEDURE_RULES[selectedProcedureIndex];

  // Retain later Picker choices when the new combination supports them.
  // Otherwise fall back to the first valid row, which resets stale choices.
  int replacement = -1;
  for (int preserve = 1; preserve >= 0 && replacement < 0; preserve--) {
    for (int i = 0; i < PROCEDURE_RULE_COUNT; i++) {
      if (procedureCandidateMatches(i, field, changedValue, oldSelection, preserve != 0)) {
        replacement = i;
        break;
      }
    }
  }
  if (replacement >= 0) selectedProcedureIndex = replacement;
}

static const char *procedureMetricState(float value, float minimum, float maximum) {
  if (isnan(value)) return "--";
  if (value < minimum) return "LOW";
  if (value > maximum) return "HIGH";
  return "OK";
}
 
 
String lastDistanceId = "";
uint32_t lastDistanceSeq = 0;
 
// Explicit prototypes below prevent Arduino IDE from auto-generating prototypes
// above the ResultRecord struct, which can cause compile errors.
void storeRecord(const ResultRecord& rec);
void openManualSaveForNode(char node);
void drawManualSave();
void handleManualSaveButton(int ev);
void handleManualSaveDelta(int delta);
void changeManualSaveField(int delta);
void saveManualDraft();
void sendSyncPacket(const char* reason);
 
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
  if (g_rtcTimeValid) {
    RTC_DateTime now = instance.rtc.getDateTime();
    instance.rtc.setDateTime(now.getYear(), now.getMonth(), now.getDay(), hh, mm, 0);
  }
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
  return instance.pmu.getBattVoltage() / 1000.0f;
}
 
int readBatteryPercent() {
  return instance.pmu.getBatteryPercent();
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
 
  u8g2.drawStr(0, 8, "T-WATCH S3 R8 LIVE");
 
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
 
  u8g2.drawStr(0, 8, "VERIFY T-WATCH R8");
 
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
  savedDetailPage = 0;
 
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
  selectedProcedureIndex = constrain(selectedProcedureIndex, 0, PROCEDURE_RULE_COUNT - 1);
  const ProcedureRule &rule = PROCEDURE_RULES[selectedProcedureIndex];
  rec.procedureIndex = selectedProcedureIndex;
  rec.procedureFlags = evaluateProcedure(rule, rec.vw, rec.amps,
                                         rec.travelSpeed, rec.heatInput);
  copySmallText(rec.wpsText, sizeof(rec.wpsText), rule.wps);
  copySmallText(rec.procedurePass, sizeof(rec.procedurePass), rule.pass);
  copySmallText(rec.thicknessText, sizeof(rec.thicknessText), rule.thickness);
  copySmallText(rec.electrodeText, sizeof(rec.electrodeText), rule.electrode);
  copyTimeText(rec.distanceTime, saveDraft.saveTime);
 
  rec.hasTcEvent = saveDraft.hasTcEvent;
  rec.tcTempC = saveDraft.tcTempC;
  rec.tcTempF = saveDraft.tcTempF;
  rec.tcUnit = saveDraft.tcUnit;
  copyTimeText(rec.tcTime, saveDraft.tcTime);
  copySmallText(rec.rodText, sizeof(rec.rodText), rule.electrode);
  copySmallText(rec.passText, sizeof(rec.passText), rule.pass);
  copySmallText(rec.weldType, sizeof(rec.weldType), rule.wps);
 
  alertRecord = rec;
  storeRecord(rec);
  alertHasRecord = true;
  alertDetailPage = 0;
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

  selectedProcedureIndex = constrain(selectedProcedureIndex, 0, PROCEDURE_RULE_COUNT - 1);
  const ProcedureRule &rule = PROCEDURE_RULES[selectedProcedureIndex];
  alertRecord.procedureIndex = selectedProcedureIndex;
  alertRecord.procedureFlags = evaluateProcedure(rule, alertRecord.vw,
                                                  alertRecord.amps,
                                                  alertRecord.travelSpeed,
                                                  alertRecord.heatInput);
  copySmallText(alertRecord.wpsText, sizeof(alertRecord.wpsText), rule.wps);
  copySmallText(alertRecord.procedurePass, sizeof(alertRecord.procedurePass), rule.pass);
  copySmallText(alertRecord.thicknessText, sizeof(alertRecord.thicknessText), rule.thickness);
  copySmallText(alertRecord.electrodeText, sizeof(alertRecord.electrodeText), rule.electrode);
 
  storeRecord(alertRecord);
 
  alertHasRecord = true;
  alertDetailPage = 0;
  returnAfterAlert = currentScreen;
  currentScreen = SCREEN_ALERT;
}
 
static void drawResultRecord(const ResultRecord &record, int page,
                             const char *recordLabel) {
  char line[64];
  page = constrain(page, 0, 2);

  const ProcedureRule *rule = nullptr;
  if (record.procedureIndex >= 0 && record.procedureIndex < PROCEDURE_RULE_COUNT) {
    rule = &PROCEDURE_RULES[record.procedureIndex];
  }

  snprintf(line, sizeof(line), "%s %d/3 %s", recordLabel, page + 1,
           record.distanceTime);
  u8g2.drawStr(0, 8, line);

  if (page == 0) {
    snprintf(line, sizeof(line), "RESULT: %s", procedureStatus(record.procedureFlags));
    u8g2.drawStr(0, 18, line);
    snprintf(line, sizeof(line), "WPS %s", record.wpsText);
    u8g2.drawStr(0, 28, line);
    snprintf(line, sizeof(line), "PASS %s", record.procedurePass);
    u8g2.drawStr(0, 38, line);
    snprintf(line, sizeof(line), "THK %s", record.thicknessText);
    u8g2.drawStr(0, 48, line);
    snprintf(line, sizeof(line), "ROD %s", record.electrodeText);
    u8g2.drawStr(0, 62, line);
    return;
  }

  if (page == 1) {
    snprintf(line, sizeof(line), "NODE %c  DIST %d mm", record.node,
             record.distanceMm);
    u8g2.drawStr(0, 18, line);
    snprintf(line, sizeof(line), "WELD TIME %.1f s", record.timeSec);
    u8g2.drawStr(0, 28, line);
    snprintf(line, sizeof(line), "WORK V %.1f", record.vw);
    u8g2.drawStr(0, 38, line);
    snprintf(line, sizeof(line), "AVG AMPS %.1f", record.amps);
    u8g2.drawStr(0, 48, line);
    if (record.hasTcEvent) {
      snprintf(line, sizeof(line), "TC %.1fC %.1fF %s", record.tcTempC,
               record.tcTempF, record.tcTime);
    } else {
      snprintf(line, sizeof(line), "TC --.-C --.-F --:--");
    }
    u8g2.drawStr(0, 62, line);
    return;
  }

  const char *ampsState = "--";
  const char *voltsState = "--";
  const char *travelState = "--";
  const char *heatState = "--";
  if (rule) {
    ampsState = procedureMetricState(record.amps, rule->minAmps, rule->maxAmps);
    voltsState = procedureMetricState(record.vw, rule->minVolts, rule->maxVolts);
    travelState = procedureMetricState(record.travelSpeed, rule->minTravel,
                                       rule->maxTravel);
    heatState = procedureMetricState(record.heatInput, rule->minHeat,
                                     rule->maxHeat);
  }
  snprintf(line, sizeof(line), "VOLTS %.1f %s", record.vw, voltsState);
  u8g2.drawStr(0, 18, line);
  snprintf(line, sizeof(line), "AMPS %.1f %s", record.amps, ampsState);
  u8g2.drawStr(0, 28, line);
  snprintf(line, sizeof(line), "TRAVEL %.0f %s", record.travelSpeed, travelState);
  u8g2.drawStr(0, 38, line);
  snprintf(line, sizeof(line), "HEAT %.2f %s", record.heatInput, heatState);
  u8g2.drawStr(0, 48, line);
  u8g2.drawStr(0, 62, "tap pages");
}

void drawAlert() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);

  if (!alertHasRecord || !alertRecord.valid) {
    u8g2.drawStr(0, 10, "SAVED");
    u8g2.drawStr(0, 26, "NO AVG READY");
    u8g2.drawStr(0, 62, "double dismiss");
    u8g2.sendBuffer();
    return;
  }

  drawResultRecord(alertRecord, alertDetailPage, "SAVED");
  u8g2.sendBuffer();
}

void drawSaved() {
  u8g2.clearBuffer();
  char label[20];
  u8g2.setFont(u8g2_font_5x8_tf);

  if (savedCount <= 0) {
    u8g2.drawStr(0, 9, "SAVED");
    u8g2.drawStr(0, 28, "NO RECORDS");
    u8g2.sendBuffer();
    return;
  }

  if (savedIndex < 0) savedIndex = 0;
  if (savedIndex >= savedCount) savedIndex = savedCount - 1;
  snprintf(label, sizeof(label), "REC %d/%d", savedIndex + 1, savedCount);
  drawResultRecord(savedRecords[savedIndex], savedDetailPage, label);
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

  selectedProcedureIndex = constrain(selectedProcedureIndex, 0, PROCEDURE_RULE_COUNT - 1);
  const ProcedureRule &rule = PROCEDURE_RULES[selectedProcedureIndex];
  snprintf(line, sizeof(line), "SAVE R8 %c %s", saveDraft.node, saveDraft.saveTime);
  u8g2.drawStr(0, 8, line);

  if (saveField <= 2) {
    snprintf(line, sizeof(line), "DISTANCE %d mm", saveDistanceMm());
    u8g2.drawStr(0, 20, line);
    snprintf(line, sizeof(line), "%c%d  %c%d  %c%d",
             (saveField == 0) ? '>' : ' ', saveDigits[0],
             (saveField == 1) ? '>' : ' ', saveDigits[1],
             (saveField == 2) ? '>' : ' ', saveDigits[2]);
    u8g2.drawStr(0, 34, line);
    const char *digitName = (saveField == 0) ? "HUNDREDS" :
                            ((saveField == 1) ? "TENS" : "ONES (0/5)");
    snprintf(line, sizeof(line), "STEP %d/3 %s", saveField + 1, digitName);
    u8g2.drawStr(0, 48, line);
    u8g2.drawStr(0, 62, "swipe value  hold next");
    u8g2.sendBuffer();
    return;
  }

  if (saveField >= 3 && saveField <= 6) {
    const char *fieldName = (saveField == 3) ? "WPS" :
                            ((saveField == 4) ? "PASS" :
                            ((saveField == 5) ? "THICKNESS" : "ELECTRODE"));
    int count = procedureChoiceCount(saveField);
    int position = currentProcedureChoicePosition(saveField);
    snprintf(line, sizeof(line), "%s %d/%d", fieldName, position + 1, count);
    u8g2.drawStr(0, 20, line);
    u8g2.drawStr(0, 34, procedureFieldText(rule, saveField));

    if (saveField == 4) snprintf(line, sizeof(line), "WPS %s", rule.wps);
    else if (saveField == 5) snprintf(line, sizeof(line), "PASS %s", rule.pass);
    else if (saveField == 6) snprintf(line, sizeof(line), "THK %s", rule.thickness);
    else snprintf(line, sizeof(line), "A%.1f V%.1f T%.1fs",
                  saveDraft.avgA, saveDraft.avgVw, saveDraft.timeSec);
    u8g2.drawStr(0, 48, line);
    u8g2.drawStr(0, 62, "swipe up/down  hold next");
    u8g2.sendBuffer();
    return;
  }

  u8g2.drawStr(0, 18, rule.wps);
  u8g2.drawStr(0, 28, rule.pass);
  u8g2.drawStr(0, 38, rule.thickness);
  u8g2.drawStr(0, 48, rule.electrode);
  snprintf(line, sizeof(line), "L%d  double SAVE", saveDistanceMm());
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
  u8g2.drawStr(0, 13, "SET TIME T-WATCH R8");
 
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
 
// ---------------- Touch handling ----------------
// A tap/hold sequence produces the same event codes as the Rev48 program
// button, preserving all existing menu and field-edit behavior.
int pollTouchInput() {
  bool raw = touchRaw;
  unsigned long now = millis();
 
  int event = BTN_NONE;
 
  if (raw != btnLastRaw) {
    delay(DEBOUNCE_MS);
    raw = touchRaw;
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
 
void handleManualSaveDelta(int delta) {
  if (delta == 0) return;
  if (saveField == 0) {
    saveDigits[0] = (saveDigits[0] + delta) % 5;
    if (saveDigits[0] < 0) saveDigits[0] += 5;
  } else if (saveField == 1) {
    saveDigits[1] = (saveDigits[1] + delta) % 10;
    if (saveDigits[1] < 0) saveDigits[1] += 10;
  } else if (saveField == 2) {
    saveDigits[2] = (saveDigits[2] == 0) ? 5 : 0;
  } else if (saveField >= 3 && saveField <= 6) {
    changeProcedureChoice(saveField, delta);
  }
}

void changeManualSaveField(int delta) {
  if (delta == 0) return;
  saveField = (saveField + delta) % 8;
  if (saveField < 0) saveField += 8;
}

void handleManualSaveButton(int ev) {
  if (ev == BTN_TRIPLE) {
    saveDraft.active = false;
    currentScreen = (saveDraft.node == 'B') ? SCREEN_AVG_B : SCREEN_AVG_A;
    return;
  }
 
  if (ev == BTN_SINGLE) {
    handleManualSaveDelta(1);
  } else if (ev == BTN_LONG) {
    changeManualSaveField(1);
  } else if (ev == BTN_DOUBLE) {
    if (saveField == 7) saveManualDraft();
    else changeManualSaveField(1);
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
    if (ev == BTN_SINGLE) {
      alertDetailPage = (alertDetailPage + 1) % 3;
    } else if (ev == BTN_DOUBLE || ev == BTN_LONG) {
      alertHasRecord = false;
      alertDetailPage = 0;
      currentScreen = returnAfterAlert;
    }
    return;
  }
 
  if (currentScreen == SCREEN_SAVED) {
    if (ev == BTN_SINGLE && savedCount > 0) {
      savedDetailPage = (savedDetailPage + 1) % 3;
    } else if (ev == BTN_DOUBLE && savedCount > 0) {
      savedIndex++;
      if (savedIndex >= savedCount) savedIndex = 0;
      savedDetailPage = 0;
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
 
// ---------------- Factory integration ----------------
void smokeSignalsBegin() {
  Serial.println("Smoke Signals T-Watch S3 Rev 8 starting...");

  uint32_t deviceStatus = instance.getDeviceProbe();

  g_rtcTimeValid = (deviceStatus & HW_RTC_ONLINE) != 0;

  int st = RADIOLIB_ERR_CHIP_NOT_FOUND;
  if (deviceStatus & HW_RADIO_ONLINE) {
    // LilyGoLib's begin() establishes the RadioLib defaults used by Rev48.
    // These two calls are board requirements from the uploaded T-Watch S3
    // SX1262 examples; only the carrier frequency changes to Rev48's 915 MHz.
    st = lora.setFrequency(915.0);
    if (st == RADIOLIB_ERR_NONE) st = lora.setTCXO(3.0);
    if (st == RADIOLIB_ERR_NONE) st = lora.setDio2AsRfSwitch();
  }
  if (st == RADIOLIB_ERR_NONE) {
    loraOK = true;
    Serial.println("LoRa init OK.");
 
    lora.setPacketReceivedAction(setRxFlag);
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
  g_battPercent = readBatteryPercent();
 
  syncActiveRFactor('A');
  syncActiveRFactor('B');

  // The factory firmware has already loaded the PCF8563 into system time.
  // Seed Rev48's unchanged HH:MM clock from that official RTC path.
  RTC_DateTime rtcNow = instance.rtc.getDateTime();
  clockBaseSeconds = (uint32_t)rtcNow.getHour() * 3600UL +
                     (uint32_t)rtcNow.getMinute() * 60UL + rtcNow.getSecond();
  clockBaseMs = millis();
  clockSet = true;
  currentScreen = SCREEN_LIVE;
}

void previousNormalScreen() {
  if (currentScreen == SCREEN_CALC || currentScreen == SCREEN_ALERT ||
      currentScreen == SCREEN_MANUAL_SAVE || currentScreen == SCREEN_SET_TIME) return;

  if (currentScreen == SCREEN_LIVE) currentScreen = SCREEN_RFB;
  else if (currentScreen == SCREEN_RFB) currentScreen = SCREEN_RFA;
  else if (currentScreen == SCREEN_RFA) currentScreen = SCREEN_VERIFY;
  else if (currentScreen == SCREEN_VERIFY) currentScreen = SCREEN_SAVED;
  else if (currentScreen == SCREEN_SAVED) currentScreen = SCREEN_AVG_B;
  else if (currentScreen == SCREEN_AVG_B) currentScreen = SCREEN_AVG_A;
  else currentScreen = SCREEN_LIVE;
}
 
void smokeSignalsLoop() {
  unsigned long nowMs = millis();
  bool changed = false;

  static unsigned long lastRtcRefreshMs = 0;
  if (g_rtcTimeValid && nowMs - lastRtcRefreshMs >= 60000UL) {
    RTC_DateTime rtcNow = instance.rtc.getDateTime();
    clockBaseSeconds = (uint32_t)rtcNow.getHour() * 3600UL +
                       (uint32_t)rtcNow.getMinute() * 60UL + rtcNow.getSecond();
    clockBaseMs = nowMs;
    lastRtcRefreshMs = nowMs;
  }
 
  if (smokeUiActive) {
    int ev = pollTouchInput();
    if (ev != BTN_NONE) {
      handleButtonEvent(ev);
      changed = true;
    }
  }
 
  // Battery update ~1 Hz, same style as Rev26.
  static unsigned long lastBattMs = 0;
  if (nowMs - lastBattMs >= 1000) {
    g_battVoltage = readBatteryVoltage();
    g_battPercent = readBatteryPercent();
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
 
      bool processRx = !smokeUiActive ||
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
          temperaturePopupPending = true;
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
          temperaturePopupPending = true;
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
 
  if (smokeUiActive && (changed || (liveStyle && (nowMs - lastDrawMs >= 250)))) {
    lastDrawMs = nowMs;
    drawCurrentScreen();
  }

  if (temperaturePopupPending) showTemperaturePopup();
}

static lv_obj_t *smokeParent = nullptr;
static lv_obj_t *temperaturePopup = nullptr;
static lv_point_t smokeTouchStart = {0, 0};
static bool smokeSwipeInProgress = false;

static void dismissTemperaturePopup(lv_event_t *event) {
  if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
  if (temperaturePopup) {
    lv_obj_delete(temperaturePopup);
    temperaturePopup = nullptr;
  }
}

static void showTemperaturePopup() {
  temperaturePopupPending = false;
  lv_display_trigger_activity(nullptr);
  instance.vibrator();

  if (temperaturePopup) lv_obj_delete(temperaturePopup);
  temperaturePopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(temperaturePopup, 218, 190);
  lv_obj_center(temperaturePopup);
  lv_obj_set_style_radius(temperaturePopup, 16, 0);
  lv_obj_set_style_bg_color(temperaturePopup, lv_color_hex(0x102B3F), 0);
  lv_obj_set_style_bg_opa(temperaturePopup, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(temperaturePopup, lv_color_hex(0x63C7FF), 0);
  lv_obj_set_style_border_width(temperaturePopup, 2, 0);
  lv_obj_set_style_pad_all(temperaturePopup, 12, 0);

  lv_obj_t *title = lv_label_create(temperaturePopup);
  lv_label_set_text(title, "TEMPERATURE EVENT");
  lv_obj_set_style_text_color(title, lv_color_hex(0x63C7FF), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *value = lv_label_create(temperaturePopup);
  char text[96];
  snprintf(text, sizeof(text), "%.1f C   %.1f F\n%s  Event %lu\nReceived %s",
           lastEventTempC, lastEventTempF, lastEventTC_ID.c_str(),
           (unsigned long)lastEventNumber, lastEventTime);
  lv_label_set_text(value, text);
  lv_obj_set_style_text_color(value, lv_color_white(), 0);
  lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(value, LV_ALIGN_CENTER, 0, -6);

  lv_obj_t *dismiss = lv_button_create(temperaturePopup);
  lv_obj_set_size(dismiss, 150, 42);
  lv_obj_align(dismiss, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(dismiss, lv_color_hex(0x1677A6), 0);
  lv_obj_add_event_cb(dismiss, dismissTemperaturePopup, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *label = lv_label_create(dismiss);
  lv_label_set_text(label, "DISMISS");
  lv_obj_center(label);
}

static void smokeTouchEvent(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  lv_indev_t *indev = lv_indev_active();

  if (code == LV_EVENT_PRESSED) {
    if (indev) lv_indev_get_point(indev, &smokeTouchStart);
    smokeSwipeInProgress = false;
    touchRaw = LOW;
  }

  if (code == LV_EVENT_PRESSING && indev) {
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    int dx = point.x - smokeTouchStart.x;
    int dy = point.y - smokeTouchStart.y;
    if ((abs(dx) > 24 || abs(dy) > 24) && abs(dx) != abs(dy)) {
      smokeSwipeInProgress = true;
      touchRaw = HIGH;
      btnPressed = false;
      pendingClickCount = 0;
    }
  }

  if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) && indev) {
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    int dx = point.x - smokeTouchStart.x;
    int dy = point.y - smokeTouchStart.y;
    touchRaw = HIGH;
    if (smokeSwipeInProgress && (abs(dx) > 40 || abs(dy) > 40)) {
      btnPressed = false;
      pendingClickCount = 0;

      bool vertical = abs(dy) > abs(dx);
      if (currentScreen == SCREEN_MANUAL_SAVE) {
        if (vertical) handleManualSaveDelta((dy < 0) ? 1 : -1);
        else changeManualSaveField((dx < 0) ? 1 : -1);
      } else if (currentScreen == SCREEN_ALERT) {
        int delta = vertical ? ((dy < 0) ? 1 : -1) : ((dx < 0) ? 1 : -1);
        alertDetailPage = (alertDetailPage + delta) % 3;
        if (alertDetailPage < 0) alertDetailPage += 3;
      } else if (currentScreen == SCREEN_SAVED) {
        if (vertical) {
          savedDetailPage = (savedDetailPage + ((dy < 0) ? 1 : -1)) % 3;
          if (savedDetailPage < 0) savedDetailPage += 3;
        } else if (savedCount > 0) {
          savedIndex = (savedIndex + ((dx < 0) ? 1 : -1)) % savedCount;
          if (savedIndex < 0) savedIndex += savedCount;
          savedDetailPage = 0;
        }
      } else if (!vertical) {
        if (dx < 0) nextNormalScreen();
        else previousNormalScreen();
      }
      drawCurrentScreen();
    }
    smokeSwipeInProgress = false;
  }
  lv_display_trigger_activity(nullptr);
}

void ui_smoke_exit(lv_obj_t *parent) {
  (void)parent;
  smokeUiActive = false;
  touchRaw = HIGH;
  btnLastRaw = HIGH;
  btnPressed = false;
  pendingClickCount = 0;
  u8g2.end();
  smokeParent = nullptr;
  set_low_power_mode_flag(true);
}

static void smokeBackEvent(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  if (code != LV_EVENT_RELEASED) return;
  ui_smoke_exit(smokeParent);
  menu_show();
}

void ui_smoke_enter(lv_obj_t *parent) {
  set_low_power_mode_flag(false);
  smokeParent = parent;
  u8g2.begin(parent);

  lv_obj_t *touchSurface = lv_obj_create(parent);
  lv_obj_remove_style_all(touchSurface);
  lv_obj_set_size(touchSurface, lv_pct(100), lv_pct(100));
  lv_obj_center(touchSurface);
  lv_obj_add_flag(touchSurface, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(touchSurface, smokeTouchEvent, LV_EVENT_ALL, nullptr);

  lv_obj_t *back = lv_button_create(parent);
  lv_obj_set_size(back, 34, 30);
  lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -2, 2);
  lv_obj_set_style_bg_opa(back, LV_OPA_50, 0);
  lv_obj_add_event_cb(back, smokeBackEvent, LV_EVENT_ALL, nullptr);
  lv_obj_t *backLabel = lv_label_create(back);
  lv_label_set_text(backLabel, LV_SYMBOL_LEFT);
  lv_obj_center(backLabel);
  lv_obj_move_foreground(back);

  smokeUiActive = true;
  drawCurrentScreen();
}

app_t ui_smoke_main = {
  .setup_func_cb = ui_smoke_enter,
  .exit_func_cb = ui_smoke_exit,
  .user_data = nullptr,
};

void smokeSignalsOpenDefault() {
  lv_obj_t *appPanel = lv_obj_get_child(main_screen, 1);
  ui_smoke_enter(appPanel);
  menu_hidden();
}
