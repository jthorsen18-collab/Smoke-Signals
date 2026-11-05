TC_Rev15_DistMode.ino
// TC_Rev15_DistMode.ino
//
// Kisew TC Sender Rev15
//
// - Based on Rev14.
// - Distance mode is now a simple sender/input mode.
// - No double-click is used on the TC.
// - TEMP page:
//      single press = capture TC event
//      long press   = distance page
// - DIST page:
//      single press = change selected digit
//      long press   = next field
//      single press on SAVE = send distance and return to TEMP page
//      long press on SAVE   = cycle back to hundreds without sending
// - Distance fields: hundreds 0-4, tens 0-9, ones 0 or 5, then SAVE.
// - Distance screen times out after 3 minutes with no button activity.
// - While in DIST page, thermocouple reads and LoRa receive/sync are paused so button handling is responsive.
// - TC screen shows event age; receiver uses its own clock for saved record time.
// - TC event packet uses ageMs so receiver timestamps it with receiver clock:
//      TCE,T01,<eventNo>,<tempC>,<tempF>,<unit>,<ageMs>
// - Distance packet repeats 3 times and sends no TC clock time; receiver uses its own clock:
//      DST,T01,<distanceNo>,<distanceMm>,--:--
 
#include <SPI.h>
#include <RadioLib.h>
 
#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
 
#define DEBUG_SERIAL 0
 
static const char* REV_NUMBER = "R15";
static const char* REV_TITLE  = "TC Rev15";
static const char* REV_DESC   = "DistMode";
 
// MAX31850K
#define ONE_WIRE_BUS 4
 
// Button
#define BUTTON_PIN   0
 
// Vext + OLED
#define Vext         36
#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18
#define OLED_RST_PIN 21
 
// LoRa SX1262 pins
#define LORA_NSS     8
#define LORA_SCK     9
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_DIO1   14
 
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  OLED_SCL_PIN,
  OLED_SDA_PIN,
  U8X8_PIN_NONE
);
 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
SX1262 lora = SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
bool loraOK = false;
bool lastSendOK = false;
 
volatile bool rxFlag = false;
volatile bool rxEnable = true;
static void setRxFlag(void) {
  if (!rxEnable) return;
  rxFlag = true;
}
 
static const char* NODE_ID = "T01";
 
// Timing
const uint32_t READ_INTERVAL_MS = 150;
const uint32_t OLED_INTERVAL_MS = 100;
const uint32_t EVENT_REPEAT_1_MS = 0;
const uint32_t EVENT_REPEAT_2_MS = 300;
const uint32_t EVENT_REPEAT_3_MS = 700;
const uint32_t DIST_TIMEOUT_MS = 180000UL; // 3 minutes
 
uint32_t lastReadMs = 0;
uint32_t lastOledMs = 0;
uint32_t tempRequestMs = 0;
bool tempRequestPending = false;
const uint32_t TEMP_CONVERSION_MS = 120;
 
float latestTempC = NAN;
float latestTempF = NAN;
 
bool showFahrenheit = false;
 
enum TcPage { PAGE_TEMP = 0, PAGE_DIST = 1 };
TcPage currentPage = PAGE_TEMP;
 
 
// Clock synced from receiver. This does not survive power-off.
bool clockSynced = false;
uint32_t clockBaseSeconds = 0;
uint32_t clockBaseMs = 0;
uint32_t lastSyncRxMs = 0;
 
// Button handling
const int BTN_NONE = 0;
const int BTN_SINGLE = 1;
const int BTN_DOUBLE = 2;
const int BTN_LONG = 3;
 
bool btnLastRaw = HIGH;
bool btnPressed = false;
bool longFired = false;
uint32_t btnDownMs = 0;
uint32_t lastReleaseMs = 0;
bool pendingSingle = false;
 
const uint32_t DEBOUNCE_MS = 25;
const uint32_t DOUBLE_MS = 350; // unused in Rev15
const uint32_t LONG_MS = 600;
 
// Event state
uint32_t eventNumber = 0;
float eventTempC = NAN;
float eventTempF = NAN;
char eventUnit = 'C';
char eventTime[6] = "--:--";
uint32_t eventButtonMs = 0;
 
bool eventPending = false;
uint8_t eventRepeatIndex = 0;
const uint32_t eventRepeatMs[3] = {EVENT_REPEAT_1_MS, EVENT_REPEAT_2_MS, EVENT_REPEAT_3_MS};
 
// Distance entry state
bool distanceMode = false;
int distDigits[3] = {0, 0, 0}; // hundreds, tens, ones; ones is 0 or 5 only
int distField = 0;
uint32_t distanceLastActivityMs = 0;
uint32_t distanceNumber = 0;
uint32_t lastSentDistanceMm = 0;
 
bool distancePending = false;
uint8_t distanceRepeatIndex = 0;
uint32_t distanceSendStartMs = 0;
const uint32_t distanceRepeatMs[3] = {0, 300, 700};
 
void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}
 
void OLED_resetRelease() {
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, HIGH);
}
 
float cToF(float c) {
  return c * 9.0f / 5.0f + 32.0f;
}
 
void fmtAge(char* buf, size_t len, uint32_t startMs) {
  if (startMs == 0) {
    snprintf(buf, len, "--:--");
    return;
  }
 
  uint32_t sec = (millis() - startMs) / 1000UL;
  uint32_t minutes = sec / 60UL;
  uint32_t seconds = sec % 60UL;
  snprintf(buf, len, "%lu:%02lu", (unsigned long)minutes, (unsigned long)seconds);
}
 
uint32_t currentClockSeconds() {
  if (!clockSynced) return 0;
  uint32_t elapsed = (millis() - clockBaseMs) / 1000UL;
  return (clockBaseSeconds + elapsed) % 86400UL;
}
 
void fmtClock(char* buf, size_t len) {
  if (!clockSynced) {
    snprintf(buf, len, "--:--");
    return;
  }
  uint32_t sec = currentClockSeconds();
  uint32_t hh = sec / 3600UL;
  uint32_t mm = (sec % 3600UL) / 60UL;
  snprintf(buf, len, "%02lu:%02lu", (unsigned long)hh, (unsigned long)mm);
}
 
bool looksLikeTimeText(const String& s) {
  if (s.length() != 5) return false;
  if (s.charAt(2) != ':') return false;
  return isDigit(s.charAt(0)) && isDigit(s.charAt(1)) && isDigit(s.charAt(3)) && isDigit(s.charAt(4));
}
 
void setClockFromText(const String& timeText) {
  if (!looksLikeTimeText(timeText)) return;
  int hh = timeText.substring(0, 2).toInt();
  int mm = timeText.substring(3, 5).toInt();
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return;
  clockBaseSeconds = (uint32_t)hh * 3600UL + (uint32_t)mm * 60UL;
  clockBaseMs = millis();
  clockSynced = true;
  lastSyncRxMs = millis();
}
 
bool parseSyncPacket(const String& packet, String& outTimeText) {
  if (!packet.startsWith("SYNC,")) return false;
  String t = packet.substring(5);
  t.trim();
  if (!looksLikeTimeText(t)) return false;
  outTimeText = t;
  return true;
}
 
int distanceMm() {
  return distDigits[0] * 100 + distDigits[1] * 10 + distDigits[2];
}
 
void drawDistanceOLED() {
  u8g2.clearBuffer();
  char line[32];
 
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(0, 13, "DIST R15");
 
  u8g2.setFont(u8g2_font_logisoso24_tf);
  snprintf(line, sizeof(line), "%d%d%d", distDigits[0], distDigits[1], distDigits[2]);
  u8g2.drawStr(0, 43, line);
 
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(58, 40, "mm");
 
  u8g2.setFont(u8g2_font_6x10_tf);
  if (distField < 3) {
    int markerX = 8 + distField * 16;
    u8g2.drawStr(markerX, 54, "^");
    u8g2.drawStr(0, 63, "clk+ hold>");
  } else {
    u8g2.drawStr(0, 54, "SAVE?");
    u8g2.drawStr(0, 63, "clk send hold edit");
  }
 
  u8g2.sendBuffer();
}
void drawOLED() {
  if (currentPage == PAGE_DIST) {
    drawDistanceOLED();
    return;
  }
 
  u8g2.clearBuffer();
 
  char line[32];
  showFahrenheit = false;
  float liveVal = latestTempC;
  char unit = 'C';
 
  // Main live temperature: large and readable.
  u8g2.setFont(u8g2_font_logisoso32_tf);
  if (isnan(liveVal)) {
    snprintf(line, sizeof(line), "--.-");
  } else {
    snprintf(line, sizeof(line), "%.1f", liveVal);
  }
  u8g2.drawStr(0, 34, line);
 
  // Unit beside the live number.
  u8g2.setFont(u8g2_font_10x20_tf);
  char unitLine[4];
  snprintf(unitLine, sizeof(unitLine), "%c", unit);
  u8g2.drawStr(104, 31, unitLine);
 
  // Last event line.
  u8g2.setFont(u8g2_font_6x10_tf);
  float evtVal = eventTempC;
  if (eventNumber == 0 || isnan(evtVal)) {
    snprintf(line, sizeof(line), "EVT--- --.-%c", unit);
  } else {
    snprintf(line, sizeof(line), "EVT%03lu %.1f%c", (unsigned long)eventNumber, evtVal, unit);
  }
  u8g2.drawStr(0, 50, line);
 
  // Event age + LoRa status. Receiver uses its own clock for saved record time.
  char ageLine[12];
  fmtAge(ageLine, sizeof(ageLine), eventButtonMs);
  snprintf(line, sizeof(line), "T+%s L%s %s", ageLine, loraOK ? "OK" : "NO", REV_NUMBER);
  u8g2.drawStr(0, 63, line);
 
  u8g2.sendBuffer();
}
 
void maybeReadTemp() {
  uint32_t now = millis();
 
  // Non-blocking temperature handling so the button double-click is not missed
  // while the thermocouple library waits for a conversion.
  if (!tempRequestPending) {
    if (now - lastReadMs < READ_INTERVAL_MS) return;
    sensors.requestTemperatures();
    tempRequestMs = now;
    tempRequestPending = true;
    return;
  }
 
  if (now - tempRequestMs < TEMP_CONVERSION_MS) return;
 
  tempRequestPending = false;
  lastReadMs = now;
 
  latestTempC = sensors.getTempCByIndex(0);
 
  if (latestTempC < -100.0f || latestTempC > 1000.0f) {
    latestTempC = NAN;
    latestTempF = NAN;
  } else {
    latestTempF = cToF(latestTempC);
  }
 
#if DEBUG_SERIAL
  Serial.print("TC C=");
  Serial.print(latestTempC, 2);
  Serial.print(" F=");
  Serial.println(latestTempF, 2);
#endif
}
 
void maybeOLED() {
  uint32_t now = millis();
  if (now - lastOledMs < OLED_INTERVAL_MS) return;
  lastOledMs = now;
  drawOLED();
}
 
void captureEvent() {
  if (isnan(latestTempC) || isnan(latestTempF)) return;
 
  eventNumber++;
  eventTempC = latestTempC;
  eventTempF = latestTempF;
  eventButtonMs = millis();
  eventUnit = showFahrenheit ? 'F' : 'C';
  fmtClock(eventTime, sizeof(eventTime));
 
  eventPending = true;
  eventRepeatIndex = 0;
 
#if DEBUG_SERIAL
  Serial.print("TC EVENT ");
  Serial.print(eventNumber);
  Serial.print(" C=");
  Serial.print(eventTempC, 2);
  Serial.print(" F=");
  Serial.println(eventTempF, 2);
#endif
 
  drawOLED();
}
 
void sendEventRepeat() {
  if (!loraOK) return;
  if (!eventPending) return;
  if (eventRepeatIndex >= 3) {
    eventPending = false;
    return;
  }
 
  uint32_t now = millis();
  uint32_t ageMs = now - eventButtonMs;
  if (ageMs < eventRepeatMs[eventRepeatIndex]) return;
 
  char packet[80];
  snprintf(packet, sizeof(packet), "TCE,%s,%lu,%.2f,%.2f,%c,%lu",
           NODE_ID,
           (unsigned long)eventNumber,
           eventTempC,
           eventTempF,
           eventUnit,
           (unsigned long)ageMs);
 
  int txState = lora.transmit(packet);
  lastSendOK = (txState == RADIOLIB_ERR_NONE);
 
#if DEBUG_SERIAL
  Serial.print("LoRa TC event TX ");
  Serial.print(packet);
  Serial.print(" state=");
  Serial.println(txState);
#endif
 
  lora.startReceive();
  eventRepeatIndex++;
  if (eventRepeatIndex >= 3) {
    eventPending = false;
  }
}
 
void enterDistanceMode() {
  distanceMode = true;
  currentPage = PAGE_DIST;
  distDigits[0] = 0;
  distDigits[1] = 0;
  distDigits[2] = 0;
  distField = 0;
  distanceLastActivityMs = millis();
  drawOLED();
}
 
void exitDistanceMode() {
  distanceMode = false;
  currentPage = PAGE_TEMP;
  drawOLED();
}
 
void nextTempPage() {
  if (currentPage == PAGE_TEMP) {
    enterDistanceMode();
    return;
  }
  exitDistanceMode();
}
 
void changeDistanceDigit() {
  if (distField == 0) {
    distDigits[0]++;
    if (distDigits[0] > 4) distDigits[0] = 0;
  } else if (distField == 1) {
    distDigits[1]++;
    if (distDigits[1] > 9) distDigits[1] = 0;
  } else if (distField == 2) {
    distDigits[2] = (distDigits[2] == 0) ? 5 : 0;
  } else {
    // SAVE field: single press sends distance.
    distanceLastActivityMs = millis();
    drawOLED();
    return;
  }
  distanceLastActivityMs = millis();
  drawOLED();
}
 
void nextDistanceField() {
  distField++;
  if (distField > 3) distField = 0;
  distanceLastActivityMs = millis();
  drawOLED();
}
 
void sendDistanceNow() {
  int mm = distanceMm();
  if (mm <= 0) {
    exitDistanceMode();
    return;
  }
 
  distanceNumber++;
  lastSentDistanceMm = (uint32_t)mm;
  distancePending = true;
  distanceRepeatIndex = 0;
  distanceSendStartMs = millis();
 
#if DEBUG_SERIAL
  Serial.print("DISTANCE SEND ");
  Serial.print(distanceNumber);
  Serial.print(" mm=");
  Serial.println(mm);
#endif
 
  exitDistanceMode();
}
 
void sendDistanceRepeat() {
  if (!loraOK) return;
  if (!distancePending) return;
  if (distanceRepeatIndex >= 3) {
    distancePending = false;
    return;
  }
 
  uint32_t now = millis();
  if (now - distanceSendStartMs < distanceRepeatMs[distanceRepeatIndex]) return;
 
  // Send --:-- so the receiver uses its own clock time for the saved record.
  char packet[64];
  snprintf(packet, sizeof(packet), "DST,%s,%lu,%lu,--:--",
           NODE_ID,
           (unsigned long)distanceNumber,
           (unsigned long)lastSentDistanceMm);
 
  int txState = lora.transmit(packet);
  lastSendOK = (txState == RADIOLIB_ERR_NONE);
 
#if DEBUG_SERIAL
  Serial.print("LoRa DST TX ");
  Serial.print(packet);
  Serial.print(" state=");
  Serial.println(txState);
#endif
 
  lora.startReceive();
  distanceRepeatIndex++;
  if (distanceRepeatIndex >= 3) {
    distancePending = false;
  }
}
 
void checkDistanceTimeout() {
  if (!distanceMode && currentPage != PAGE_DIST) return;
  if (millis() - distanceLastActivityMs >= DIST_TIMEOUT_MS) {
    exitDistanceMode();
  }
}
 
int pollButton() {
  bool raw = digitalRead(BUTTON_PIN);
  uint32_t now = millis();
  int event = BTN_NONE;
 
  if (raw != btnLastRaw) {
    delay(DEBOUNCE_MS);
    raw = digitalRead(BUTTON_PIN);
    btnLastRaw = raw;
  }
 
  if (!btnPressed && raw == LOW) {
    btnPressed = true;
    longFired = false;
    btnDownMs = now;
  }
 
  if (btnPressed && raw == LOW && !longFired) {
    if (now - btnDownMs >= LONG_MS) {
      longFired = true;
      event = BTN_LONG;
    }
  }
 
  if (btnPressed && raw == HIGH) {
    btnPressed = false;
    if (!longFired) {
      // Rev15: no double-click delay. Any clean quick release is a single press.
      event = BTN_SINGLE;
    }
  }
 
  return event;
}
 
void handleButtonEvent(int ev) {
  if (ev == BTN_NONE) return;
 
  if (currentPage == PAGE_DIST || distanceMode) {
    distanceMode = true;
    distanceLastActivityMs = millis();
 
    if (ev == BTN_SINGLE) {
      if (distField == 3) {
        sendDistanceNow();
      } else {
        changeDistanceDigit();
      }
    } else if (ev == BTN_LONG) {
      if (distField == 3) {
        // On SAVE, hold returns to hundreds so you can keep editing.
        distField = 0;
        drawOLED();
      } else {
        nextDistanceField();
      }
    }
    return;
  }
 
  if (ev == BTN_SINGLE) {
    showFahrenheit = false;
    captureEvent();
  } else if (ev == BTN_LONG) {
    enterDistanceMode();
  }
}
 
void handleLoraReceive() {
  if (!loraOK || !rxFlag) return;
 
  rxEnable = false;
  rxFlag = false;
 
  String pkt;
  int st = lora.readData(pkt);
 
  rxEnable = true;
 
  if (st == RADIOLIB_ERR_NONE) {
    String timeText;
    if (parseSyncPacket(pkt, timeText)) {
      setClockFromText(timeText);
#if DEBUG_SERIAL
      Serial.print("SYNC RX ");
      Serial.println(timeText);
#endif
    }
  }
 
  lora.startReceive();
}
 
void drawBootSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(0, 22, REV_TITLE);
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(0, 42, REV_DESC);
  u8g2.drawStr(0, 60, "Kisew TC Sender");
  u8g2.sendBuffer();
}
 
void setup() {
  Serial.begin(115200);
  delay(200);
 
#if DEBUG_SERIAL
  Serial.println();
  Serial.println("TC Rev15 DistMode starting...");
#endif
 
  VextON();
  delay(50);
  OLED_resetRelease();
  delay(50);
  u8g2.begin();
  drawBootSplash();
  delay(2000);
 
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  tempRequestMs = millis();
  tempRequestPending = true;
 
  pinMode(BUTTON_PIN, INPUT_PULLUP);
 
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(915.0);
 
  loraOK = (state == RADIOLIB_ERR_NONE);
 
  if (loraOK) {
    lora.setDio1Action(setRxFlag);
    rxFlag = false;
    rxEnable = true;
    lora.startReceive();
  }
 
#if DEBUG_SERIAL
  if (loraOK) {
    Serial.println("LoRa init OK (SX1262 @ 915 MHz).");
  } else {
    Serial.print("LoRa init failed, code ");
    Serial.println(state);
  }
#endif
 
  lastReadMs = millis();
  lastOledMs = millis() - OLED_INTERVAL_MS;
  drawOLED();
}
 
void loop() {
  int ev = pollButton();
  if (ev != BTN_NONE) handleButtonEvent(ev);
 
  checkDistanceTimeout();
 
  if (currentPage == PAGE_DIST || distanceMode) {
    // Distance mode: stop thermocouple reading and sync receiving so button entry stays responsive.
    sendDistanceRepeat();
    maybeOLED();
    return;
  }
 
  handleLoraReceive();
  maybeReadTemp();
  sendEventRepeat();
  sendDistanceRepeat();
  maybeOLED();
}
