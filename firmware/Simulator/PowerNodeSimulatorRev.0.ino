// Maskwa Connect
// PowerNode Simulator Rev.0
//
// Target hardware:
//   Heltec WiFi LoRa 32 V3
//
// Purpose:
//   Bench simulator for testing the Maskwa Connect receiver
//   without a welder or live Power Node sensors.
//
// Production-compatible packet format:
//   P,<node>,<machine volts>,<amps>,<mode>
//
// Example:
//   P,A,27.18,100.0,DC
//
// Rev.0 changes:
//   - Renamed as the Maskwa Connect PowerNode Simulator
//   - One transmission per second (1 Hz)
//   - Improved OLED splash and operating status
//   - Added successful-transmission counter
//   - Added realistic deterministic voltage/current variation
//   - DC-only by default
//   - Future AC packets available through ENABLE_AC_MODE
//
// Existing receiver-validation and R-factor simulations retained.
// ======================================================
 
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
 
#define BUTTON_PIN    0
#define VEXT_PIN      36
#define OLED_SDA_PIN  17
#define OLED_SCL_PIN  18
#define OLED_RST_PIN  21
#define LORA_NSS      8
#define LORA_SCK      9
#define LORA_MOSI     10
#define LORA_MISO     11
#define LORA_RST      12
#define LORA_BUSY     13
#define LORA_DIO1     14
 
// Rev.0 remains DC-only to match the current production Power Node.
// Change to 1 later to send AC packets for future receiver testing.
#define ENABLE_AC_MODE 0
 
static constexpr float RADIO_MHZ = 915.0f;
static constexpr float TEST_R_OHMS = 0.032f;
static constexpr uint32_t WELD_MS = 12000;
static constexpr uint32_t GAP_MS = 15000;
static constexpr uint32_t TX_MS = 1000;
static constexpr uint32_t LONG_PRESS_MS = 900;
 
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(
    U8G2_R0, OLED_SCL_PIN, OLED_SDA_PIN, U8X8_PIN_NONE);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
struct WeldTest {
  const char *name;
  const char *expected;
  float workVolts;
  float amps;
  float distanceMm;
};
 
// BW-10 Rev3, pass A1, 2.4/2.5 mm electrode limits from the supplied workbook:
// 78-117 A, 19-28 V, 92-138 mm/min, 0.96-1.44 kJ/mm.
static const WeldTest TESTS[] = {
  {"NORMAL WELD",     "PASS",       24.0f,  98.0f, 24.0f},
  {"LOW AMPS",        "AMPS LOW",   27.5f,  75.0f, 24.0f},
  {"HIGH AMPS",       "AMPS HIGH",  20.0f, 120.0f, 24.0f},
  {"LOW VOLTS",       "VOLTS LOW",  18.5f, 110.0f, 24.0f},
  {"HIGH VOLTS",      "VOLTS HIGH", 28.5f,  80.0f, 24.0f},
  {"SLOW TRAVEL",     "TRAVEL LOW", 20.0f,  80.0f, 16.0f},
  {"FAST TRAVEL",     "TRAVEL HIGH",24.0f, 110.0f, 30.0f},
  {"HIGH HEAT",       "HEAT HIGH",  28.0f, 117.0f, 24.0f},
  {"LOW HEAT",        "HEAT LOW",   19.0f,  78.0f, 24.0f},
  {"FIELD WELD",      "PASS",       25.0f, 100.0f, 24.0f},
};
static constexpr uint8_t TEST_COUNT = sizeof(TESTS) / sizeof(TESTS[0]);
 
enum RunMode {
  MODE_NORMAL,
  MODE_R_MENU,
  MODE_R_TWO_NODE,
  MODE_R_ONE_MACHINE,
  MODE_R_ONE_WAIT,
  MODE_R_ONE_WORK,
  MODE_R_DONE
};
 
RunMode mode = MODE_NORMAL;
uint8_t testIndex = 0;
uint8_t rMenuChoice = 0;
bool weldActive = true;
uint32_t stateStarted = 0;
uint32_t lastTx = 0;
uint32_t lastDraw = 0;
uint32_t txCount = 0;
 
float lastMachineVolts = 0.0f;
float lastAmps = 0.0f;
char lastNode = 'A';
 
bool rawButton = false;
bool stableButton = false;
uint32_t rawChanged = 0;
uint32_t pressedAt = 0;
 
const char *packetMode() {
#if ENABLE_AC_MODE
  return "AC";
#else
  return "DC";
#endif
}
 
void powerDisplayOn() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(20);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(50);
}
 
void sendPower(char node, float machineVolts, float amps) {
  char packet[48];
  snprintf(packet, sizeof(packet), "P,%c,%.2f,%.1f,%s",
           node, machineVolts, amps, packetMode());
 
  int16_t status = radio.transmit(packet);
  if (status == RADIOLIB_ERR_NONE) {
    txCount++;
  }
 
  lastNode = node;
  lastMachineVolts = machineVolts;
  lastAmps = amps;
}
 
void sendIdle(char node) {
  sendPower(node, 0.50f, 0.1f);
}
 
float varied(float base, float amount) {
  uint8_t phase = (millis() / TX_MS) % 7;
  static const int8_t pattern[7] = {-1, 1, 0, 1, -1, 0, 1};
  return base + pattern[phase] * amount;
}
 
void startNormalTest(uint8_t index) {
  testIndex = index % TEST_COUNT;
  weldActive = true;
  stateStarted = millis();
}
 
void enterNormal() {
  mode = MODE_NORMAL;
  startNormalTest(testIndex);
}
 
void startRMode() {
  stateStarted = millis();
  if (rMenuChoice == 0) mode = MODE_R_TWO_NODE;
  else mode = MODE_R_ONE_MACHINE;
}
 
void shortPress() {
  if (mode == MODE_R_MENU) {
    rMenuChoice ^= 1;
  } else if (mode == MODE_R_ONE_WAIT) {
    mode = MODE_R_ONE_WORK;
    stateStarted = millis();
  } else if (mode == MODE_R_DONE) {
    mode = MODE_R_MENU;
  }
}
 
void longPress() {
  if (mode == MODE_NORMAL) {
    mode = MODE_R_MENU;
  } else if (mode == MODE_R_MENU) {
    startRMode();
  } else {
    enterNormal();
  }
}
 
void pollButton() {
  bool reading = digitalRead(BUTTON_PIN) == LOW;
  uint32_t now = millis();
  if (reading != rawButton) {
    rawButton = reading;
    rawChanged = now;
  }
  if (now - rawChanged < 30 || rawButton == stableButton) return;
  stableButton = rawButton;
  if (stableButton) {
    pressedAt = now;
  } else {
    uint32_t held = now - pressedAt;
    if (held >= LONG_PRESS_MS) longPress();
    else if (held >= 40) shortPress();
  }
}
 
void updateNormal() {
  uint32_t now = millis();
  uint32_t elapsed = now - stateStarted;
  if (weldActive && elapsed >= WELD_MS) {
    weldActive = false;
    stateStarted = now;
  } else if (!weldActive && elapsed >= GAP_MS) {
    startNormalTest(testIndex + 1);
  }
 
  if (now - lastTx < TX_MS) return;
  lastTx = now;
 
  if (!weldActive) {
    sendIdle('A');
    return;
  }
 
  const WeldTest &t = TESTS[testIndex];
  float ampVariation = (testIndex == 9) ? 5.0f : 2.5f;
  float amps = varied(t.amps, ampVariation);
  float workVolts = varied(t.workVolts, 0.25f);
  float machineVolts = workVolts + amps * TEST_R_OHMS;
  sendPower('A', machineVolts, amps);
}
 
void updateRMode() {
  uint32_t now = millis();
  if (mode == MODE_R_TWO_NODE && now - stateStarted >= WELD_MS) {
    mode = MODE_R_DONE;
    stateStarted = now;
  } else if (mode == MODE_R_ONE_MACHINE && now - stateStarted >= WELD_MS) {
    mode = MODE_R_ONE_WAIT;
    stateStarted = now;
  } else if (mode == MODE_R_ONE_WORK && now - stateStarted >= WELD_MS) {
    mode = MODE_R_DONE;
    stateStarted = now;
  }
 
  if (now - lastTx < TX_MS) return;
  lastTx = now;
 
  if (mode == MODE_R_TWO_NODE) {
    float amps = varied(100.0f, 1.0f);
    if ((now % 2000) < 1000) sendPower('A', 27.20f, amps); // machine
    else sendPower('B', 24.00f, amps);                     // work
  } else if (mode == MODE_R_ONE_MACHINE) {
    sendPower('A', varied(27.18f, 0.08f), varied(100.5f, 1.0f));
  } else if (mode == MODE_R_ONE_WORK) {
    sendPower('A', varied(23.98f, 0.08f), varied(99.5f, 1.0f));
  } else {
    sendIdle('A');
  }
}
 
void line(uint8_t y, const char *text) {
  display.drawStr(0, y, text);
}
 
void drawScreen() {
  if (millis() - lastDraw < 150) return;
  lastDraw = millis();
  display.clearBuffer();
  display.setFont(u8g2_font_5x8_tf);
  char text[40];
 
  if (mode == MODE_NORMAL) {
    const WeldTest &t = TESTS[testIndex];
 
    line(8, "NODE A SIMULATOR REV.0");
 
    snprintf(text, sizeof(text), "TEST %u/%u  %s",
             testIndex + 1, TEST_COUNT, weldActive ? "WELD" : "GAP");
    line(18, text);
 
    line(28, t.name);
 
    snprintf(text, sizeof(text), "%c %.1fV %.0fA %s",
             lastNode, lastMachineVolts, lastAmps, packetMode());
    line(38, text);
 
    snprintf(text, sizeof(text), "EXPECT: %s", t.expected);
    line(48, text);
 
    if (weldActive) {
      snprintf(text, sizeof(text), "TX %lu  DIST %.0fmm",
               (unsigned long)txCount, t.distanceMm);
    } else {
      uint32_t elapsed = millis() - stateStarted;
      uint32_t left = (elapsed < GAP_MS) ? (GAP_MS - elapsed + 999) / 1000 : 0;
      snprintf(text, sizeof(text), "TX %lu  NEXT %lus",
               (unsigned long)txCount, (unsigned long)left);
    }
    line(58, text);
    line(64, "HOLD: R-FACTOR TESTS");
  } else if (mode == MODE_R_MENU) {
    line(9, "R-FACTOR SIMULATOR");
    line(23, rMenuChoice == 0 ? "> TWO NODES / ONE PASS" : "  TWO NODES / ONE PASS");
    line(35, rMenuChoice == 1 ? "> ONE NODE / TWO PASSES" : "  ONE NODE / TWO PASSES");
    line(50, "CLICK: CHANGE");
    line(62, "HOLD: START");
  } else {
    line(9, "R-FACTOR  TARGET .032");
    if (mode == MODE_R_TWO_NODE) {
      line(22, "2 NODE / 1 PASS - TX");
      line(34, "A MACHINE 27.20V 100A");
      line(46, "B WORK    24.00V 100A");
    } else if (mode == MODE_R_ONE_MACHINE) {
      line(22, "1 NODE / 2 PASS");
      line(36, "PASS 1: MACHINE - TX");
      line(48, "A 27.18V 100.5A");
    } else if (mode == MODE_R_ONE_WAIT) {
      line(22, "PASS 1 MACHINE DONE");
      line(38, "MOVE TO WORK");
      line(54, "CLICK TO START PASS 2");
    } else if (mode == MODE_R_ONE_WORK) {
      line(22, "1 NODE / 2 PASS");
      line(36, "PASS 2: WORK - TX");
      line(48, "A 23.98V 99.5A");
    } else {
      line(24, "SIMULATION COMPLETE");
      line(40, "EXPECTED R = 0.032");
      line(56, "CLICK: R MENU");
    }
 
    snprintf(text, sizeof(text), "TX %lu  HOLD: NORMAL",
             (unsigned long)txCount);
    line(64, text);
  }
 
  display.sendBuffer();
}
 
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  powerDisplayOn();
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "MASKWA CONNECT");
  display.drawStr(0, 28, "NODE A SIMULATOR");
  display.drawStr(0, 44, "REV.0");
  display.drawStr(0, 60, "Starting LoRa...");
  display.sendBuffer();
 
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int16_t status = radio.begin(RADIO_MHZ);
  if (status != RADIOLIB_ERR_NONE) {
    display.clearBuffer();
    display.drawStr(0, 18, "LORA INIT FAILED");
    char errorText[24];
    snprintf(errorText, sizeof(errorText), "RadioLib error %d", status);
    display.drawStr(0, 36, errorText);
    display.sendBuffer();
    while (true) delay(1000);
  }
 
  display.clearBuffer();
  display.drawStr(0, 18, "LORA OK");
  display.drawStr(0, 34, "915 MHz");
  display.drawStr(0, 50, "Starting simulation");
  display.sendBuffer();
  delay(900);
 
  stateStarted = millis();
}
 
void loop() {
  pollButton();
  if (mode == MODE_NORMAL) updateNormal();
  else if (mode != MODE_R_MENU) updateRMode();
  drawScreen();
  delay(2);
}
