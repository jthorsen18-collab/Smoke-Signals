// Waska_PN_A_Rev50_FieldCalShift.ino
// DC-only field test: OCV-to-weld voltage refresh, serial debug ON, low values visible, manual zero disabled.

/*
  Waska_PN_A_Rev50_FieldCalShift

  Rev50:
  - Based on Rev47 DC-only OCV refresh sender.
  - OLED top line shows the exact revision: A NODE Rev50.
  - Manual field zero remains disabled for troubleshooting. Boot zero only.
  - OLED updates locally at 4 Hz.
  - LoRa sends one simple packet per second in Node A first half.
  - ADS and LoRa status kept on OLED.
  - Serial debug prints are ON for field troubleshooting.
  - Low voltage/amp display hiding removed: OLED and packet show calculated values.
  - Auto-zero-during-no-load disabled so residual drift can be observed.
  - Field calibration shift: VOLT_CAL = 1.084f based on Rev49 reading high at 61.6V vs target 59.8V.
  - Amp calibration bumped from 0.967f to 1.020f based on field comparisons showing amps low by about 5-6%.
  - Adds voltage ADS refresh when high OCV appears to be stuck during welding.
  - If amps are above weld threshold and volts remain in OCV range, the voltage ADS is reinitialized and fresh voltage samples are taken.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

const char REV_NUMBER[] = "Rev50";

#define DEBUG_SERIAL 1
const char NODE_ID[] = "A";

// OLED
const int PIN_VEXT      = 36;
const int PIN_OLED_SDA  = 17;
const int PIN_OLED_SCL  = 18;
const int PIN_OLED_RST  = 21;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  PIN_OLED_SCL,
  PIN_OLED_SDA,
  U8X8_PIN_NONE
);

// LoRa
const int LORA_NSS   = 8;
const int LORA_SCK   = 9;
const int LORA_MOSI  = 10;
const int LORA_MISO  = 11;
const int LORA_RST   = 12;
const int LORA_BUSY  = 13;
const int LORA_DIO1  = 14;

// Button
const int BUTTON_PIN = 0;

SX1262 lora = SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
bool loraOK = false;

volatile bool rxFlag = false;
volatile bool rxEnable = true;

// ADS1115 I2C
const int I2C_SDA_PIN = 41;
const int I2C_SCL_PIN = 42;

const uint8_t ADS_VOLT_ADDR = 0x48;
const uint8_t ADS_AMP_ADDR  = 0x49;

Adafruit_ADS1115 adsVolt;
Adafruit_ADS1115 adsAmp;

bool adsVoltReady = false;
bool adsAmpReady  = false;

// ADS1115 GAIN_ONE = +/-4.096 V, 0.125 mV/count
const float ADS_VOLTS_PER_COUNT = 0.000125f;

// Sampling
const uint16_t SAMPLE_COUNT = 24;
const uint16_t SAMPLE_SPACING_US = 500;

// Timing
const uint32_t SENSE_INTERVAL_MS = 125;
const uint32_t OLED_INTERVAL_MS = 250;
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;

// Amp calibration
const float YC500_ADS_TO_RAW_SCALE = 1.1f;
const float YC500_AMPS_PER_RAW_VOLT = 500.0f;
const float AMP_CAL = 1.020f;

// Voltage calibration
const float AMC3330_SCALE = 72.0f;
const float VOLT_CAL = 1.084f;
const float VOLT_POLARITY = 1.0f;

// Detection / field refresh thresholds
const float WELD_I_THRESHOLD = 15.0f;
const float OCV_STALE_VOLTS_MIN = 45.0f;

// Deadband constants retained for reference; Rev50 does not hide low values.
const float VOLTS_NOISE_FLOOR = 0.25f;
const float AMPS_NOISE_FLOOR = 0.3f;

const float VOLTS_PRESENT_MIN = 2.0f;
const float AMPS_PRESENT_MIN  = 1.5f;

const float AC_DETECT_VOLTS_MIN = 2.0f;
const float AC_DETECT_AMPS_MIN = 1.5f;
const float AC_TO_DC_RATIO = 0.35f;

// Legacy AC/DC constants retained but not used in this DC-only field version
const float VOLT_ZERO_CROSS_MIN = 0.75f;
const float VOLT_AC_MIN_RATIO = 0.25f;
const float DC_SMOOTH_ALPHA = 0.25f;

// Auto-zero
float ampZeroAdsVolts = 0.0f;
float voltZeroDiffVolts = 0.0f;
const float AMP_ZERO_ALPHA = 0.02f;

float smoothedDcVolts = 0.0f;
bool smoothedDcValid = false;

// Timed slot for Node A
const uint32_t SLOT_EPOCH_MS = 1000;
// Node A owns the first half of each second.
// This is a wide zone, not a narrow slot, so the sender does not miss TX just because the loop is busy.
const uint32_t NODE_A_ZONE_START_MS = 100;
const uint32_t NODE_A_ZONE_END_MS   = 500;

uint32_t lastTxEpochIndex = UINT32_MAX;

// Receiver can broadcast SYNC,<millis>.
// Sender uses that only to align its first-half TX zone.
bool syncValid = false;
int32_t syncOffsetMs = 0;  // local millis - receiver millis
uint32_t lastSyncRxMs = 0;

struct SignalStats {
  float mean;
  float rmsAc;
};

struct Measurement {
  float dcVolts;
  float acVolts;
  float dcAmps;
  float acAmps;

  float sendVolts;
  float sendAmps;

  bool voltagePresent;
  bool ampsPresent;

  bool voltsAreAc;
  bool ampsAreAc;

  bool noLoad;
  bool valid;
};

Measurement latest = {
  0.0f, 0.0f,
  0.0f, 0.0f,
  0.0f, 0.0f,
  false,
  false,
  false,
  false,
  true,
  false
};

static void setRxFlag(void) {
  if (!rxEnable) return;
  rxFlag = true;
}

bool selectedDcMode = true;
bool lastSendOK = false;
uint32_t voltageAdcRefreshCount = 0;
uint32_t lastVoltageAdcRefreshMs = 0;

// One-button control. Long press re-zeros voltage and current baselines.
bool btnLastRaw = HIGH;
bool btnPressed = false;
uint32_t btnDownMs = 0;
const uint32_t BUTTON_DEBOUNCE_MS = 25;
// Manual zero is intentionally disabled in Rev50. Boot zero only.

uint32_t lastSenseMs = 0;
uint32_t lastOledMs = 0;
uint32_t lastHeartbeatMs = 0;

void VextON() {
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
}

void oledHardResetPulse() {
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW);
  delay(20);
  digitalWrite(PIN_OLED_RST, HIGH);
  delay(20);
}

float applyDeadband(float value, float floorValue) {
  if (fabs(value) < floorValue) return 0.0f;
  return value;
}

float adsCountsToVolts(int16_t counts) {
  return (float)counts * ADS_VOLTS_PER_COUNT;
}

float readAmpAdsA0() {
  return adsCountsToVolts(adsAmp.readADC_SingleEnded(0));
}

float readVoltDiffA0A1() {
  return adsCountsToVolts(adsVolt.readADC_Differential_0_1());
}

float ampAdsVoltsToAmps(float adsAmpVolts) {
  return (adsAmpVolts - ampZeroAdsVolts) *
         YC500_ADS_TO_RAW_SCALE *
         YC500_AMPS_PER_RAW_VOLT *
         AMP_CAL;
}

float voltDiffToPrimaryVolts(float diffVolts) {
  float correctedDiff = diffVolts - voltZeroDiffVolts;
  return VOLT_POLARITY * correctedDiff * AMC3330_SCALE * VOLT_CAL;
}

SignalStats calculateStats(float samples[], uint16_t count) {
  SignalStats stats;
  stats.mean = 0.0f;
  stats.rmsAc = 0.0f;

  if (count == 0) return stats;

  double sum = 0.0;
  for (uint16_t i = 0; i < count; i++) {
    sum += samples[i];
  }

  stats.mean = (float)(sum / count);

  double sumSquares = 0.0;
  for (uint16_t i = 0; i < count; i++) {
    double centered = (double)samples[i] - stats.mean;
    sumSquares += centered * centered;
  }

  stats.rmsAc = (float)sqrt(sumSquares / count);
  return stats;
}

float calculateRmsFromZero(float samples[], uint16_t count) {
  if (count == 0) return 0.0f;

  double sumSquares = 0.0;
  for (uint16_t i = 0; i < count; i++) {
    sumSquares += (double)samples[i] * (double)samples[i];
  }

  return (float)sqrt(sumSquares / count);
}

void scanI2CBus() {
  Serial.println("Scanning I2C bus...");

  byte count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) Serial.println("Found no I2C devices");

  Serial.println("I2C scan complete.");
  Serial.println();
}

float averageAmpZero(int samples) {
  double sum = 0.0;

  for (int i = 0; i < samples; i++) {
    sum += readAmpAdsA0();
    delay(10);
  }

  return sum / samples;
}

float averageVoltZero(int samples) {
  double sum = 0.0;

  for (int i = 0; i < samples; i++) {
    sum += readVoltDiffA0A1();
    delay(10);
  }

  return sum / samples;
}

void captureZeroBaselines() {
  Serial.println("Capturing zero baselines.");
  Serial.println("Keep clamp empty and no voltage on AMC input.");

  ampZeroAdsVolts = averageAmpZero(100);
  voltZeroDiffVolts = averageVoltZero(100);

  smoothedDcVolts = 0.0f;
  smoothedDcValid = false;

  Serial.print("Amp zero ADS A0 = ");
  Serial.print(ampZeroAdsVolts, 6);
  Serial.println(" V");

  Serial.print("Voltage zero diff A0-A1 = ");
  Serial.print(voltZeroDiffVolts, 6);
  Serial.println(" V");

  Serial.println();
}

uint32_t networkMillis() {
  if (!syncValid) return millis();
  return (uint32_t)((int32_t)millis() - syncOffsetMs);
}

uint32_t currentEpochIndex() {
  return networkMillis() / SLOT_EPOCH_MS;
}

uint32_t currentEpochOffsetMs() {
  return networkMillis() % SLOT_EPOCH_MS;
}

bool transmitGateOpen() {
  uint32_t offset = currentEpochOffsetMs();
  return offset >= NODE_A_ZONE_START_MS && offset < NODE_A_ZONE_END_MS;
}

void setupAds() {
  adsVoltReady = adsVolt.begin(ADS_VOLT_ADDR, &Wire);
  adsAmpReady  = adsAmp.begin(ADS_AMP_ADDR, &Wire);

  if (adsVoltReady) {
    adsVolt.setGain(GAIN_ONE);
    adsVolt.setDataRate(RATE_ADS1115_860SPS);
  }

  if (adsAmpReady) {
    adsAmp.setGain(GAIN_ONE);
    adsAmp.setDataRate(RATE_ADS1115_860SPS);
  }
}

bool refreshVoltageAds(const char* reason) {
#if DEBUG_SERIAL
  Serial.print("Voltage ADS refresh: ");
  Serial.println(reason);
#endif

  adsVoltReady = adsVolt.begin(ADS_VOLT_ADDR, &Wire);
  if (adsVoltReady) {
    adsVolt.setGain(GAIN_ONE);
    adsVolt.setDataRate(RATE_ADS1115_860SPS);
    voltageAdcRefreshCount++;
    lastVoltageAdcRefreshMs = millis();
  }
  return adsVoltReady;
}

float readFreshDcVoltageOnly() {
  if (!adsVoltReady) return NAN;

  float voltSamples[SAMPLE_COUNT];
  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float voltDiff = readVoltDiffA0A1();
    voltSamples[i] = voltDiffToPrimaryVolts(voltDiff);
    delayMicroseconds(SAMPLE_SPACING_US);
  }

  SignalStats voltStats = calculateStats(voltSamples, SAMPLE_COUNT);
  return voltStats.mean;
}

Measurement readMeasurement() {
  Measurement m;

  m.dcVolts = 0.0f;
  m.acVolts = 0.0f;
  m.dcAmps = 0.0f;
  m.acAmps = 0.0f;
  m.sendVolts = 0.0f;
  m.sendAmps = 0.0f;
  m.voltagePresent = false;
  m.ampsPresent = false;
  m.voltsAreAc = false;
  m.ampsAreAc = false;
  m.noLoad = true;
  m.valid = adsAmpReady && adsVoltReady;

  if (!m.valid) return m;

  float voltSamples[SAMPLE_COUNT];
  float ampSamples[SAMPLE_COUNT];

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float ampAds = readAmpAdsA0();
    float voltDiff = readVoltDiffA0A1();

    ampSamples[i] = ampAdsVoltsToAmps(ampAds);
    voltSamples[i] = voltDiffToPrimaryVolts(voltDiff);

    delayMicroseconds(SAMPLE_SPACING_US);
  }

  SignalStats voltStats = calculateStats(voltSamples, SAMPLE_COUNT);
  SignalStats ampStats = calculateStats(ampSamples, SAMPLE_COUNT);
  float voltRmsFromZero = calculateRmsFromZero(voltSamples, SAMPLE_COUNT);

  // Rev50 debug: do not hide small values. Show calculated values after boot zero baseline.
  m.dcVolts = voltStats.mean;
  m.acVolts = voltRmsFromZero;
  m.dcAmps = ampStats.mean;
  m.acAmps = ampStats.rmsAc;

  // DC-only field test. Use DC mean values only.
  m.acVolts = 0.0f;
  m.acAmps = 0.0f;
  m.sendVolts = fabs(m.dcVolts);
  m.sendAmps = fabs(m.dcAmps);
  m.voltsAreAc = false;
  m.ampsAreAc = false;

  bool weldCurrentNow = m.sendAmps >= WELD_I_THRESHOLD;
  bool previousWasNoLoadOcv = (latest.sendAmps < WELD_I_THRESHOLD && latest.sendVolts >= OCV_STALE_VOLTS_MIN);
  bool highVoltageDuringWeld = (weldCurrentNow && m.sendVolts >= OCV_STALE_VOLTS_MIN);

  // Rev50: if the sender appears to carry an OCV value into a weld, refresh the voltage ADS
  // and immediately take fresh voltage samples. This is meant to catch the field condition
  // where the sender OLED stayed stuck near OCV after welding started.
  if (previousWasNoLoadOcv && weldCurrentNow) {
    if (refreshVoltageAds("OCV_TO_WELD")) {
      delay(20);
      float freshV = readFreshDcVoltageOnly();
      if (!isnan(freshV)) {
        m.dcVolts = freshV;
        m.sendVolts = fabs(freshV);
      }
    }
  } else if (highVoltageDuringWeld) {
    if (refreshVoltageAds("HIGH_V_DURING_WELD")) {
      delay(20);
      float freshV = readFreshDcVoltageOnly();
      if (!isnan(freshV)) {
        m.dcVolts = freshV;
        m.sendVolts = fabs(freshV);
      }
    }
  }

  m.voltagePresent = m.sendVolts >= VOLTS_PRESENT_MIN;
  m.ampsPresent = m.sendAmps >= AMPS_PRESENT_MIN;
  m.noLoad = !m.voltagePresent && !m.ampsPresent;

  // Rev50 debug: no background auto-zero. Manual zero is disabled; boot zero only.

  return m;
}

void maybeSense() {
  uint32_t now = millis();

  if (now - lastSenseMs < SENSE_INTERVAL_MS) return;

  lastSenseMs = now;
  latest = readMeasurement();
}

void drawOLED() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  char line[32];
  snprintf(line, sizeof(line), "A NODE %s", REV_NUMBER);
  u8g2.drawStr(0, 10, line);

  snprintf(line, sizeof(line), "%.1fV", latest.sendVolts);
  u8g2.drawStr(0, 24, line);

  snprintf(line, sizeof(line), "%.1fA DC", latest.sendAmps);
  u8g2.drawStr(0, 38, line);

  snprintf(line, sizeof(line), "ADS V:%s A:%s",
           adsVoltReady ? "OK" : "NO",
           adsAmpReady ? "OK" : "NO");
  u8g2.drawStr(0, 52, line);

  snprintf(line, sizeof(line), "LoRa:%s VR:%lu", loraOK ? "OK" : "NO", (unsigned long)voltageAdcRefreshCount);
  u8g2.drawStr(0, 64, line);

  u8g2.sendBuffer();
}

void maybeOLED() {
  uint32_t now = millis();

  if (now - lastOledMs < OLED_INTERVAL_MS) return;

  lastOledMs = now;
  drawOLED();
}

String finalModeText(const Measurement &m) {
  return "DC";
}

String simplePowerPacket(const Measurement &m) {
  // New Rev42 packet:
  // P,<node>,<rawMachineVolts>,<amps>,<mode>
  // Example: P,A,27.10,64.20,DC
  String packet = "P,";
  packet += NODE_ID;
  packet += ",";
  packet += String(m.sendVolts, 2);
  packet += ",";
  packet += String(m.sendAmps, 1);
  packet += ",";
  packet += finalModeText(m);
  return packet;
}

bool parseSyncPacket(const String& packet, uint32_t& receiverMs) {
  if (!packet.startsWith("SYNC,")) return false;
  String msStr = packet.substring(5);
  msStr.trim();
  receiverMs = (uint32_t)msStr.toInt();
  return true;
}

void handleSyncRx() {
  if (!loraOK || !rxFlag) return;

  rxEnable = false;
  rxFlag = false;

  String pkt;
  int st = lora.readData(pkt);

  rxEnable = true;

  if (st == RADIOLIB_ERR_NONE) {
    uint32_t rxMs = 0;
    if (parseSyncPacket(pkt, rxMs)) {
      syncOffsetMs = (int32_t)millis() - (int32_t)rxMs;
      syncValid = true;
      lastSyncRxMs = millis();
#if DEBUG_SERIAL
      Serial.print("SYNC RX offset=");
      Serial.println(syncOffsetMs);
#endif
    }
  }

  lora.startReceive();
}

void maybeTransmit() {
  if (!loraOK) return;
  if (!latest.valid) return;
  if (!transmitGateOpen()) return;

  uint32_t epoch = currentEpochIndex();
  if (lastTxEpochIndex == epoch) return;

  String packet = simplePowerPacket(latest);
  rxEnable = false;
  int txState = lora.transmit(packet);
  rxEnable = true;
  lastSendOK = (txState == RADIOLIB_ERR_NONE);
  lora.startReceive();

#if DEBUG_SERIAL
  Serial.print("TX ");
  Serial.print(packet);
  Serial.print(" state=");
  Serial.println(txState);
#endif

  lastTxEpochIndex = epoch;
}

void printBootDebug() {
  Serial.println();
  Serial.println("Maskwa Connect Power Sender A");
  Serial.println("Rev50 AMC3330 Heltec ADS DC Only OCV Refresh FieldCalShift");

  Serial.print("ADS voltage 0x48: ");
  Serial.println(adsVoltReady ? "OK" : "MISSING");

  Serial.print("ADS amps 0x49: ");
  Serial.println(adsAmpReady ? "OK" : "MISSING");

  Serial.print("ADS I2C SDA: GPIO");
  Serial.println(I2C_SDA_PIN);

  Serial.print("ADS I2C SCL: GPIO");
  Serial.println(I2C_SCL_PIN);

  Serial.print("LoRa init: ");
  Serial.println(loraOK ? "OK" : "FAIL");

  Serial.println("Voltage read: AMC3330EVM OUTP/OUTN to ADS 0x48 differential A0-A1");
  Serial.println("Amp read: ADS 0x49 single-ended A0 with boot zero baseline");
  Serial.println("Voltage/current mode Rev50: DC only, OCV refresh, amp calibration bump");
  Serial.println("Packet:");
  Serial.println("P,<node>,<volts>,<amps>,<mode>");
  Serial.println("Mode = DC only");
  Serial.println("TX zone: Node A first half, 100-500ms each second");

  Serial.print("AMC3330 scale = ");
  Serial.println(AMC3330_SCALE, 3);

  Serial.print("VOLT_CAL = ");
  Serial.println(VOLT_CAL, 3);

  Serial.print("AMP_CAL = ");
  Serial.println(AMP_CAL, 3);

  Serial.println();
}

void maybeHeartbeat() {
#if DEBUG_SERIAL
  uint32_t now = millis();

  if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;

  lastHeartbeatMs = now;

  float liveVoltDiff = 0.0f;
  float correctedVoltDiff = 0.0f;
  float rawCalculatedVolts = 0.0f;

  if (adsVoltReady) {
    liveVoltDiff = readVoltDiffA0A1();
    correctedVoltDiff = liveVoltDiff - voltZeroDiffVolts;
    rawCalculatedVolts = voltDiffToPrimaryVolts(liveVoltDiff);
  }

  Serial.print("HB Rev50 ");

  Serial.print("Vpresent=");
  Serial.print(latest.voltagePresent ? "YES" : "NO");

  Serial.print(" Apresent=");
  Serial.print(latest.ampsPresent ? "YES" : "NO");

  Serial.print(" Vmode=");
  if (!latest.voltagePresent) {
    Serial.print("NONE");
  } else {
    Serial.print(latest.voltsAreAc ? "AC" : "DC");
  }

  Serial.print(" Amode=");
  if (!latest.ampsPresent) {
    Serial.print("NONE");
  } else {
    Serial.print(latest.ampsAreAc ? "AC" : "DC");
  }

  Serial.print(" sendVolts=");
  Serial.print(latest.sendVolts, 2);

  Serial.print(" sendAmps=");
  Serial.print(latest.sendAmps, 2);

  Serial.print(" dcVolts=");
  Serial.print(latest.dcVolts, 2);

  Serial.print(" acVolts=");
  Serial.print(latest.acVolts, 2);

  Serial.print(" dcAmps=");
  Serial.print(latest.dcAmps, 2);

  Serial.print(" acAmps=");
  Serial.print(latest.acAmps, 2);

  Serial.print(" ampZero=");
  Serial.print(ampZeroAdsVolts, 4);

  Serial.print(" voltZeroDiff=");
  Serial.print(voltZeroDiffVolts, 6);

  Serial.print(" liveVoltDiff=");
  Serial.print(liveVoltDiff, 6);

  Serial.print(" correctedVoltDiff=");
  Serial.print(correctedVoltDiff, 6);

  Serial.print(" rawCalcV=");
  Serial.print(rawCalculatedVolts, 2);

  Serial.print(" window=");
  Serial.print(transmitGateOpen() ? "OPEN" : "CLOSED");

  Serial.print(" epoch=");
  Serial.print(currentEpochIndex());

  Serial.print(" offset_ms=");
  Serial.print(currentEpochOffsetMs());

  Serial.print(" ADS48=");
  Serial.print(adsVoltReady ? "OK" : "MISSING");

  Serial.print(" ADS49=");
  Serial.print(adsAmpReady ? "OK" : "MISSING");

  Serial.print(" VadcRefresh=");
  Serial.print(voltageAdcRefreshCount);

  Serial.print(" lastVadcRefreshMs=");
  Serial.print(lastVoltageAdcRefreshMs);

  Serial.print(" LoRa=");
  Serial.println(loraOK ? "OK" : "FAIL");
#endif
}

void handleButton() {
  // Rev50: button zero is disabled for field troubleshooting.
  // Zero baseline is captured only at boot so field idle values stay visible.
  bool raw = digitalRead(BUTTON_PIN);

  if (raw != btnLastRaw) {
    delay(BUTTON_DEBOUNCE_MS);
    raw = digitalRead(BUTTON_PIN);
    btnLastRaw = raw;
  }

  if (!btnPressed && raw == LOW) {
    btnPressed = true;
    btnDownMs = millis();
  }

  if (btnPressed && raw == HIGH) {
    btnPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("BOOT: Waska PN A Rev50 AMC3330 Heltec ADS DC Only OCV Refresh FieldCalShift");

  VextON();
  delay(50);

  oledHardResetPulse();
  u8g2.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  drawOLED();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(100);

  scanI2CBus();
  setupAds();

  if (adsVoltReady && adsAmpReady) {
    captureZeroBaselines();
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = lora.begin(915.0);
  loraOK = (state == RADIOLIB_ERR_NONE);
  if (loraOK) {
    lora.setDio1Action(setRxFlag);
    rxFlag = false;
    rxEnable = true;
    lora.startReceive();
  }

  printBootDebug();

  lastSenseMs = millis() - SENSE_INTERVAL_MS;
  lastOledMs = millis() - OLED_INTERVAL_MS;
  lastHeartbeatMs = millis() - HEARTBEAT_INTERVAL_MS;
}

void loop() {
  handleSyncRx();
  handleButton();
  maybeSense();
  maybeTransmit();
  maybeOLED();
  maybeHeartbeat();
}


