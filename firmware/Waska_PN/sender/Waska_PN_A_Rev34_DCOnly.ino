// Waska_PN_A_Rev34_DCOnly.ino
//
// Rev34 DC Only:
// - DC-only current reporting (no AC component).
// - Preserve current direction (signed amps).
// - Retain battery rule that allows amps with V=0 when current is present.
// - Remove boot-time clamp zeroing; auto-zero when gate is OFF and current is low.
// - Update AMP_CAL for 1 mV/A clamp scaling.
//
// Rev33 AC/DC:
// - Bump revision number only.
//
// Rev32 AC/DC:
// - Report AC and DC separately (no total in OLED/packet).
//
// Rev31 AC/DC:
// - Add DC (mean) and AC (RMS ripple) current reporting.
// - Compute total RMS from DC + AC components.
//
// Rev30:
// - Reduce blocking by sampling sensors and updating OLED on intervals.
// - Smaller voltage filter window to speed loop.
// - Reduced clamp RMS window to speed loop.
// - Add TX jitter to reduce collisions between senders.
// - Throttle Serial output while keeping visibility.
//
// Rev22:
// - Same as Rev20, but FIXES the "amps go to zero when battery/VSENSE is unplugged" behavior.
// - We only force Gate OFF when V is ~0 AND current is also basically ~0.
//   If real current is present, we allow amps to display even with V=0.
//
// Rev20 features retained:
// - Adaptive NoiseFloorA when Gate=OFF
// - 2-hit ON and 2-hit OFF hysteresis
// - Force Gate OFF when V is essentially 0 (battery removed)  <-- now conditional on current
// - AMP_CAL as in Rev20

#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// ---------------------------- Pins ----------------------------
const int PIN_AMP    = 4;
const int PIN_VSENSE = 6;

const int PIN_VEXT      = 36; // Vext control (active LOW)
const int PIN_OLED_SDA  = 17;
const int PIN_OLED_SCL  = 18;
const int PIN_OLED_RST  = 21;

const int LORA_NSS   = 8;
const int LORA_SCK   = 9;
const int LORA_MOSI  = 10;
const int LORA_MISO  = 11;
const int LORA_RST   = 12;
const int LORA_BUSY  = 13;
const int LORA_DIO1  = 14;

// ------------------------- OLED object ------------------------
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0,
  PIN_OLED_SCL,
  PIN_OLED_SDA,
  U8X8_PIN_NONE
);

// ---------------------- LoRa / RadioLib -----------------------
SX1262 lora = SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
bool loraOK = false;

// -------------------------- Calibration -----------------------
float DIVIDER_GAIN  = 115.805f;  // from your Rev19 run
float CLAMP_V_PER_A = 0.001f;    // 1mV/A clamp mode
float AMP_CAL       = 1.00f;     // 1mV/A scaling

// -------------------------- Samples ---------------------------
const int AMP_N_SAMPLES       = 600;
const int AMP_SAMPLE_DELAY_US = 40;

// -------------------------- Timing ----------------------------
const unsigned long BOOT_SETTLE_MS   = 2500;
const unsigned long TX_INTERVAL_MS   = 500;
const unsigned long BEAT_INTERVAL_MS = 1800;
const unsigned long SENSE_INTERVAL_MS = 200;
const unsigned long OLED_INTERVAL_MS  = 200;
const long TX_JITTER_MS = 80;
const unsigned long MIN_TX_GAP_MS = 200;

// Gate thresholds (on NET current)
const float GATE_ON_A  = 0.8f;
const float GATE_OFF_A = 0.4f;

// 2-hit rules
const int HITS_ON_REQUIRED  = 2;
const int HITS_OFF_REQUIRED = 2;
int hitsOn  = 0;
int hitsOff = 0;

// Adaptive noise floor behavior
const float ADAPT_INET_MAX_A = 0.30f;
const float NOISE_ALPHA      = 0.05f;

// Battery-present threshold
const float V_BAT_PRESENT_MIN = 1.0f;  // if V < 1.0V, battery/VSENSE is "not present"

// Rev22-style current threshold used with battery-removed rule
const float BAT_REMOVED_I_MAX_A = 0.30f; // only force gate off if current is also basically zero

// ------------------------ State -------------------------------
float NoiseFloorA = 0.0f;
float ClampZeroCounts = 0.0f;
bool gateOn = false;
unsigned long nextTxMs = 0;
unsigned long lastBeat = 0;
unsigned long lastSenseMs = 0;
unsigned long lastOledMs = 0;
float lastV = 0.0f;
float lastIdc = 0.0f;
float lastInet = 0.0f;

// ------------------------ Helpers ----------------------------
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

HardwareSerial &LOGSERIAL = Serial;

void initSerial() {
  LOGSERIAL.begin(115200);
  delay(200);
  LOGSERIAL.println();
}

uint32_t readMilliVoltsFiltered(int pin) {
  const int N = 9;
  uint32_t v[N];

  for (int i = 0; i < N; i++) {
    v[i] = analogReadMilliVolts(pin);
    delayMicroseconds(400);
  }

  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (v[j] < v[i]) { uint32_t t = v[i]; v[i] = v[j]; v[j] = t; }
    }
  }

  const int trim = 1;
  uint64_t sum = 0;
  int count = 0;
  for (int i = trim; i < N - trim; i++) { sum += v[i]; count++; }
  return (uint32_t)(sum / (uint64_t)count);
}

float readV_V() {
  uint32_t mv = readMilliVoltsFiltered(PIN_VSENSE);
  float vSense = mv / 1000.0f;
  return vSense * DIVIDER_GAIN;
}

float readClampMeanCounts() {
  uint32_t acc = 0;
  for (int i = 0; i < AMP_N_SAMPLES; i++) {
    uint16_t x = (uint16_t)analogRead(PIN_AMP);
    acc += x;
    delayMicroseconds(AMP_SAMPLE_DELAY_US);
  }
  return acc / (float)AMP_N_SAMPLES;
}

float countsToAmpsSigned(float counts) {
  const float ADC_REF_V = 3.3f;
  const float ADC_MAX   = 4095.0f;
  float v = (counts * ADC_REF_V) / ADC_MAX;
  float iA = (v / CLAMP_V_PER_A) * AMP_CAL;
  return iA;
}

void drawOLED(float V, float Idc) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Waska-PN A Rev34 DC");

  char l2[24];
  snprintf(l2, sizeof(l2), "V  %5.2f", V);
  u8g2.drawStr(0, 28, l2);

  char l3[24];
  snprintf(l3, sizeof(l3), "DC %5.2fA", Idc);
  u8g2.drawStr(0, 44, l3);

  u8g2.sendBuffer();
}

// --------------------------- Setup ---------------------------
void setup() {
  initSerial();
  LOGSERIAL.println("BOOT: Waska_PN_A Rev34 DC Only");
  LOGSERIAL.println("Baud=115200");
  LOGSERIAL.print("Pins: AMP=GPIO"); LOGSERIAL.print(PIN_AMP);
  LOGSERIAL.print(" VSENSE=GPIO"); LOGSERIAL.println(PIN_VSENSE);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_AMP, ADC_11db);
  analogSetPinAttenuation(PIN_VSENSE, ADC_11db);

  pinMode(PIN_AMP, INPUT);
  pinMode(PIN_VSENSE, INPUT);

  VextON();
  delay(50);
  oledHardResetPulse();
  u8g2.begin();
  drawOLED(0.0f, 0.0f);

  LOGSERIAL.print("Boot settle "); LOGSERIAL.print(BOOT_SETTLE_MS); LOGSERIAL.println(" ms...");
  delay(BOOT_SETTLE_MS);

  LOGSERIAL.println("Clamp auto-zero: enabled when gate OFF and current is low.");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    loraOK = true;
    LOGSERIAL.println("LoRa init: OK (code 0)");
  } else {
    loraOK = false;
    LOGSERIAL.print("LoRa init: FAIL (code "); LOGSERIAL.print(state); LOGSERIAL.println(")");
  }

  LOGSERIAL.print("Cal: DIVIDER_GAIN="); LOGSERIAL.println(DIVIDER_GAIN, 3);
  LOGSERIAL.print("Cal: CLAMP_V_PER_A="); LOGSERIAL.println(CLAMP_V_PER_A, 6);
  LOGSERIAL.print("Cal: AMP_CAL="); LOGSERIAL.println(AMP_CAL, 3);

  LOGSERIAL.print("Rev34: Bat rule forces Gate OFF if V<");
  LOGSERIAL.print(V_BAT_PRESENT_MIN, 2);
  LOGSERIAL.print(" AND Inet<");
  LOGSERIAL.println(BAT_REMOVED_I_MAX_A, 2);

  randomSeed((uint32_t)analogRead(PIN_AMP) ^ millis());
  nextTxMs = millis() + TX_INTERVAL_MS;
}

// ---------------------------- Loop ---------------------------
void loop() {
  unsigned long now = millis();

  if (now - lastSenseMs >= SENSE_INTERVAL_MS) {
    lastSenseMs = now;

    float V = readV_V();
    float clampMeanCounts = readClampMeanCounts();
    float IdcSigned = countsToAmpsSigned(clampMeanCounts - ClampZeroCounts);
    float IdcAbs = fabs(IdcSigned);

    float Inet = IdcAbs - NoiseFloorA;
    if (Inet < 0.0f) Inet = 0.0f;

    // Rev22 battery removed rule:
    // Only force gate OFF when battery/VSENSE is absent AND current is also basically zero.
    if (V < V_BAT_PRESENT_MIN && Inet <= BAT_REMOVED_I_MAX_A) {
      gateOn = false;
      hitsOn = 0;
      hitsOff = 0;
    }

    // Adaptive clamp zero + noise floor (only when gate OFF and current is small)
    if (!gateOn && IdcAbs <= ADAPT_INET_MAX_A) {
      ClampZeroCounts = (1.0f - NOISE_ALPHA) * ClampZeroCounts + (NOISE_ALPHA) * clampMeanCounts;
      NoiseFloorA = (1.0f - NOISE_ALPHA) * NoiseFloorA + (NOISE_ALPHA) * IdcAbs;
    }

    // 2-hit ON / 2-hit OFF with hysteresis thresholds
    if (!gateOn) {
      hitsOff = 0;
      if (Inet >= GATE_ON_A) {
        hitsOn++;
        if (hitsOn >= HITS_ON_REQUIRED) {
          gateOn = true;
          hitsOn = 0;
        }
      } else {
        hitsOn = 0;
      }
    } else {
      hitsOn = 0;
      if (Inet <= GATE_OFF_A) {
        hitsOff++;
        if (hitsOff >= HITS_OFF_REQUIRED) {
          gateOn = false;
          hitsOff = 0;
        }
      } else {
        hitsOff = 0;
      }
    }

    float Iout = gateOn ? IdcSigned : 0.0f;

    lastV = V;
    lastIdc = Iout;
    lastInet = Inet;
  }

  if (now - lastBeat >= BEAT_INTERVAL_MS) {
    lastBeat = now;
    LOGSERIAL.print("BEAT ms="); LOGSERIAL.print(now);
    LOGSERIAL.print(" V="); LOGSERIAL.print(lastV, 2);
    LOGSERIAL.print(" Idc="); LOGSERIAL.print(lastIdc, 2);
    LOGSERIAL.print(" Inet="); LOGSERIAL.print(lastInet, 2);
    LOGSERIAL.print(" NoiseFloorA="); LOGSERIAL.print(NoiseFloorA, 3);
    LOGSERIAL.print(" ZeroCounts="); LOGSERIAL.print(ClampZeroCounts, 1);
    LOGSERIAL.print(" Gate="); LOGSERIAL.print(gateOn ? "ON" : "OFF");
    LOGSERIAL.print(" OnHits="); LOGSERIAL.print(hitsOn);
    LOGSERIAL.print(" OffHits="); LOGSERIAL.println(hitsOff);
  }

  if (now - lastOledMs >= OLED_INTERVAL_MS) {
    lastOledMs = now;
    drawOLED(lastV, lastIdc);
  }

  if (loraOK && (now >= nextTxMs)) {
    long jitter = random(-TX_JITTER_MS, TX_JITTER_MS + 1);
    long nextDelay = (long)TX_INTERVAL_MS + jitter;
    if (nextDelay < (long)MIN_TX_GAP_MS) nextDelay = (long)MIN_TX_GAP_MS;
    nextTxMs = now + (unsigned long)nextDelay;

    char packet[64];
    snprintf(packet, sizeof(packet),
             "P,A,RUN,%.2f,%.2f",
             lastV, lastIdc);

    int txState = lora.transmit(packet);
    if (txState == RADIOLIB_ERR_NONE) {
      LOGSERIAL.print("LoRa TX: OK  "); LOGSERIAL.println(packet);
    } else {
      LOGSERIAL.print("LoRa TX: FAIL (code "); LOGSERIAL.print(txState); LOGSERIAL.println(")");
    }
  }

  delay(20);
}
