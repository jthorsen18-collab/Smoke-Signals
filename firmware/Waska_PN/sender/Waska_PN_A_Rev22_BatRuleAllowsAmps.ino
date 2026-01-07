// Waska_PN_A_Rev22_BatRuleAllowsAmps.ino
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
float AMP_CAL       = 4.40f;     // Rev20 value

// -------------------------- RMS window ------------------------
const int AMP_N_SAMPLES       = 1200;
const int AMP_SAMPLE_DELAY_US = 60;

// -------------------------- Timing ----------------------------
const unsigned long BOOT_SETTLE_MS   = 2500;
const unsigned long TX_INTERVAL_MS   = 500;
const unsigned long BEAT_INTERVAL_MS = 1800;

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

// Rev22: current threshold used with battery-removed rule
const float BAT_REMOVED_I_MAX_A = 0.30f; // only force gate off if current is also basically zero

// ------------------------ State -------------------------------
float NoiseFloorA = 0.0f;
bool gateOn = false;
unsigned long lastTxMs = 0;
unsigned long lastBeat = 0;

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

uint32_t readMilliVoltsFiltered(int pin) {
  const int N = 25;
  uint32_t v[N];

  for (int i = 0; i < N; i++) {
    v[i] = analogReadMilliVolts(pin);
    delayMicroseconds(800);
  }

  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (v[j] < v[i]) { uint32_t t = v[i]; v[i] = v[j]; v[j] = t; }
    }
  }

  const int trim = 4;
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

float readClampRMS_meas_A() {
  uint32_t acc = 0;
  static uint16_t s[AMP_N_SAMPLES];

  for (int i = 0; i < AMP_N_SAMPLES; i++) {
    uint16_t x = (uint16_t)analogRead(PIN_AMP);
    s[i] = x;
    acc += x;
    delayMicroseconds(AMP_SAMPLE_DELAY_US);
  }
  float mean = acc / (float)AMP_N_SAMPLES;

  double sumSq = 0.0;
  for (int i = 0; i < AMP_N_SAMPLES; i++) {
    float d = s[i] - mean;
    sumSq += (double)d * (double)d;
  }
  float rmsCounts = sqrt(sumSq / (double)AMP_N_SAMPLES);

  const float ADC_REF_V = 3.3f;
  const float ADC_MAX   = 4095.0f;

  float vRms = (rmsCounts * ADC_REF_V) / ADC_MAX;
  float iA = (vRms / CLAMP_V_PER_A) * AMP_CAL;

  if (iA < 0.0f) iA = 0.0f;
  return iA;
}

void drawOLED(float V, float Itotal) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Waska-PN A Rev22");

  char l2[24];
  snprintf(l2, sizeof(l2), "V  %5.2f", V);
  u8g2.drawStr(0, 32, l2);

  char l3[24];
  snprintf(l3, sizeof(l3), "I  %5.2fA", Itotal);
  u8g2.drawStr(0, 54, l3);

  u8g2.sendBuffer();
}

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("BOOT: Waska_PN_A Rev22 BatRuleAllowsAmps");
  Serial.println("Baud=115200");
  Serial.print("Pins: AMP=GPIO"); Serial.print(PIN_AMP);
  Serial.print(" VSENSE=GPIO"); Serial.println(PIN_VSENSE);

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

  Serial.print("Boot settle "); Serial.print(BOOT_SETTLE_MS); Serial.println(" ms...");
  delay(BOOT_SETTLE_MS);

  Serial.println("Measuring clamp noise floor (BNC connected, clamp NOT clamped)...");
  NoiseFloorA = readClampRMS_meas_A();
  Serial.print("NoiseFloorA (A): "); Serial.println(NoiseFloorA, 3);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    loraOK = true;
    Serial.println("LoRa init: OK (code 0)");
  } else {
    loraOK = false;
    Serial.print("LoRa init: FAIL (code "); Serial.print(state); Serial.println(")");
  }

  Serial.print("Cal: DIVIDER_GAIN="); Serial.println(DIVIDER_GAIN, 3);
  Serial.print("Cal: CLAMP_V_PER_A="); Serial.println(CLAMP_V_PER_A, 6);
  Serial.print("Cal: AMP_CAL="); Serial.println(AMP_CAL, 3);

  Serial.print("Rev22: Bat rule only forces Gate OFF if V<");
  Serial.print(V_BAT_PRESENT_MIN, 2);
  Serial.print(" AND Inet<");
  Serial.println(BAT_REMOVED_I_MAX_A, 2);
}

// ---------------------------- Loop ---------------------------
void loop() {
  unsigned long now = millis();

  float V = readV_V();
  float Imeas = readClampRMS_meas_A();

  // Compute Inet first (Rev22 needs this before the battery rule)
  float Inet = Imeas - NoiseFloorA;
  if (Inet < 0.0f) Inet = 0.0f;

  // Rev22 battery removed rule:
  // Only force gate OFF when battery/VSENSE is absent AND current is also basically zero.
  if (V < V_BAT_PRESENT_MIN && Inet <= BAT_REMOVED_I_MAX_A) {
    gateOn = false;
    hitsOn = 0;
    hitsOff = 0;
  }

  // Adaptive noise floor (only when gate OFF and Inet is small)
  if (!gateOn && Inet <= ADAPT_INET_MAX_A) {
    NoiseFloorA = (1.0f - NOISE_ALPHA) * NoiseFloorA + (NOISE_ALPHA) * Imeas;
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

  float Itotal = gateOn ? Inet : 0.0f;

  if (now - lastBeat >= BEAT_INTERVAL_MS) {
    lastBeat = now;
    Serial.print("BEAT ms="); Serial.print(now);
    Serial.print(" NoiseFloorA="); Serial.print(NoiseFloorA, 3);
    Serial.print(" Gate="); Serial.print(gateOn ? "ON" : "OFF");
    Serial.print(" OnHits="); Serial.print(hitsOn);
    Serial.print(" OffHits="); Serial.println(hitsOff);
  }

  Serial.print("V="); Serial.print(V, 2);
  Serial.print("  Itotal="); Serial.print(Itotal, 2);
  Serial.print("  Imeas="); Serial.print(Imeas, 2);
  Serial.print("  Inet="); Serial.println(Inet, 2);

  drawOLED(V, Itotal);

  if (loraOK && (now - lastTxMs >= TX_INTERVAL_MS)) {
    lastTxMs = now;

    float Vditch = 0.0f;
    int   Rindex = 0;

    char packet[64];
    snprintf(packet, sizeof(packet),
             "P,A,RUN,%.2f,%.2f,%.2f,%d",
             V, Itotal, Vditch, Rindex);

    int txState = lora.transmit(packet);
    if (txState == RADIOLIB_ERR_NONE) {
      Serial.print("LoRa TX: OK  "); Serial.println(packet);
    } else {
      Serial.print("LoRa TX: FAIL (code "); Serial.print(txState); Serial.println(")");
    }
  }

  delay(20);
}

