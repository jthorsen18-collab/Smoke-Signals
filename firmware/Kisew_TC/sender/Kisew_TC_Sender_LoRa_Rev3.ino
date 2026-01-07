// Kisew_TC_Sender_LoRa_Rev3.ino
//
// Changes vs Rev2:
// - TX every 250ms (4 Hz) instead of every 2s
// - No delay(500) blocking the loop
// - Adds node ID + sequence number to confirm you're receiving THIS sender
//   Packet: "KIS_TEMP:T01:<seq>:<tempC>"

#include <SPI.h>
#include <RadioLib.h>

#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

bool showFahrenheit = false;
bool lastButtonState = HIGH;

static const char* NODE_ID = "T01";

unsigned long lastReadMs = 0;
const unsigned long READ_INTERVAL_MS = 250;    // sensor read + OLED update cadence

unsigned long lastTxMs = 0;
const unsigned long TX_INTERVAL_MS   = 250;    // TX cadence (match read)

uint32_t seq = 0;

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void OLED_resetRelease() {
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Kisew-TC LoRa sender Rev3 starting...");

  VextON();
  delay(50);
  OLED_resetRelease();
  delay(50);

  u8g2.begin();

  sensors.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(915.0);

  if (state == RADIOLIB_ERR_NONE) {
    loraOK = true;
    Serial.println("LoRa init OK (SX1262 @ 915 MHz).");
  } else {
    loraOK = false;
    Serial.print("LoRa init failed, code ");
    Serial.println(state);
  }
}

void loop() {
  // Button edge toggle
  bool currentBtn = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentBtn == LOW) {
    showFahrenheit = !showFahrenheit;
  }
  lastButtonState = currentBtn;

  const unsigned long now = millis();
  static float tempC = NAN;

  // Periodic sensor read + OLED update
  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;

    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);

    float displayValue = tempC;
    char unitChar = 'C';
    if (showFahrenheit) {
      displayValue = tempC * 9.0f / 5.0f + 32.0f;
      unitChar = 'F';
    }

    Serial.print("Temp C: ");
    Serial.println(tempC, 2);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 10, "Kisew-TC");

    u8g2.setFont(u8g2_font_logisoso32_tf);
    char buf[20];
    snprintf(buf, sizeof(buf), "%.1f%c", displayValue, unitChar);
    u8g2.drawStr(0, 48, buf);

    u8g2.setFont(u8g2_font_6x12_tf);
    char idbuf[24];
    snprintf(idbuf, sizeof(idbuf), "ID:%s  #%lu", NODE_ID, (unsigned long)seq);
    u8g2.drawStr(0, 62, idbuf);

    u8g2.sendBuffer();
  }

  // Periodic TX (always Celsius)
  if (loraOK && !isnan(tempC) && (now - lastTxMs >= TX_INTERVAL_MS)) {
    lastTxMs = now;
    seq++;

    char packet[48];
    snprintf(packet, sizeof(packet), "KIS_TEMP:%s:%lu:%.2f", NODE_ID, (unsigned long)seq, tempC);

    int txState = lora.transmit(packet);
    if (txState == RADIOLIB_ERR_NONE) {
      Serial.print("LoRa TX OK: ");
      Serial.println(packet);
    } else {
      Serial.print("LoRa TX failed, code ");
      Serial.println(txState);
    }
  }
}
