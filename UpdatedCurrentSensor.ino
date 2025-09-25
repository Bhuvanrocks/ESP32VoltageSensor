// ESP32-C6 with 1 Voltage Sensor + INA219 Current Sensor + ThingSpeak + OLED
// Bhuvan Kumar

#include <WiFi.h>
#include "ThingSpeak.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_INA219.h>

// ---------- WiFi credentials ----------
const char* ssid = "MBA-VTU";     
const char* password = "Dscemba@2022";

// ---------- ThingSpeak credentials ----------
unsigned long myChannelNumber = 3063862;
const char * myWriteAPIKey = "OKTNQOPIKJPYZKVE";
WiFiClient client;

// ---------- Pin Configuration ----------
const int voltagePin = 1;   // Analog pin for Voltage Sensor

// ---------- Calibration ----------
const float VOLTAGE_RATIO = 1.26 * 5.0; // Voltage divider scaling
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;

// ---------- INA219 Current Sensor ----------
Adafruit_INA219 ina219;

// ---------- Timing ----------
unsigned long previousMillis = 0;
const unsigned long uploadInterval = 30000;

// ---------- OLED Configuration ----------
#define OLED_SDA 21
#define OLED_SCL 22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------- Upload status ----------
bool uploadSuccess = false;

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // OLED setup
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  showWiFiStatus();

  // INA219 init
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("INA219 Current Sensor Initialized");
}

// ---------- Show WiFi Status on OLED ----------
void showWiFiStatus() {
  u8g2.clearBuffer();
  u8g2.setCursor(10, 20);
  u8g2.print("Connecting to WiFi...");
  u8g2.sendBuffer();

  unsigned long startAttemptTime = millis();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  u8g2.clearBuffer();
  u8g2.setCursor(10, 20);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    u8g2.print("WiFi Connected!");
  } else {
    Serial.println("\nWiFi Connection Failed!");
    u8g2.print("Connection Failed");
  }
  u8g2.sendBuffer();
  delay(2000);
}

// ---------- Voltage Reading ----------
float readVoltageSensor(int pin, int samples = 50) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float sensorVoltage = (sum / (float)samples) * (ADC_REF / ADC_MAX);
  return sensorVoltage * VOLTAGE_RATIO;
}

// ---------- Current Reading from INA219 ----------
float readCurrentINA219() {
  return abs(ina219.getCurrent_mA() / 1000.0); // Convert mA to A
}

// ---------- Main Loop ----------
void loop() {
  unsigned long currentMillis = millis();

  // Read sensors
  float voltage = readVoltageSensor(voltagePin);
  float current = readCurrentINA219();

  // Print to Serial
  Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.print(" V\t");
  Serial.print("Current: "); Serial.print(current, 3); Serial.println(" A");
  delay(200);

  // Send to ThingSpeak every 30s
  if (currentMillis - previousMillis >= uploadInterval) {
    previousMillis = currentMillis;
    if (WiFi.status() == WL_CONNECTED) {
      ThingSpeak.setField(1, voltage);
      ThingSpeak.setField(2, current);
      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if (x == 200) {
        Serial.println("ThingSpeak update successful!");
        uploadSuccess = true;
      } else {
        Serial.print("ThingSpeak update failed, error: ");
        Serial.println(x);
        uploadSuccess = false;
      }
    } else {
      Serial.println("WiFi not connected. Skipping update.");
      uploadSuccess = false;
    }
  }

  // Display on OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  char buf[10];

  // Voltage
  u8g2.setCursor(10, 20);
  u8g2.print("V: ");
  dtostrf(voltage, 4, 2, buf);
  u8g2.print(buf);
  u8g2.print("V");
  int barWidthV = map(voltage, 0, 12.0, 0, 100);
  u8g2.drawFrame(10, 25, 100, 10);
  u8g2.drawBox(10, 25, barWidthV, 10);

  // Current
  u8g2.setCursor(10, 55);
  u8g2.print("I: ");
  dtostrf(current, 4, 3, buf);
  u8g2.print(buf);
  u8g2.print("A");

  // Upload Status
  if (uploadSuccess) {
    u8g2.setCursor(115, 10);
    u8g2.print("*");
  }

  u8g2.sendBuffer();
}
