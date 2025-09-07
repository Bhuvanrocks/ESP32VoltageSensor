// ESP32-C6 with 2 Voltage Sensors + 1 ACS712 Current Sensor + ThingSpeak + OLED
// Bhuvan Kumar

#include <WiFi.h>
#include "ThingSpeak.h"
#include <Wire.h>
#include <U8g2lib.h>

// ---------- WiFi credentials ----------
const char* ssid = "q";     
const char* password = "q";

// ---------- ThingSpeak credentials ----------
unsigned long myChannelNumber = q;
const char * myWriteAPIKey = "q";
WiFiClient client;

// ---------- Pin Configuration ----------
const int voltagePin1 = 0;
const int voltagePin2 = 1;
const int currentPin  = 2;

// ---------- Calibration ----------
const float VOLTAGE_RATIO = 1.26 * 5.0;
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;
const float sensitivity = 100.0;
float offset = 0;

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

  // Calibrate current sensor
  offset = calibrateCurrentSensor(currentPin);
  Serial.print("Current Sensor Calibrated. Offset = ");
  Serial.print(offset, 3);
  Serial.println(" V");
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
  delay(2000); // Display for 2 seconds before proceeding
}

// ---------- Calibration ----------
float calibrateCurrentSensor(int pin) {
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (sum / (float)samples) * (ADC_REF / ADC_MAX);
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

// ---------- Current Reading ----------
float readCurrentSensor(int pin, int samples = 50) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float sensorVoltage = (sum / (float)samples) * (ADC_REF / ADC_MAX);
  float actualVoltage = sensorVoltage - offset;
  float current = (actualVoltage * 1000.0) / sensitivity;
  return fabs(current);
}

// ---------- Main Loop ----------
void loop() {
  unsigned long currentMillis = millis();

  // Read sensors
  float voltage1 = readVoltageSensor(voltagePin1);
  float voltage2 = readVoltageSensor(voltagePin2);
  float current  = readCurrentSensor(currentPin);

  // Print to Serial Plotter
  Serial.print(voltage1, 2); Serial.print("\t");
  Serial.print(voltage2, 2); Serial.print("\t");
  Serial.println(current, 3);
  delay(200);

  // Send to ThingSpeak every 30s
  if (currentMillis - previousMillis >= uploadInterval) {
    previousMillis = currentMillis;
    if (WiFi.status() == WL_CONNECTED) {
      ThingSpeak.setField(1, voltage1);
      ThingSpeak.setField(2, voltage2);
      ThingSpeak.setField(3, current);
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

  // V1
  u8g2.setCursor(10, 15);
  u8g2.print("V1: ");
  dtostrf(voltage1, 4, 2, buf);
  u8g2.print(buf);
  u8g2.print("V");
  int barWidth1 = map(voltage1, 0, 12.0, 0, 100);
  u8g2.drawFrame(10, 18, 100, 10);
  u8g2.drawBox(10, 18, barWidth1, 10);

  // V2
  u8g2.setCursor(10, 40);
  u8g2.print("V2: ");
  dtostrf(voltage2, 4, 2, buf);
  u8g2.print(buf);
  u8g2.print("V");
  int barWidth2 = map(voltage2, 0, 12.0, 0, 100);
  u8g2.drawFrame(10, 43, 100, 10);
  u8g2.drawBox(10, 43, barWidth2, 10);

  // I
  u8g2.setCursor(10, 62);
  u8g2.print("I: ");
  dtostrf(current, 4, 3, buf);
  u8g2.print(buf);
  u8g2.print("A");

  // Upload Status Star Icon at Top Right
  if (uploadSuccess) {
    u8g2.setCursor(115, 10);
    u8g2.print("*"); // Show star if successful
  }

  u8g2.sendBuffer();
}
