// ESP32-C6 with 2 Voltage Sensors + 1 ACS712 Current Sensor + ThingSpeak + OLED
// Bhuvan Kumar

#include <WiFi.h>
#include "ThingSpeak.h"
#include <Wire.h>
#include <U8g2lib.h>

// ---------- WiFi credentials ----------
const char* ssid = "SSID";     
const char* password = "Password";

// ---------- ThingSpeak credentials ----------
unsigned long myChannelNumber = CHANNEL_NUMBER;
const char * myWriteAPIKey = WRITE_API_KEY;
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

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  Serial.print("Connecting to WiFi...");
  unsigned long startAttemptTime = millis();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }

  // Calibrate current sensor
  offset = calibrateCurrentSensor(currentPin);
  Serial.print("Current Sensor Calibrated. Offset = ");
  Serial.print(offset, 3);
  Serial.println(" V");

  // OLED setup
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
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

// ---------- Draw Bar Graph ----------
void drawBar(int x, int y, float value, float maxValue, const char* label, float displayValue) {
  int barWidth = map(value, 0, maxValue, 0, 100);  // 100 pixel width bar
  u8g2.drawStr(x, y - 2, label);                   // Label above bar
  u8g2.drawBox(x, y, barWidth, 10);               // Filled bar

  // Display numeric value next to the bar
  char buf[10];
  dtostrf(displayValue, 4, 2, buf);  // 2 decimal points
  u8g2.setCursor(x + 105, y + 8);
  u8g2.print(buf);
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
      } else {
        Serial.print("ThingSpeak update failed, error: ");
        Serial.println(x);
      }
    } else {
      Serial.println("WiFi not connected. Skipping update.");
    }
  }

  // Display on OLED
// Display on OLED
// Display on OLED
// Display on OLED
u8g2.clearBuffer();
u8g2.setFont(u8g2_font_ncenB08_tr);

// V1 label and value
char buf[10];
u8g2.setCursor(10, 15);
u8g2.print("V1: ");
dtostrf(voltage1, 4, 2, buf);
u8g2.print(buf);
u8g2.print("V");

// V1 bar graph below
int barWidth1 = map(voltage1, 0, 12.0, 0, 100);
u8g2.drawFrame(10, 18, 100, 10);
u8g2.drawBox(10, 18, barWidth1, 10);

// V2 label and value
u8g2.setCursor(10, 40);
u8g2.print("V2: ");
dtostrf(voltage2, 4, 2, buf);
u8g2.print(buf);
u8g2.print("V");

// V2 bar graph below
int barWidth2 = map(voltage2, 0, 12.0, 0, 100);
u8g2.drawFrame(10, 43, 100, 10);
u8g2.drawBox(10, 43, barWidth2, 10);

// I label and value
u8g2.setCursor(10, 62);
u8g2.print("I: ");
dtostrf(current, 4, 3, buf);
u8g2.print(buf);
u8g2.print("A");

u8g2.sendBuffer();



}
