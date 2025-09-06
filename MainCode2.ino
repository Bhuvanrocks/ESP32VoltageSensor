// ESP32-C6 with 2 Voltage Sensors + 1 ACS712 Current Sensor + ThingSpeak
// Bhuvan Kumar

#include <WiFi.h>
#include "ThingSpeak.h"

// ---------- WiFi credentials ----------
const char* ssid = "SSID";     
const char* password = "Password";

// ThingSpeak credentials
unsigned long myChannelNumber = CHANNEL_ID;   // Replace with your channel ID
const char * myWriteAPIKey = WRITE_API_KEY;
WiFiClient client;

// ---------- Pin Configuration ----------
const int voltagePin1 = 0;   // First voltage sensor
const int voltagePin2 = 1;   // Second voltage sensor
const int currentPin  = 2;   // ACS712 current sensor

// ---------- Voltage Sensor Calibration ----------
const float VOLTAGE_RATIO = 1.26 * 5.0;  // Adjust this based on divider calibration
const float ADC_REF = 3.3;               // ESP32 ADC reference (V)
const int ADC_MAX = 4095;                // 12-bit ADC

// ---------- ACS712 Current Sensor ----------
const float sensitivity = 100.0;  // mV per A (100 for 20A module)
float offset = 0;                 // Zero-current offset (V)

// ---------- Timing ----------
unsigned long previousMillis = 0;
const unsigned long uploadInterval = 30000;  // 30s

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  // Try to connect WiFi with timeout
  Serial.print("Connecting to WiFi...");
  unsigned long startAttemptTime = millis();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) { // 10s timeout
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }

  // Calibrate current sensor (no load connected during this step!)
  offset = calibrateCurrentSensor(currentPin);
  Serial.print("Current Sensor Calibrated. Offset = ");
  Serial.print(offset, 3);
  Serial.println(" V");
}

// ---------- Calibration for ACS712 ----------
float calibrateCurrentSensor(int pin) {
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (sum / (float)samples) * (ADC_REF / ADC_MAX);
}

// ---------- Averaged Voltage Reading ----------
float readVoltageSensor(int pin, int samples = 50) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float sensorVoltage = (sum / (float)samples) * (ADC_REF / ADC_MAX);
  return sensorVoltage * VOLTAGE_RATIO;  // Convert back to measured voltage
}

// ---------- Averaged Current Reading ----------
float readCurrentSensor(int pin, int samples = 50) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float sensorVoltage = (sum / (float)samples) * (ADC_REF / ADC_MAX);

  // Subtract offset
  float actualVoltage = sensorVoltage - offset;

  // Convert to current (Amps)
  float current = (actualVoltage * 1000.0) / sensitivity;

  return fabs(current);  // Absolute value
}

// ---------- Main Loop ----------
void loop() {
  unsigned long currentMillis = millis();

  // Read sensor values
  float voltage1 = readVoltageSensor(voltagePin1);
  float voltage2 = readVoltageSensor(voltagePin2);
  float current  = readCurrentSensor(currentPin);

  // Print for Serial Plotter (real-time)
  Serial.print(voltage1, 2); Serial.print("\t");
  Serial.print(voltage2, 2); Serial.print("\t");
  Serial.println(current, 3);
  delay(200);
  // Send to ThingSpeak every 30 seconds (non-blocking)
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
        Serial.print("ThingSpeak update failed, error code: ");
        Serial.println(x);
      }
    } else {
      Serial.println("WiFi not connected. Skipping ThingSpeak update.");
    }
  }
}
