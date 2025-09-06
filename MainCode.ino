// ESP32-C6 Voltage + Current Monitoring with ThingSpeak
// Bhuvan Kumar

#include <WiFi.h>
#include "ThingSpeak.h"

// ---------- WiFi credentials ----------
// WiFi credentials
const char* ssid = "SSID";     
const char* password = "Password";

// ThingSpeak credentials
unsigned long myChannelNumber = 1111111;   // Replace with your channel ID
const char * myWriteAPIKey = "*********";

WiFiClient client;

// ---------- Voltage Sensor pins ----------
const int voltagePin1 = 0;   // GPIO0
const int voltagePin2 = 1;   // GPIO1
const float RATIO = 5.0 * 1.26;  // Divider ratio calibration

// ---------- Current Sensor pins ----------
const int currentPin1 = 2;   // GPIO2 (ACS712 #1)
const int currentPin2 = 3;   // GPIO3 (ACS712 #2)
const float sensitivity = 0.185; // V/A (ACS712-05B). Use 0.100 for 20A, 0.066 for 30A.
const float ADC_Vmax = 3.3;      // ESP32-C6 ADC max voltage
const int ADC_Resolution = 4095; // 12-bit ADC

// Auto-calibrated offsets
float Vref1 = 0, Vref2 = 0;

// ---------- Timing ----------
unsigned long currentTime, previousTime, delayTime = 30000; // 30 sec upload interval

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // -------- Current Sensor Calibration --------
  Serial.println("Calibrating ACS712 sensors...");
  long sum1 = 0, sum2 = 0;
  for (int i = 0; i < 500; i++) {
    sum1 += analogRead(currentPin1);
    sum2 += analogRead(currentPin2);
    delay(2);
  }
  Vref1 = (sum1 / 500.0) * (ADC_Vmax / ADC_Resolution);
  Vref2 = (sum2 / 500.0) * (ADC_Vmax / ADC_Resolution);

  Serial.print("Vref1 = ");
  Serial.print(Vref1, 3);
  Serial.print(" V | Vref2 = ");
  Serial.print(Vref2, 3);
  Serial.println(" V");
}

void loop() {
  currentTime = millis();

  // -------- Voltage Sensors --------
  int rawV1 = analogRead(voltagePin1);
  int rawV2 = analogRead(voltagePin2);
  float voltageAtPin1 = (rawV1 / 4095.0) * 3.3;
  float voltageAtPin2 = (rawV2 / 4095.0) * 3.3;
  float actualVoltage1 = voltageAtPin1 * RATIO;
  float actualVoltage2 = voltageAtPin2 * RATIO;

  // -------- Current Sensors --------
  int rawC1 = analogRead(currentPin1);
  int rawC2 = analogRead(currentPin2);
  float voltageC1 = (rawC1 * ADC_Vmax) / ADC_Resolution;
  float voltageC2 = (rawC2 * ADC_Vmax) / ADC_Resolution;
  float current1 = (voltageC1 - Vref1) / sensitivity;
  float current2 = (voltageC2 - Vref2) / sensitivity;

  // -------- Serial Output --------
  Serial.print(actualVoltage1, 2);
  Serial.print(" ");
  Serial.print(actualVoltage2, 2);
  Serial.print(" ");
  Serial.print(current1, 3);
  Serial.print(" ");
  Serial.println(current2, 3);

  delay(200); // Fast sampling for Serial Monitor

  // -------- ThingSpeak Upload every 30s --------
  if (currentTime - previousTime >= delayTime) {
    ThingSpeak.setField(1, actualVoltage1);
    ThingSpeak.setField(2, actualVoltage2);
    ThingSpeak.setField(3, current1);
    ThingSpeak.setField(4, current2);

    int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    if (response == 200) {
      Serial.println("Data sent to ThingSpeak successfully.");
    } else {
      Serial.print("Error sending data. HTTP error code: ");
      Serial.println(response);
    }
    previousTime = currentTime;
  }
}
