#include <WiFi.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <HTTPClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <NTPClient.h>
#include <WiFiUDP.h>
#include <ESP32Time.h>

// WiFi credentials
const char* ssid = "TP-Link_1EF4";  // Your WiFi network name
const char* password = "!th3sw9sT^uZ0";  // Your WiFi network password

// Sensor initialization
OneWire oneWire(27);  // OneWire data pin
DallasTemperature tempSensor(&oneWire);

// Sensor pins
const int tdssensorPin = 35;  // TDS sensor pin (analog)
const int phSensorPin = 14;   // pH sensor pin (analog)
const int turbiditySensorPin = 15;  // Turbidity sensor pin (analog)

// Relay pins
const int RELAY1 = 32;  // Relay for intake
const int RELAY2 = 33;  // Relay for drain

// Time synchronization
WiFiUDP ntpUDP;  // UDP connection for NTP
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // NTP Client for time sync
ESP32Time rtc;  // Real-time clock library

// Task parameters
const size_t STACK_SIZE = 4096;  // Stack size for FreeRTOS tasks
const int WIFI_RETRY_DELAY = 30000;  // 30-second WiFi retry delay

// OneM2M connection information
const char* CSE_IP = "dev-onem2m.iiit.ac.in";  // Server IP
const int CSE_PORT = 443;  // HTTPS port
const char* OM2M_ORGIN = "wdmon@20:gpod@llk4";  // Origin for OneM2M
const char* OM2M_MN = "/~/in-cse/in-name/";  // OneM2M main node
const char* OM2M_AE = "AE-WM/WM-WD";  // Application Entity
const char* OM2M_DATA_CONT = "WM-WD-VN00-00/Data";  // Data container

// Global task handle
TaskHandle_t mainTaskHandle;

// Shared sensor data structure
struct SensorData {
    float temp;
    float tds;
    float tdsVoltage;
    float tdsWithoutTemp;
    float ph;
    float turbidity;
};

// Declare sensorData globally to ensure accessibility
SensorData sensorData;

// Relay control functions
void flush() {
    Serial.println("Flushing...");
    digitalWrite(RELAY1, LOW);  // Turn on intake
    digitalWrite(RELAY2, LOW);  // Turn on drain
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Flush for 5 seconds
    digitalWrite(RELAY1, HIGH);  // Turn off intake
    digitalWrite(RELAY2, HIGH);  // Turn off drain
}

void intake() {
    Serial.println("Intaking...");
    digitalWrite(RELAY1, LOW);  // Turn on intake relay
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Intake for 5 seconds
    digitalWrite(RELAY1, HIGH);  // Turn off intake relay
}

void drain() {
    Serial.println("Draining...");
    digitalWrite(RELAY2, LOW);  // Turn on drain relay
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Drain for 5 seconds
    digitalWrite(RELAY2, HIGH);  // Turn off drain relay
}

void mainTask(void* pvParameters) {
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        vTaskDelay(WIFI_RETRY_DELAY / portTICK_PERIOD_MS);  // Retry every 30 seconds
    }
    Serial.println("Connected to WiFi!");

    // Synchronize time with NTP
    timeClient.begin();
    timeClient.update();
    rtc.setTime(timeClient.getEpochTime());

    // Get epoch time
    String epochTime = String(rtc.getEpoch());  // Correct conversion from unsigned long to String

    // Flush and intake water
    flush();
    intake();

    // Read sensor data
    tempSensor.begin();
    tempSensor.requestTemperatures();
    float temp = tempSensor.getTempCByIndex(0);

    int tdsSensorValue = analogRead(tdssensorPin);  // Read TDS sensor
    float voltage = tdsSensorValue * 3.3 / 1024.0;  // Convert to voltage
    float compensationCoefficient = 1.0 + 0.02 * (temp - 25.0);  // With temperature compensation
    float compensationVoltage = voltage / compensationCoefficient;
    float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                     - 255.86 * compensationVoltage * compensationVoltage 
                     + 857.39 * compensationVoltage) * 0.5;

    // TDS without temperature compensation
    float compensationCoefficientWithoutTemp = 1.0 + 0.02 * (25.0 - 25.0);
    float compensationVoltageWithoutTemp = voltage / compensationCoefficientWithoutTemp;
    float tdsValueWithoutTemp = (133.42 * compensationVoltageWithoutTemp * compensationVoltageWithoutTemp 
                                - 255.86 * compensationVoltageWithoutTemp * compensationVoltageWithoutTemp 
                                + 857.39 * compensationVoltageWithoutTemp) * 0.5;

    int phSensorValue = analogRead(phSensorPin);  // Read pH sensor
    float phVoltage = phSensorValue * 5.0 / 1024.0;
    float phValue = 3.5 * phVoltage + 0.5;  // Example calculation; adjust as needed

    int turbiditySensorValue = analogRead(turbiditySensorPin);  // Read turbidity sensor
    float turbidityVoltage = turbiditySensorValue * 5.0 / 1024.0;
    float turbidityValue = 100.0 * turbidityVoltage;  // Example calculation; adjust as needed

    // Update shared sensor data
    sensorData.temp = temp;
    sensorData.tds = tdsValue;  // With temperature compensation
    sensorData.tdsVoltage = voltage;  // Raw TDS voltage
    sensorData.tdsWithoutTemp = tdsValueWithoutTemp;  // Without temperature compensation
    sensorData.ph = phValue;
    sensorData.turbidity = turbidityValue;

    // Log sensor data
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("TDS Voltage: "); Serial.println(voltage);  // Voltage from TDS sensor
    Serial.print("Uncompensated TDS: "); Serial.println(tdsValueWithoutTemp);  // TDS without temp compensation
    Serial.print("Compensated TDS: "); Serial.println(tdsValue);  // TDS with temp compensation
    Serial.print("pH Value: "); Serial.println(phValue);
    Serial.print("Turbidity Value: "); Serial.println(turbidityValue);

    // Post data to OneM2M with retries
    if (WiFi.status() == WL_CONNECTED) {
        int maxAttempts = 10;
        int statusCode = -1;

        for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
            String dataString = "[" + epochTime + ", " + String(temp) + ", " + String(voltage) + ", " + String(tdsValueWithoutTemp) + ", " + String(tdsValue) + ", " + String(phValue) + ", " + String(turbidityValue) + "]";

            HTTPClient http;
            http.begin("http://" + String(CSE_IP) + ":" + String(CSE_PORT) + String(OM2M_MN) + OM2M_AE + "/" + OM2M_DATA_CONT);
            http.addHeader("X-M2M-Origin", OM2M_ORGIN);
            http.addHeader("Content-Type", "application/json;ty=4");

            String req_data = "{\"m2m:cin\": {\"con\": \"" + dataString + "\", \"rn\": \"cin_" + String(millis()) + "\"}}";

            statusCode = http.POST(req_data);  // Post data and get response status
            Serial.println("HTTP Response Code: " + String(statusCode));
            http.end();

            if (statusCode == 201) {  // If successful posting
                break;
            }

            vTaskDelay(5000 / portTICK_PERIOD_MS);  // Wait 5 seconds before retrying
        }

        if (statusCode != 201) {  // If all attempts fail
            Serial.println("All attempts to post data failed. Resetting...");
            ESP.restart();  // Restart the ESP32
        }
    }

    // Drain water and wait before restarting the loop
    drain();
    Serial.println("Waiting for 1 hour before repeating...");
    vTaskDelay(3600000 / portTICK_PERIOD_MS);  // Wait for 1 hour before repeating
}

void setup() {
    Serial.begin(115200);

    // Set relay pins to outputs and ensure they are off initially (relays are active-low)
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    digitalWrite(RELAY1, HIGH);  // Turn off intake
    digitalWrite(RELAY2, HIGH);  // Turn off drain

    // Create the main FreeRTOS task
    xTaskCreate(mainTask, "MainTask", STACK_SIZE, NULL, 2, &mainTaskHandle);
}

void loop() {
    // The main work is done in FreeRTOS tasks, so this loop is empty
}
