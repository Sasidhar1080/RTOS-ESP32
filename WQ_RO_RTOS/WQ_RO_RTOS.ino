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

// Relay pins
const int Solenoid1 = 15;  // Solenoid for intake
const int Motor1 = 16;  // Motor for intake
const int Solenoid2 = 17;  // Solenoid for drain
const int Motor2 = 18;  // Motor for drain

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
};

// Declare sensorData globally
SensorData sensorData;

// Relay control functions
void intake() {
    Serial.println("Intaking...");
    digitalWrite(Solenoid1, LOW);  // Turn on intake solenoid
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for solenoid to open
    
    digitalWrite(Motor1, LOW);  // Turn on intake motor
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Intake for 5 seconds
    
    digitalWrite(Solenoid1, HIGH);  // Turn off intake solenoid
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Ensure solenoid closes
    
    digitalWrite(Motor1, HIGH);  // Turn off intake motor
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Final delay
}

void drain() {
    Serial.println("Draining...");
    digitalWrite(Solenoid2, LOW);  // Turn on drain solenoid
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for solenoid to open
    
    digitalWrite(Motor2, LOW);  // Turn on drain motor
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Drain for 5 seconds
    
    digitalWrite(Solenoid2, HIGH);  // Turn off drain solenoid
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Ensure solenoid closes
    
    digitalWrite(Motor2, HIGH);  // Turn off drain motor
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Final delay
}

void flush() {
    Serial.println("Flushing...");
    drain();  // Drain first
    intake();  // Intake
    drain();  // Drain again

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Final delay
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
    String epochTime = String(rtc.getEpoch());  // Convert unsigned long to String

    // Flush and intake water
    flush();  // Start with flush
    intake();  // Then intake

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

    // Update shared sensor data
    sensorData.temp = temp;
    sensorData.tds = tdsValue;  // With temperature compensation
    sensorData.tdsVoltage = voltage;  // Raw TDS voltage
    sensorData.tdsWithoutTemp = tdsValueWithoutTemp;  // Without temperature compensation

    // Log sensor data
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("TDS Voltage: "); Serial.println(voltage);  // Voltage from TDS sensor
    Serial.print("Uncompensated TDS: "); Serial.println(tdsValueWithoutTemp);  // TDS without temp compensation
    Serial.print("Compensated TDS: "); Serial.println(tdsValue);  // TDS with temp compensation

    // Post data to OneM2M with retries
    if (WiFi.status() == WL_CONNECTED) {
        int maxAttempts = 10;
        int statusCode = -1;

        for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
            String dataString = "[" + epochTime + ", " + String(temp) + ", " + String(voltage) + ", " + String(tdsValueWithoutTemp) + ", " + String(tdsValue) + "]";

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
    pinMode(Solenoid1, OUTPUT);
    pinMode(Motor1, OUTPUT);
    pinMode(Solenoid2, OUTPUT);
    pinMode(Motor2, OUTPUT);

    // Initialize to "off" (assuming relays are active-low)
    digitalWrite(Solenoid1, HIGH);  // Turn off intake solenoid
    digitalWrite(Motor1, HIGH);  // Turn off intake motor
    digitalWrite(Solenoid2, HIGH);  // Turn off drain solenoid
    digitalWrite(Motor2, HIGH);  // Turn off drain motor

    // Create the main FreeRTOS task
    xTaskCreate(mainTask, "MainTask", STACK_SIZE, NULL, 2, &mainTaskHandle);
}

void loop() {
    // The main work is done in FreeRTOS tasks, so this loop is empty
}