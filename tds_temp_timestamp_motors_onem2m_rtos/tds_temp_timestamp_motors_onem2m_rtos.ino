#include <WiFi.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <HTTPClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "TP-Link_1EF4";  // Change to your network name
const char* password = "!th3sw9sT^uZ0";  // Change to your network password

// Sensor initialization
OneWire oneWire(27);
DallasTemperature tempSensor(&oneWire);
const int tdssensorPin = 35;

// Relay pins
const int RELAY1 = 32;  // Relay for intake
const int RELAY2 = 33;  // Relay for drain

// Task parameters
const size_t STACK_SIZE = 2048;
const int WIFI_RETRY_DELAY = 30000;  // 30 seconds

// OneM2M connection information
const char* CSE_IP = "dev-onem2m.iiit.ac.in";
const int CSE_PORT = 443;  // HTTPs port for secure connection
const char* OM2M_ORGIN = "wdmon@20:gpod@llk4";
const char* OM2M_MN = "/~/in-cse/in-name/";
const char* OM2M_AE = "AE-WM/WM-WD";
const char* OM2M_DATA_CONT = "WM-WD-VN00-00/Data";

// WiFi and HTTP client
WiFiClient client;
HTTPClient http;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // NTP Server, UTC offset, update interval

// Task handles
TaskHandle_t wifiTaskHandle;
TaskHandle_t mainTaskHandle;

// Semaphore for data synchronization
SemaphoreHandle_t xDataMutex;

// Shared sensor data structure
struct SensorData {
    float temp;
    float tds;
    float tdsWithoutTemp;
};

// Initialize shared sensor data
SensorData sensorData;

// Relay control functions
void flush() {
    Serial.println("Flushing...");
    digitalWrite(RELAY1, LOW);  // Turn on intake relay
    digitalWrite(RELAY2, LOW);  // Turn on drain relay
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

// Main processing task
void mainTask(void* pvParameters) {
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        vTaskDelay(WIFI_RETRY_DELAY / portTICK_PERIOD_MS);  // Retry every 30 seconds
    }
    Serial.println("Connected to WiFi!");

    // Get timestamp from NTP
    timeClient.begin();
    timeClient.update();
    unsigned long epoch = timeClient.getEpochTime();

    // Flush and intake
    flush();
    intake();

    // Read sensor data
    tempSensor.begin();
    tempSensor.requestTemperatures();
    float temp = tempSensor.getTempCByIndex(0);

    int tdssensorValue = analogRead(tdssensorPin);
    float voltage = tdssensorValue * 3.3 / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (temp - 25.0);
    float compensationVoltage = voltage / compensationCoefficient;
    float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                     - 255.86 * compensationVoltage * compensationVoltage 
                     + 857.39 * compensationVoltage) * 0.5;

    sensorData.temp = temp;
    sensorData.tds = tdsValue;

    Serial.print("Temperature: "); 
    Serial.println(temp);
    Serial.print("TDS Value: ");
    Serial.println(tdsValue);

    // Post data to OneM2M
    if (WiFi.status() == WL_CONNECTED) {
        String dataString = "[" + String(epoch) + ", " + String(temp) + ", " + String(tdsValue) + ", " + String(voltage) + "]";

        http.begin(client, "http://" + String(CSE_IP) + ":" + String(CSE_PORT) + String(OM2M_MN) + OM2M_AE + "/" + OM2M_DATA_CONT);
        http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");

        String req_data = "{\"m2m:cin\": {\"con\": \"" + dataString + "\", \"rn\": \"cin_" + String(millis()) + "\"}}";
        int responseCode = http.POST(req_data);
        Serial.println("HTTP Response Code: " + String(responseCode));
        http.end();
    }

    // Drain and wait before restarting the loop
    drain();

    // Wait for 2 minutes before restarting the loop
    vTaskDelay(120000 / portTICK_PERIOD_MS);
}

void setup() {
    Serial.begin(115200);

    // Set up relay pins as outputs and ensure they are off initially
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    digitalWrite(RELAY1, HIGH);  // Turn off intake
    digitalWrite(RELAY2, HIGH);  // Turn off drain

    // Create the main task
    xTaskCreate(mainTask, "MainTask", STACK_SIZE, NULL, 2, &mainTaskHandle);
}

void loop() {
    // The loop is empty, as FreeRTOS tasks handle everything
}
