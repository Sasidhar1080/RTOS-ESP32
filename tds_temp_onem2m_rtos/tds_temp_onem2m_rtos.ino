#include <WiFi.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <HTTPClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// WiFi credentials
// const char* ssid = "SCRC_LAB_IOT";
// const char* password = "Scrciiith@123";

const char* ssid = "esw-m19@iiith";
const char* password = "e5W-eMai@3!20hOct";  

// Sensor initialization
OneWire oneWire(27);
DallasTemperature tempSensor(&oneWire);
#define tdssensorPin 35

// Task parameters
#define STACK_SIZE 2048
#define WIFI_RETRY_DELAY 30000

// Sensor variables
int tdssensorValue = 0;
float tdsValue = 0;
float Voltage = 0;
float Temp = 0;
float tdsValueWithoutTemp = 0;

// WiFi and HTTP client
WiFiClient client;
HTTPClient http;

// OneM2M connection information
#define CSE_IP "dev-onem2m.iiit.ac.in"
#define CSE_PORT 443
#define OM2M_ORGIN "wdmon@20:gpod@llk4"
#define OM2M_MN "/~/in-cse/in-name/"
#define OM2M_AE "AE-WM/WM-WD"
#define OM2M_DATA_CONT "WM-WD-VN00-00/Data"

// Task handles
TaskHandle_t wifiTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t postTaskHandle;

// Semaphore for data synchronization
SemaphoreHandle_t xDataMutex;


// Shared sensor data structure
struct SensorData {
    float temp;
    float tds;
    float tdsWithoutTemp;
};

SensorData sensorData;

// WiFi connection task
void vTaskWiFiConnection(void* pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(WIFI_RETRY_DELAY / portTICK_PERIOD_MS);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");

    vTaskSuspend(NULL);  // Suspend the task once WiFi is connected
}

// Sensor reading task
void vTaskReadSensors(void* pvParameters) {
    while (true) {
        // Read sensor data
        tdssensorValue = analogRead(tdssensorPin);
        Voltage = tdssensorValue * 3.3 / 1024.0;
        
        float compensationCoefficient = 1.0 + 0.02 * (Temp - 25.0);
        float compensationVoltage = Voltage / compensationCoefficient;
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
            - 255.86 * compensationVoltage * compensationVoltage
            + 857.39 * compensationVoltage) * 0.5;
        
        float compensationCoefficientWithoutTemp = 1.0 + 0.02 * (25.0 - 25.0);
        float compensationVoltageWithoutTemp = Voltage / compensationCoefficientWithoutTemp;
        tdsValueWithoutTemp = (133.42 * compensationVoltageWithoutTemp * compensationVoltageWithoutTemp
            - 255.86 * compensationVoltageWithoutTemp * compensationVoltageWithoutTemp
            + 857.39 * compensationVoltageWithoutTemp) * 0.5;

        tempSensor.requestTemperaturesByIndex(0);
        Temp = tempSensor.getTempCByIndex(0);

        Serial.print("TDS Value (with Temp Compensation) = ");
        Serial.print(tdsValue);
        Serial.println(" ppm");

        Serial.print("TDS Value (without Temp Compensation) = ");
        Serial.print(tdsValueWithoutTemp);
        Serial.println(" ppm");

        Serial.print("Temperature: ");
        Serial.print(Temp);
        Serial.println(" °C");

        // Update shared data structure with mutex
        if (xSemaphoreTake(xDataMutex, (TickType_t)10) == pdTRUE) {
            sensorData.temp = Temp;
            sensorData.tds = tdsValue;
            sensorData.tdsWithoutTemp = tdsValueWithoutTemp;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Wait before next reading
    }
}

// Task for posting data to server
void vTaskPostData(void* pvParameters) {
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            // Get shared sensor data with mutex
            if (xSemaphoreTake(xDataMutex, (TickType_t)10) == pdTRUE) {
                SensorData data = sensorData;
                xSemaphoreGive(xDataMutex);

                // Construct the HTTP POST request data
                String dataString = "[" + String(data.temp) + ", " + String(data.tdsWithoutTemp) + ", " + String(data.tds) + ", " + String(Voltage) + "]";
                
                String server = "http://" + String(CSE_IP) + ":" + String(CSE_PORT) + String(OM2M_MN);
                http.begin(client, server + OM2M_AE + "/" + OM2M_DATA_CONT);
                http.addHeader("X-M2M-Origin", OM2M_ORGIN);
                http.addHeader("Content-Type", "application/json;ty=4");
                
                String req_data = String() + "{\"m2m:cin\": {"
                    + "\"con\": \"" + dataString + "\","
                    + "\"rn\": \"" + "cin_" + String(millis()) + "\","
                    + "\"cnf\": \"text\""
                    + "}}";

                Serial.println("Server URL: " + server);
                Serial.println("Request Data: " + req_data);

                int responseCode = http.POST(req_data);
                Serial.println("HTTP Response Code: " + String(responseCode));
                http.end();

                vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay after POST request
            }
        }
        else {
            Serial.println("WiFi Disconnected. Attempting to reconnect...");
            vTaskResume(wifiTaskHandle);  // Resume WiFi connection task
        }
    }
}

// Setup function to create tasks
void setup() {
    Serial.begin(115200);
    tempSensor.begin();

    // Create the semaphore
    xDataMutex = xSemaphoreCreateMutex();

    // Create tasks for WiFi, sensor reading, and data posting
    xTaskCreate(vTaskWiFiConnection, "WiFiConnection", STACK_SIZE, NULL, 2, &wifiTaskHandle);
    xTaskCreate(vTaskReadSensors, "ReadSensors", STACK_SIZE, NULL, 1, &sensorTaskHandle);
    xTaskCreate(vTaskPostData, "PostData", STACK_SIZE, NULL, 1, &postTaskHandle);
}

// Empty loop since tasks manage concurrency
void loop() {
    // In FreeRTOS, tasks manage concurrency, so the loop can remain empty
}
