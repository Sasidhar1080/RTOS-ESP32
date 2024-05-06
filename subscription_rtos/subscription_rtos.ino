#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PORT 8100
#define BAUD 115200

// WiFi credentials
const char* ssid = "esw-m19@iiith";
const char* password = "e5W-eMai@3!20hOct";

// JSON document size
StaticJsonDocument<512> doc;

// AsyncWebServer instance
AsyncWebServer server(PORT);

// Function prototype for data processing
void action(char* data_input);

// Task handles
TaskHandle_t wifiTaskHandle;
TaskHandle_t serverTaskHandle;

// Flag for data reception
bool isDataReceived = false;

// Task to handle WiFi connection
void wifiConnectTask(void* pvParameters) {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  Serial.println("\nConnected to WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  // Notify the server task to start after connection
  xTaskNotifyGive(serverTaskHandle);
  vTaskDelete(NULL); // Delete task after connection
}

// Task to handle incoming data and run the server
void serverTask(void* pvParameters) {
  // Wait for notification from WiFi task
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Server route and data handling
  server.on("/", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL, [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    String jsonString;
    for (int i = 0; i < len; i++) {
      jsonString += (char)data[i];
    }
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    JsonObject m2m_cin = doc["m2m:sgn"]["m2m:nev"]["m2m:rep"]["m2m:cin"];
    const char* m2m_cin_con = m2m_cin["con"];
    char* data_c = const_cast<char*>(m2m_cin_con);
    action(data_c);
    request->send(200, "application/json", jsonString);
    isDataReceived = true; // Set flag after receiving data
  });

  server.begin();
  Serial.println("Server started");

  // Main loop for server task
  while (true) {
    if (isDataReceived) {
      isDataReceived = false;
      // Process the received data here (e.g., send to another task)
      // ...
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}

void setup() {
  Serial.begin(115200);

  // Create tasks
  xTaskCreate(wifiConnectTask, "WiFiTask", 2048, NULL, 1, &wifiTaskHandle);
  xTaskCreate(serverTask, "ServerTask", 4096, NULL, 1, &serverTaskHandle);
}

void loop() {
  // Tasks are managed by FreeRTOS scheduler
}

// Implement the 'action' function for data processing
void action(char* data_input) {
  // Add your data processing logic here
  // ...
}