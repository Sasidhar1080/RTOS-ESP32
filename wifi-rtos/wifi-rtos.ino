#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Wi-Fi credentials
const char* ssid = "TP-Link_1EF4";
const char* password = "!th3sw9sT^uZ0";


// LED Pin
const int ledPin = 14;

// Function prototype for the Wi-Fi connection task
void vWiFiConnectionTask(void *pvParameters);

void setup() {
  // Start serial communication
  Serial.begin(115200);

  Serial.println("mac address :");
  Serial.print(WiFi.macAddress());

  // Set LED pin as output
  pinMode(ledPin, OUTPUT);

  // Create the FreeRTOS task for Wi-Fi connection
  xTaskCreate(vWiFiConnectionTask, "WiFiConnect", 4096, NULL, 1, NULL);

  // Additional setup (if required) can be done here
}

void loop() {
// FreeRTOS scheduler manages task execution
// No need for code in the loop function
}

// FreeRTOS task to connect to Wi-Fi
void vWiFiConnectionTask(void *pvParameters) {
  // Start the Wi-Fi connection process
  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password); // Connect to Wi-Fi with given credentials
  
  // Loop until the Wi-Fi is connected
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for a while before re-checking
  }

  Serial.println();
  Serial.println("Connected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Get the local IP address
  Serial.println("mac address :");
  Serial.print(WiFi.macAddress());
    // Turn on the LED to indicate successful connection
  digitalWrite(ledPin, HIGH);
  // This task could then either end or continue with additional functionality
  vTaskDelete(NULL); // End this task after connecting to Wi-Fi
}
