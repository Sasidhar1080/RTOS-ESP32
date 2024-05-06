#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define the LED pin
#define LED_PIN 14

// Function prototype for the LED blink task
void vBlinkLedTask(void *pvParameters);

void setup() {
  // Start the serial communication
  Serial.begin(115200); // Common baud rate for serial communication
  
  // Initial feedback on startup
  Serial.println("Starting FreeRTOS LED blink task...");

  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  // Create the FreeRTOS task for blinking the LED
  xTaskCreate(vBlinkLedTask, "BlinkLed", 1024, NULL, 1, NULL); // Reduced stack size
}

void loop() {
  // FreeRTOS scheduler manages task execution
  // No need for code in the loop function
}

// FreeRTOS task to blink an LED every 3 seconds
void vBlinkLedTask(void *pvParameters) {
  bool ledState = false; // Keep track of the LED state

  for (;;) {
    // Toggle the LED state
    ledState = !ledState; // Flip the state
    digitalWrite(LED_PIN, ledState); // Update the LED based on the new state

    // Print the current state of the LED less frequently (optional)
    Serial.print("LED is now: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Wait for 3 seconds (3000 ms)
    vTaskDelay(1300 / portTICK_PERIOD_MS); // Consistent 3-second delay
  }
}
