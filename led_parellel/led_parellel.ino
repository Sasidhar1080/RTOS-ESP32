#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define the LED pins
#define LED_PIN_1 14
#define LED_PIN_2 27

// Function prototypes for the LED blink tasks
void vBlinkLedTask1(void *pvParameters);
void vBlinkLedTask2(void *pvParameters);

void setup() {
  // Start serial communication for debugging/feedback
  Serial.begin(115200);
  
  // Initialize the LED pins as outputs
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  // Create the FreeRTOS tasks for blinking the LEDs
  xTaskCreate(vBlinkLedTask1, "BlinkLed1", 2048, NULL, 1, NULL);
  xTaskCreate(vBlinkLedTask2, "BlinkLed2", 2048, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS scheduler manages task execution
  // No need for code in the loop function
}

// FreeRTOS task to blink the LED on pin 14 every 3 seconds
void vBlinkLedTask1(void *pvParameters) {
  bool ledState = false; // Track the LED state

  for (;;) {
    // Toggle the LED state
    ledState = !ledState;
    digitalWrite(LED_PIN_1, ledState);

    // Serial output for debugging
    Serial.print("LED 14 is now: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Wait for 3 seconds
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// FreeRTOS task to blink the LED on pin 27 every 3 seconds
void vBlinkLedTask2(void *pvParameters) {
  bool ledState = false; // Track the LED state

  for (;;) {
    // Toggle the LED state
    ledState = !ledState;
    digitalWrite(LED_PIN_2, ledState);

    // Serial output for debugging
    Serial.print("LED 27 is now: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Wait for 3 seconds
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}
