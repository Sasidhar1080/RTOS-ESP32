#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define the LED pins
#define LED_PIN_1 14
#define LED_PIN_2 27
#define LED_PIN_3 35


// Function prototype for the LED blink task
void vBlinkTwoLedsTask(void *pvParameters);

void setup() {
  // Start serial communication
  Serial.begin(115200); // Common baud rate
  
  // Initialize the LED pins as outputs
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);

  // Create the FreeRTOS task for blinking the LEDs
  xTaskCreate(vBlinkTwoLedsTask, "BlinkTwoLeds", 1000, NULL, 1, NULL); // Larger stack size
}

void loop() {
  // FreeRTOS scheduler handles task execution
  // No need for code in the loop function
}

// FreeRTOS task to blink two LEDs every 3 seconds
void vBlinkTwoLedsTask(void *pvParameters) {
  bool ledState = false; // Track the LEDs' state

  for (;;) {
    // Toggle the state of both LEDs
    ledState = !ledState; // Flip the state
    digitalWrite(LED_PIN_1, ledState); // Set LED on pin 14
    digitalWrite(LED_PIN_2, ledState); // Set LED on pin 27
    digitalWrite(LED_PIN_3, ledState); // Set LED on pin 14
 

    // Serial output indicating current LED states
    Serial.print("LED 14 is now: ");
    Serial.println(ledState ? "ON" : "OFF");
    Serial.print("LED 27 is now: ");
    Serial.println(ledState ? "ON" : "OFF");
    Serial.print("LED 35 is now: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Wait for 3 seconds (3000 ms)
    vTaskDelay(3000 / portTICK_PERIOD_MS); // Consistent 3-second delay
  }
}
