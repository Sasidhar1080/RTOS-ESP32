#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Define the LED pins
#define LED_PIN_1 14
#define LED_PIN_2 27

// Semaphore to control task execution order
SemaphoreHandle_t xSerialSemaphore;

// Function prototypes for the LED tasks
void vBlinkLedTask1(void *pvParameters);
void vBlinkLedTask2(void *pvParameters);

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Create the semaphore for task synchronization
  xSerialSemaphore = xSemaphoreCreateBinary();

  // Initialize the LED pins as outputs
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  // Create the FreeRTOS tasks for blinking the LEDs
  xTaskCreate(vBlinkLedTask1, "BlinkLed1", 2048, NULL, 1, NULL);
  xTaskCreate(vBlinkLedTask2, "BlinkLed2", 2048, NULL, 1, NULL);

  // Give the semaphore to start the first task
  xSemaphoreGive(xSerialSemaphore);
}

void loop() {
  // FreeRTOS scheduler handles task execution
  // No need for code in the loop function
}

// Task to blink the LED on pin 14
void vBlinkLedTask1(void *pvParameters) {
  // Wait for the semaphore to proceed
  xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);

  Serial.println("Task 1 (LED 14) starting...");

  // Toggle the LED
  digitalWrite(LED_PIN_1, HIGH); // Turn it on
  vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait for 3 seconds
  digitalWrite(LED_PIN_1, LOW); // Turn it off

  // Signal the next task to proceed
  xSemaphoreGive(xSerialSemaphore);

  Serial.println("Task 1 (LED 14) completed.");

  // Delete the task after execution
  vTaskDelete(NULL);
}

// Task to blink the LED on pin 27
void vBlinkLedTask2(void *pvParameters) {
  // Wait for the semaphore to proceed
  xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);

  Serial.println("Task 2 (LED 27) starting...");

  // Toggle the LED
  digitalWrite(LED_PIN_2, HIGH); // Turn it on
  vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait for 3 seconds
  digitalWrite(LED_PIN_2, LOW); // Turn it off

  // Signal the task completion
  xSemaphoreGive(xSerialSemaphore);

  Serial.println("Task 2 (LED 27) completed.");

  // Delete the task after execution
  vTaskDelete(NULL);
}
