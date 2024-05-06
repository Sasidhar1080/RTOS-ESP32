#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin where the DS18B20 sensor is connected
#define ONE_WIRE_BUS 4  // GPIO pin for OneWire bus

// Create a OneWire instance and DallasTemperature instance
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Function prototype for the temperature reading task
void vTemperatureReadingTask(void *pvParameters);

void setup() {
  // Start serial communication for debugging or monitoring
  Serial.begin(115200);
  
  // Start the temperature sensor
  sensors.begin();

  // Create the FreeRTOS task for reading temperature
  xTaskCreate(vTemperatureReadingTask, "TempRead", 2048, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS scheduler manages task execution
}

// FreeRTOS task to read temperature from DS18B20 sensor
void vTemperatureReadingTask(void *pvParameters) {
  for (;;) {  // Infinite loop for the task
    // Request temperatures from the sensor
    sensors.requestTemperatures();
    
    // Get temperature in Celsius
    float tempC = sensors.getTempCByIndex(0);
    
    if (tempC == DEVICE_DISCONNECTED_C) { // Check for sensor errors
      Serial.println("Error: Sensor not found or disconnected.");
    } else {
      // Output the temperature reading to the serial monitor
      Serial.print("Temperature: ");
      Serial.print(tempC);
      Serial.println(" °C");
    }

    // Delay for a set period (5 seconds)
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Adjust interval as needed
  }
}
