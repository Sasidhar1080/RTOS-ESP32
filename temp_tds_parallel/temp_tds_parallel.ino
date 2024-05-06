#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Define the pins for the sensors
#define TDS_SENSOR_PIN 36 // Analog pin for TDS sensor
#define TEMP_SENSOR_PIN 4 // GPIO pin for OneWire (DS18B20)

// Constants for TDS calculation
#define V_REF 3.3
#define ADC_RESOLUTION 4096
#define CALIBRATION_FACTOR 0.5

// Create OneWire and DallasTemperature instances for temperature sensor
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Function prototypes for the FreeRTOS tasks
void vReadTDSTask(void *pvParameters);
void vReadTempTask(void *pvParameters);

void setup() {
  // Start serial communication for debugging and monitoring
  Serial.begin(115200);

  // Initialize the temperature sensor
  tempSensor.begin();

  // Create the FreeRTOS tasks for reading TDS and temperature
  xTaskCreate(vReadTDSTask, "ReadTDS", 2048, NULL, 1, NULL);
  xTaskCreate(vReadTempTask, "ReadTemp", 2048, NULL, 1, NULL);
}

void loop() {
  // The FreeRTOS scheduler manages task execution
}

// FreeRTOS task to read from the TDS sensor
void vReadTDSTask(void *pvParameters) {
  for (;;) {
    // Read the analog value from the TDS sensor
    int sensorValue = analogRead(TDS_SENSOR_PIN);
    
    // Convert the analog value to voltage
    float voltage = (sensorValue * V_REF) / ADC_RESOLUTION;

    // Calculate the TDS value
    float tdsValue = (133.42 * voltage * voltage * voltage - 
                      255.86 * voltage * voltage + 
                      857.39 * voltage) * CALIBRATION_FACTOR;

    // Output the TDS value to the serial monitor
    Serial.print("TDS Value: ");
    Serial.print(tdsValue);
    Serial.println(" ppm");

    // Delay before the next reading (2 seconds)
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// FreeRTOS task to read from the DS18B20 temperature sensor
void vReadTempTask(void *pvParameters) {
  for (;;) {
    // Request temperatures from the DS18B20 sensor
    tempSensor.requestTemperatures();

    // Get the temperature in Celsius
    float tempC = tempSensor.getTempCByIndex(0);

    // Output the temperature reading to the serial monitor
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");

    // Delay before the next reading (2 seconds)
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
