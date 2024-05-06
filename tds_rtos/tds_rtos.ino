#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define the pin where the TDS sensor is connected
#define TDS_SENSOR_PIN 35 // Change to your ESP32's analog input pin

// Reference voltage and calibration parameters for TDS calculation
#define V_REF 3.3
#define ADC_RESOLUTION 4096 // For ESP32 with 12-bit ADC
#define CALIBRATION_FACTOR 0.5

// Function prototype for the TDS reading task
void vReadTDSTask(void *pvParameters);

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize the TDS sensor pin as input
  pinMode(TDS_SENSOR_PIN, INPUT);

  // Create the FreeRTOS task for reading TDS
  xTaskCreate(vReadTDSTask, "ReadTDS", 2048, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS scheduler manages task execution
}

// FreeRTOS task to read TDS sensor
void vReadTDSTask(void *pvParameters) {
  for (;;) { // Infinite loop for the task
    // Read the analog value from the TDS sensor
    int sensorValue = analogRead(TDS_SENSOR_PIN);
    
    // Convert the analog value to voltage
    float voltage = (sensorValue * V_REF) / ADC_RESOLUTION;

    // Calculate the TDS value (with calibration)
    float tdsValue = (133.42 * voltage * voltage * voltage - 
                      255.86 * voltage * voltage + 
                      857.39 * voltage) * CALIBRATION_FACTOR;

    // Output the TDS value to the serial monitor
    Serial.print("TDS Value: ");
    Serial.print(tdsValue);
    Serial.println(" ppm");

    // Delay before the next reading (e.g., every 2 seconds)
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
