#include <Arduino.h>
#include "config/config.h"
#include "measurement/measurement.h"
#include "leds/leds.h"
#include "communication/communication.h"
#include "esp_task_wdt.h"

// FreeRTOS task handles (global so communication handler can signal them)
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t measurementTaskHandle = NULL;

// SPI Communication Task (runs continuously, polls master)
void spiCommunicationTask(void *pvParameters) {
  (void)pvParameters;
  
  // Remove this task from watchdog (SPI can block waiting for master)
  esp_task_wdt_delete(NULL);
  
  Serial.println("[SPI Task] Started - polling for master commands");
  
  while (1) {
    // Block until master initiates SPI transfer
    receiveCommand();
    
    // Small delay to prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  // Initialize peripherals
  initPins();
  initPeripherals();
  initCamera();
  powerLEDInit();
  initIMU();
  initIRTemp();
  initSPIComm();
  
  Serial.println("\n=== HYDROCARE SENSOR SLAVE v0.1 ===");
  Serial.println("Using FreeRTOS multi-task architecture");
  Serial.println("Initialized: Pins, Peripherals, Camera, LEDs, IMU, IR, SPI\n");

  // Create FreeRTOS tasks
  // Task 1: SPI Communication (high priority, continuous polling)
  xTaskCreatePinnedToCore(
    spiCommunicationTask,      // Task function
    "SPI Task",                 // Task name
    4096,                       // Stack size (bytes)
    NULL,                       // Parameters
    3,                          // Priority (higher = more priority)
    &spiTaskHandle,             // Task handle
    1                           // Core (0 or 1)
  );

  // Task 2: Measurement Collector (lower priority, event-driven)
  xTaskCreatePinnedToCore(
    measurementCollectorTask,   // Task function
    "Measurement Task",         // Task name
    16384,                      // Stack size (larger for sensor operations + thermal frame)
    NULL,                       // Parameters
    2,                          // Priority (lower than SPI)
    &measurementTaskHandle,     // Task handle
    0                           // Core (different from SPI task)
  );

  Serial.println("✓ FreeRTOS Tasks Created");
  Serial.println("  - SPI Communication Task (Core 1, Priority 3)");
  Serial.println("  - Measurement Collector Task (Core 0, Priority 2)");
  Serial.println("\nSystem ready. Waiting for master...\n");
}

void loop() {
  // Main loop: FreeRTOS scheduler handles everything
  // This function is now just idle time
  delay(1000);
}
