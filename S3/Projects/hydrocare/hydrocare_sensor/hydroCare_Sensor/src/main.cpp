#include <Arduino.h>
#include "communication/communication.h"
#include "esp_task_wdt.h"

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Initialize SPI
  initSPIComm();
  
  Serial.println("\n=== HYDROCARE SENSOR SLAVE - 256 BYTE DATA PACKETS ===");
  Serial.println("Task-based architecture: SPI + Data update\n");
  
  // Create SPI Communication Task (Core 1, High Priority)
  xTaskCreatePinnedToCore(
    spiCommunicationTask,
    "SPI Task",
    4096,
    NULL,
    3,
    &spiTaskHandle,
    1
  );
  
  // Create Data Update Task (Core 0, Medium Priority)
  xTaskCreatePinnedToCore(
    dataUpdateTask,
    "Data Task",
    4096,
    NULL,
    2,
    &dataTaskHandle,
    0
  );
  
  Serial.println("✓ FreeRTOS Tasks Created");
  Serial.println("  - SPI Communication Task (Core 1, Priority 3)");
  Serial.println("  - Data Update Task (Core 0, Priority 2)");
  Serial.println("\nSystem ready. Waiting for master...\n");
}

void loop() {
  // Main loop handled by FreeRTOS scheduler
  delay(1000);
}

