#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <cstdio>
#include "memory/memory.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "ble/ble.h"
#include "config/config.h"

String sessionFolder = "";  // Root session folder path

uint8_t eepromRead(uint8_t address)
{
    return EEPROM.read(address);
}
void eepromWrite(uint8_t address, uint8_t value)
{ 
    EEPROM.write(address, value);
    EEPROM.commit();
}
void initEEPROM()
{
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        while (true);  // Stop execution
    }
    byte value = eepromRead(deviceRoleAddress);
    Serial.print("Read byte from EEPROM: ");
    Serial.println(value, HEX);
}
void initSD(){

  Serial.print("Mounting SD Card... ");

  if (!SD.begin(SD_CS,SPI, 40000000)) {
    for(int i=0; i<5; i++){
        Serial.println("Error: Card not found or wiring incorrect.");
        delay(500);
    }
  }
  else
    Serial.println("Success.");
}

// Create timestamped session folder structure (e.g., /20260325_143022/)
void initSessionFolder() {
  // Generate timestamp: YYYYMMDD_HHMMSS
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char timestamp[16];
  strftime(timestamp, sizeof(timestamp), "/%Y%m%d_%H%M%S", timeinfo);
  
  sessionFolder = String(timestamp);
  
  Serial.println("Creating session folder: " + sessionFolder);
  
  // Create main session folder
  SD.mkdir(sessionFolder.c_str());
  
  // Create subfolders
  createSubfolder("sensor_data");
  createSubfolder("color_camera");
  createSubfolder("thermal_camera");
  createSubfolder("mic&accel");
  
  Serial.println("Session folder structure created.");
}

// Create a subfolder within the session folder
void createSubfolder(String folderName) {
  String fullPath = sessionFolder + "/" + folderName;
  if (!SD.exists(fullPath.c_str())) {
    SD.mkdir(fullPath.c_str());
    Serial.println("Created folder: " + fullPath);
  }
}

// Get the path for sensor data CSV (sensor_data/sensor.csv)
String getSensorDataPath() {
  return sessionFolder + "/sensor_data/sensor.csv";
}

// Get the path for color camera folder
String getColorCameraPath() {
  return sessionFolder + "/color_camera";
}

// Get the path for thermal camera folder
String getThermalCameraPath() {
  return sessionFolder + "/thermal_camera";
}

// Get the path for microphone & accelerometer CSV
String getMicAccelPath() {
  return sessionFolder + "/mic&accel/mic_accel.csv";
}



// Initialize the mic&accel CSV file with header
void openMicAccelFile() {
  String micAccelFile = getMicAccelPath();
  
  // Create header for column format
  String header = "timestamp_ms,mic,accelX,accelY,accelZ";
  
  File file = SD.open(micAccelFile.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open mic&accel file for writing");
    return;
  }
  if (file.println(header)) {
    Serial.println("Mic&Accel CSV header written to: " + micAccelFile);
  } else {
    Serial.println("Mic&Accel header write failed");
  }
  file.close();
}



// Log 2000 microphone & accelerometer samples to mic&accel CSV
// Column format: timestamp, mic, accelX, accelY, accelZ (2000 rows)
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount, uint32_t timestamp) {
  String micAccelFile = getMicAccelPath();
  
  if (sampleCount == 0) {
    Serial.println("No mic/accel samples to log");
    return;
  }
  
  // Ensure we don't exceed 2000 samples
  if (sampleCount > 2000) sampleCount = 2000;
  
  File file = SD.open(micAccelFile, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open mic&accel file for appending");
    return;
  }
  
  // Burst write: Build entire CSV in buffer first, then write once
  // Allocate buffer for all 2000 rows (~65 KB)
  size_t bufferSize = sampleCount * 50; // Conservative estimate: ~50 bytes per row with newline
  char* csvBuffer = (char*)malloc(bufferSize);
  
  if (!csvBuffer) {
    Serial.println("Failed to allocate buffer for burst write");
    file.close();
    return;
  }
  
  size_t offsetPos = 0;
  
  // Build entire CSV content in buffer
  for (uint16_t i = 0; i < sampleCount; i++) {
    int written = snprintf(csvBuffer + offsetPos, bufferSize - offsetPos, "%u,%u,%d,%d,%d\n",
      timestamp,
      mic[i],
      accelX[i],
      accelY[i],
      accelZ[i]
    );
    if (written > 0) {
      offsetPos += written;
    }
  }
  
  // Burst write entire buffer at once
  if (offsetPos > 0) {
    file.write((uint8_t*)csvBuffer, offsetPos);
  }
  
  free(csvBuffer);
  file.close();
  Serial.println("Mic/Accel samples logged (" + String(sampleCount) + " samples, burst write)");
}

// Initialize sensor data CSV file with comprehensive header
void initSensorDataFile() {
  String sensorFile = getSensorDataPath();
  
  String header = "timestamp_ms,battery_pct,ambLight_master,pir,mmwave_dist,mmwave_energy,static_dist,static_energy,detection_dist,ambLight_slave,humidity,temperature";
  
  File file = SD.open(sensorFile.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open sensor data file for writing");
    return;
  }
  if (file.println(header)) {
    Serial.println("Sensor data CSV header written to: " + sensorFile);
  } else {
    Serial.println("Sensor data header write failed");
  }
  file.close();
}

// Log sensor data (master and slave) to sensor CSV with timestamp
void logSensorData(uint32_t timestamp, float batteryPct, float ambLight, float pir, float mmWaveDist, float mmWaveEnergy, 
                   float staticDist, float staticEnergy, float detectionDist, float ambLight_slave, float humidity, float temperature) {
  String sensorFile = getSensorDataPath();
  
  String dataRow = String(timestamp) + "," +
                   String(batteryPct, 2) + "," +
                   String(ambLight, 2) + "," +
                   String(pir, 2) + "," +
                   String(mmWaveDist, 2) + "," +
                   String(mmWaveEnergy, 2) + "," +
                   String(staticDist, 2) + "," +
                   String(staticEnergy, 2) + "," +
                   String(detectionDist, 2) + "," +
                   String(ambLight_slave, 2) + "," +
                   String(humidity, 2) + "," +
                   String(temperature, 2);
  
  File file = SD.open(sensorFile, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open sensor data file for appending");
    return;
  }
  
  if (file.println(dataRow)) {
    Serial.printf("[SD] Sensor data logged: ts=%u ms\n", timestamp);
  } else {
    Serial.println("Sensor data write failed");
  }
  file.close();
}

// Save RGB565 image as raw binary file (.rgb565)
// Raw binary: 64x64 image with RGB565 pixel data (8192 bytes)
void saveRGBImage(uint16_t* rgbFrame, uint32_t timestamp) {
  if (rgbFrame == nullptr) {
    Serial.println("[RGB] No frame data to save");
    return;
  }
  
  // Create filename with timestamp
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/color_camera/%u.rgb565", sessionFolder.c_str(), timestamp);
  
  // RGB565 image: 64x64 pixels = 4096 pixels * 2 bytes = 8192 bytes
  const uint16_t width = 64;
  const uint16_t height = 64;
  const size_t dataSize = width * height * sizeof(uint16_t);
  
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[RGB] Failed to open file: %s\n", filename);
    return;
  }
  
  // Burst write: write all RGB565 data in one call (8KB raw binary)
  size_t bytesWritten = file.write((uint8_t*)rgbFrame, dataSize);
  file.close();
  
  Serial.printf("[RGB] Raw RGB565 saved: %s (%zu bytes, burst write)\n", filename, bytesWritten);
}

// Save IR thermal image as CSV (16x12 grid of decimal values)
void saveIRImage(uint16_t* irFrame, uint32_t timestamp) {
  if (irFrame == nullptr) {
    Serial.println("[IR] No frame data to save");
    return;
  }
  
  // Create filename with timestamp
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/thermal_camera/%u.csv", sessionFolder.c_str(), timestamp);
  
  // IR thermal image: 16x12 pixels
  const uint16_t width = 16;
  const uint16_t height = 12;
  
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[IR] Failed to open file: %s\n", filename);
    return;
  }
  
  // Write thermal data as 12 rows x 16 columns CSV with decimal values
  size_t bytesWritten = 0;
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      uint16_t value = irFrame[row * width + col];
      
      // Write decimal value
      char buffer[16];
      int len = snprintf(buffer, sizeof(buffer), "%u", value);
      bytesWritten += file.write((uint8_t*)buffer, len);
      
      // Add comma separator except for last column
      if (col < width - 1) {
        file.write(',');
        bytesWritten++;
      }
    }
    // Add newline at end of row
    file.println();
    bytesWritten += 2;  // \r\n
  }
  
  file.close();
  Serial.printf("[IR] Thermal CSV saved: %s (%zu bytes)\n", filename, bytesWritten);
}