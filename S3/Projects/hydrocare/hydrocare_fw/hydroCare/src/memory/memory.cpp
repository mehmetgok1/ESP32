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

  if (!SD.begin(SD_CS)) {
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
  
  // Create header showing the data type and first few sample indices
  String header = "sample_type,val_1,val_2,val_3,val_4,val_5,...,val_2000";
  
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
// Pattern: Row 1 = mic, Row 2 = accelX, Row 3 = accelY, Row 4 = accelZ, repeat
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount) {
  String micAccelFile = getMicAccelPath();
  
  if (sampleCount == 0) {
    Serial.println("No mic/accel samples to log");
    return;
  }
  
  // Ensure we don't exceed 2000 samples
  if (sampleCount > 2000) sampleCount = 2000;
  
  // Log mic samples (Row 1)
  String micRow = "mic,";
  for (uint16_t i = 0; i < sampleCount; i++) {
    micRow += String(mic[i]);
    if (i < sampleCount - 1) micRow += ",";
  }
  
  // Log accelX samples (Row 2)
  String accelXRow = "accelX,";
  for (uint16_t i = 0; i < sampleCount; i++) {
    accelXRow += String(accelX[i]);
    if (i < sampleCount - 1) accelXRow += ",";
  }
  
  // Log accelY samples (Row 3)
  String accelYRow = "accelY,";
  for (uint16_t i = 0; i < sampleCount; i++) {
    accelYRow += String(accelY[i]);
    if (i < sampleCount - 1) accelYRow += ",";
  }
  
  // Log accelZ samples (Row 4)
  String accelZRow = "accelZ,";
  for (uint16_t i = 0; i < sampleCount; i++) {
    accelZRow += String(accelZ[i]);
    if (i < sampleCount - 1) accelZRow += ",";
  }
  
  // Write all 4 rows to file
  File file = SD.open(micAccelFile, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open mic&accel file for appending");
    return;
  }
  
  if (file.println(micRow) && file.println(accelXRow) && file.println(accelYRow) && file.println(accelZRow)) {
    Serial.println("Mic/Accel samples logged (" + String(sampleCount) + " samples)");
  } else {
    Serial.println("Mic/Accel write failed");
  }
  
  file.close();
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

// Convert RGB565/BGR565 to RGB888 exactly like OpenCV's COLOR_BGR5652BGR
// Takes 2 raw bytes (little-endian) and converts to R, G, B
void rgb565ToRgb888(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
  // Interpret as BGR565 (BBBBB GGGGGG RRRRR) in little-endian
  // This matches OpenCV's COLOR_BGR5652BGR conversion
  uint8_t r5 = (rgb565 & 0x1F);           // Bits 0-4: Red (5 bits)
  uint8_t g6 = ((rgb565 >> 5) & 0x3F);   // Bits 5-10: Green (6 bits)
  uint8_t b5 = ((rgb565 >> 11) & 0x1F);  // Bits 11-15: Blue (5 bits)
  
  // Scale from their bit-width to 8-bit
  *r = (r5 << 3) | (r5 >> 2);     // 5-bit → 8-bit (shift left 3, add top 2 bits)
  *g = (g6 << 2) | (g6 >> 4);     // 6-bit → 8-bit (shift left 2, add top 2 bits)
  *b = (b5 << 3) | (b5 >> 2);     // 5-bit → 8-bit (shift left 3, add top 2 bits)
}

// Save RGB565 image as PPM file (directly openable)
// PPM: 64x64 image with RGB888 pixel data (~12.3 KB)
void saveRGBImage(uint16_t* rgbFrame, uint32_t timestamp) {
  if (rgbFrame == nullptr) {
    Serial.println("[RGB] No frame data to save");
    return;
  }
  
  // Create filename with timestamp
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/color_camera/%u.ppm", sessionFolder.c_str(), timestamp);
  
  // PPM image: 64x64 pixels
  const uint16_t width = 64;
  const uint16_t height = 64;
  
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[RGB] Failed to open file: %s\n", filename);
    return;
  }
  
  // Write PPM header
  // P6 = binary RGB format
  file.println("P6");
  file.printf("%d %d\n", width, height);
  file.println("255");  // Max color value
  
  uint32_t fileSize = 0;
  // Write RGB888 pixel data (3 bytes per pixel)
  for (int i = 0; i < width * height; i++) {
    uint16_t rgb565 = rgbFrame[i];
    uint8_t r, g, b;
    rgb565ToRgb888(rgb565, &r, &g, &b);
    
    file.write(r);
    file.write(g);
    file.write(b);
    fileSize += 3;
  }
  
  file.close();
  // PPM header is ~21 bytes + 12288 bytes data = ~12309 bytes
  Serial.printf("[RGB] PPM image saved: %s (%u bytes)\n", filename, fileSize + 21);
}

// Save IR thermal image as raw binary
// IR thermal: 16x12 pixels (192 pixels * 2 bytes = 384 bytes)

// Thermal colormap: maps 8-bit value (0-255) to RGB
// Blue (cold) → Cyan → Green → Yellow → Red (hot)
void getThermalColor(uint8_t value, uint8_t* r, uint8_t* g, uint8_t* b) {
  // Simple thermal colormap with 5 keypoints
  if (value < 64) {
    // Blue to Cyan (0-64)
    *r = 0;
    *g = (value * 4);  // 0 to 255
    *b = 255;
  } else if (value < 128) {
    // Cyan to Green (64-128)
    *r = 0;
    *g = 255;
    *b = (255 - (value - 64) * 4);  // 255 to 0
  } else if (value < 192) {
    // Green to Yellow (128-192)
    *r = (value - 128) * 4;  // 0 to 255
    *g = 255;
    *b = 0;
  } else {
    // Yellow to Red (192-255)
    *r = 255;
    *g = (255 - (value - 192) * 4);  // 255 to 0
    *b = 0;
  }
}

void saveIRImage(uint16_t* irFrame, uint32_t timestamp) {
  if (irFrame == nullptr) {
    Serial.println("[IR] No frame data to save");
    return;
  }
  
  // Create filename with timestamp
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/thermal_camera/%u.ppm", sessionFolder.c_str(), timestamp);
  
  // IR thermal image: 16x12 pixels
  const uint16_t width = 16;
  const uint16_t height = 12;
  
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[IR] Failed to open file: %s\n", filename);
    return;
  }
  
  // Find min and max for normalization (critical for 16-bit data)
  uint16_t minVal = 65535, maxVal = 0;
  for (int i = 0; i < width * height; i++) {
    if (irFrame[i] < minVal) minVal = irFrame[i];
    if (irFrame[i] > maxVal) maxVal = irFrame[i];
  }
  if (maxVal == minVal) maxVal = minVal + 1;  // Avoid division by zero
  
  // Write PPM header
  // P6 = binary RGB format
  file.println("P6");
  file.printf("%d %d\n", width, height);
  file.println("255");  // Max color value
  
  uint32_t fileSize = 0;
  // Write thermal-colorized RGB888 pixel data (3 bytes per pixel)
  for (int i = 0; i < width * height; i++) {
    // Normalize 16-bit value to 8-bit (0-255)
    uint16_t ir16 = irFrame[i];
    uint8_t normalized = (uint8_t)(((ir16 - minVal) * 255) / (maxVal - minVal));
    
    // Apply thermal colormap
    uint8_t r, g, b;
    getThermalColor(normalized, &r, &g, &b);
    
    file.write(r);
    file.write(g);
    file.write(b);
    fileSize += 3;
  }
  
  file.close();
  // PPM header is ~23 bytes + 576 bytes data = ~599 bytes
  Serial.printf("[IR] Thermal PPM saved: %s (%u bytes)\n", filename, fileSize + 23);
}