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

// Persistent file handles - kept open during 60-second rotation window
static File sensorFile;
static File micAccelFile;
static File rgbSessionFile;
static File irSessionFile;

// Rotation tracking
static uint32_t sessionStartTime = 0;      // milliseconds when logging started
static uint32_t lastRotationTime = 0;      // milliseconds of last file rotation
static const uint32_t ROTATION_INTERVAL = 60000;  // 60 seconds in milliseconds

// Generate timestamped filename (for rotation)
String generateRotatedFilename(String folder, String baseName, uint32_t timestamp) {
  time_t timeVal = timestamp / 1000;  // Convert ms to seconds
  struct tm* timeinfo = localtime(&timeVal);
  char filename[64];
  strftime(filename, sizeof(filename), ".csv", timeinfo);
  
  // Format: /sessionFolder/folder/basename_HHMMSS.csv
  char fullPath[128];
  snprintf(fullPath, sizeof(fullPath), "%s/%s/%s_%02d%02d%02d.csv", 
    sessionFolder.c_str(), 
    folder.c_str(),
    baseName.c_str(),
    timeinfo->tm_hour,
    timeinfo->tm_min,
    timeinfo->tm_sec);
  
  return String(fullPath);
}

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



// Initialize the mic&accel CSV file with header (called at session start and rotation)
void openMicAccelFile(uint32_t timestamp) {
  String micAccelFilePath = generateRotatedFilename("mic&accel", "micaccel", timestamp ? timestamp : millis());
  
  micAccelFile = SD.open(micAccelFilePath.c_str(), FILE_WRITE);
  if (!micAccelFile) {
    Serial.println("Failed to open mic&accel file for writing");
    return;
  }
  
  // Create header for column format
  String header = "timestamp_ms,mic,accelX,accelY,accelZ";
  
  if (micAccelFile.println(header)) {
    Serial.println("Mic&Accel CSV header written to: " + micAccelFilePath);
  } else {
    Serial.println("Mic&Accel header write failed");
  }
}



// Log 2000 microphone & accelerometer samples to persistent mic&accel CSV
// Column format: timestamp, mic, accelX, accelY, accelZ (2000 rows)
// OPTIMIZED: Use static buffer with chunked writes to avoid SD card stalls
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount, uint32_t timestamp) {
  if (!micAccelFile) {
    Serial.println("Mic/Accel file not open");
    return;
  }
  
  if (sampleCount == 0) {
    Serial.println("No mic/accel samples to log");
    return;
  }
  
  // Ensure we don't exceed 2000 samples
  if (sampleCount > 2000) sampleCount = 2000;
  
  // Use static buffer (100KB) - no malloc, no fragmentation!
  static char csvBuffer[102400];  // 100KB static buffer for 2000 samples
  size_t offsetPos = 0;
  
  // Build entire CSV content in buffer
  for (uint16_t i = 0; i < sampleCount; i++) {
    int written = snprintf(csvBuffer + offsetPos, sizeof(csvBuffer) - offsetPos, "%u,%u,%d,%d,%d\n",
      timestamp, mic[i], accelX[i], accelY[i], accelZ[i]);
    if (written > 0) {
      offsetPos += written;
    }
  }
  
  // Chunked write: Write in 10KB chunks to avoid SD card stalls
  const size_t CHUNK_SIZE = 10240;  // 10KB chunks
  size_t bytesWritten = 0;
  
  while (bytesWritten < offsetPos) {
    size_t chunkSize = (offsetPos - bytesWritten > CHUNK_SIZE) ? CHUNK_SIZE : (offsetPos - bytesWritten);
    micAccelFile.write((uint8_t*)(csvBuffer + bytesWritten), chunkSize);
    bytesWritten += chunkSize;
  }
  
  Serial.println("Mic/Accel samples logged (" + String(sampleCount) + " samples, " + String(offsetPos) + " bytes)");
}

// Initialize sensor data CSV file with comprehensive header (called at session start and rotation)
void openSensorDataFile(uint32_t timestamp) {
  String sensorFilePath = generateRotatedFilename("sensor_data", "sensor", timestamp ? timestamp : millis());
  
  sensorFile = SD.open(sensorFilePath.c_str(), FILE_WRITE);
  if (!sensorFile) {
    Serial.println("Failed to open sensor data file for writing");
    return;
  }
  
  String header = "timestamp_ms,battery_pct,ambLight_master,pir,mmwave_dist,mmwave_energy,static_dist,static_energy,detection_dist,ambLight_slave,humidity,temperature";
  
  if (sensorFile.println(header)) {
    Serial.println("Sensor data CSV header written to: " + sensorFilePath);
  } else {
    Serial.println("Sensor data header write failed");
  }
}

// Old function - kept for compatibility, redirects to new one
void initSensorDataFile() {
  openSensorDataFile();
}

// Open persistent RGB session file (called at session start and rotation)
void openRGBSessionFile(uint32_t timestamp) {
  String rgbFilePath = generateRotatedFilename("color_camera", "rgb", timestamp ? timestamp : millis());
  
  rgbSessionFile = SD.open(rgbFilePath.c_str(), FILE_WRITE);
  if (!rgbSessionFile) {
    Serial.println("Failed to open RGB session file");
    return;
  }
  
  Serial.println("RGB session file opened: " + rgbFilePath);
}

// Open persistent IR session file (called at session start and rotation)
void openIRSessionFile(uint32_t timestamp) {
  String irFilePath = generateRotatedFilename("thermal_camera", "ir", timestamp ? timestamp : millis());
  
  irSessionFile = SD.open(irFilePath.c_str(), FILE_WRITE);
  if (!irSessionFile) {
    Serial.println("Failed to open IR session file");
    return;
  }
  
  Serial.println("IR session file opened: " + irFilePath);
}

// Close all persistent files (called at session end)
void closeSessionFiles() {
  if (sensorFile) {
    sensorFile.close();
    Serial.println("Sensor file closed");
  }
  if (micAccelFile) {
    micAccelFile.close();
    Serial.println("Mic/Accel file closed");
  }
  if (rgbSessionFile) {
    rgbSessionFile.close();
    Serial.println("RGB file closed");
  }
  if (irSessionFile) {
    irSessionFile.close();
    Serial.println("IR file closed");
  }
}

// Check if 60 seconds have elapsed and rotate files if needed
void checkAndRotateFiles(uint32_t currentTime) {
  // Initialize on first call
  if (sessionStartTime == 0) {
    sessionStartTime = currentTime;
    lastRotationTime = currentTime;
    return;
  }
  
  // Check if 60 seconds have passed since last rotation
  if (currentTime - lastRotationTime >= ROTATION_INTERVAL) {
    Serial.printf("[ROTATION] 60 seconds elapsed, rotating all files at %u ms\n", currentTime);
    lastRotationTime = currentTime;
    
    // Close all current files
    if (sensorFile) sensorFile.close();
    if (micAccelFile) micAccelFile.close();
    if (rgbSessionFile) rgbSessionFile.close();
    if (irSessionFile) irSessionFile.close();
    
    // Reopen with new timestamps
    openSensorDataFile(currentTime);
    openMicAccelFile(currentTime);
    openRGBSessionFile(currentTime);
    openIRSessionFile(currentTime);
  }
}

// Log sensor data (master and slave) to persistent sensor CSV
void logSensorData(uint32_t timestamp, float batteryPct, float ambLight, float pir, float mmWaveDist, float mmWaveEnergy, 
                   float staticDist, float staticEnergy, float detectionDist, float ambLight_slave, float humidity, float temperature) {
  if (!sensorFile) {
    Serial.println("Sensor file not open");
    return;
  }
  
  sensorFile.printf("%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    timestamp, batteryPct, ambLight, pir, mmWaveDist, mmWaveEnergy, 
    staticDist, staticEnergy, detectionDist, ambLight_slave, humidity, temperature);
  
  Serial.printf("[SD] Sensor data logged: ts=%u ms\n", timestamp);
}

// Save RGB565 image as 2D grid in persistent CSV (called every cycle)
// OPTIMIZED: Buffer entire grid in RAM, write once (not 4096 printf calls)
// Format:
//   timestamp
//   row0: rgb[0,0], rgb[0,1], ..., rgb[0,63]
//   row1: rgb[1,0], rgb[1,1], ..., rgb[1,63]
//   ...
//   row63: rgb[63,0], rgb[63,1], ..., rgb[63,63]
void saveRGBImage(uint16_t* rgbFrame, uint32_t timestamp) {
  if (rgbFrame == nullptr) {
    Serial.println("[RGB] No frame data to save");
    return;
  }
  
  if (!rgbSessionFile) {
    Serial.println("[RGB] Session file not open");
    return;
  }
  
  // Allocate buffer for entire 64x64 grid (~25KB)
  // Format: timestamp(1 line) + 64 rows × 64 values (5 digits + comma) + newlines
  static char rgbBuffer[32768];  // 32KB static buffer
  int pos = 0;
  
  // Write timestamp header
  pos += snprintf(rgbBuffer + pos, sizeof(rgbBuffer) - pos, "%u\n", timestamp);
  
  // Write 64x64 grid (64 rows x 64 columns)
  const int width = 64;
  const int height = 64;
  
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int idx = row * width + col;
      if (col < width - 1) {
        pos += snprintf(rgbBuffer + pos, sizeof(rgbBuffer) - pos, "%u,", rgbFrame[idx]);
      } else {
        pos += snprintf(rgbBuffer + pos, sizeof(rgbBuffer) - pos, "%u\n", rgbFrame[idx]);
      }
    }
  }
  
  // Chunked write: Write in 10KB chunks to avoid SD card stalls
  const size_t CHUNK_SIZE = 10240;  // 10KB chunks
  size_t bytesWritten = 0;
  
  while (bytesWritten < pos) {
    size_t chunkSize = (pos - bytesWritten > CHUNK_SIZE) ? CHUNK_SIZE : (pos - bytesWritten);
    rgbSessionFile.write((uint8_t*)(rgbBuffer + bytesWritten), chunkSize);
    bytesWritten += chunkSize;
  }
  
  Serial.printf("[RGB] 64x64 grid saved (timestamp=%u, %d bytes)\n", timestamp, pos);
}

// Save IR thermal image as 2D grid in persistent CSV (called every cycle)
// OPTIMIZED: Buffer entire grid in RAM, write once (not 192 printf calls)
// Format:
//   timestamp
//   row0: ir[0,0], ir[0,1], ..., ir[0,15]
//   row1: ir[1,0], ir[1,1], ..., ir[1,15]
//   ...
//   row11: ir[11,0], ir[11,1], ..., ir[11,15]
void saveIRImage(uint16_t* irFrame, uint32_t timestamp) {
  if (irFrame == nullptr) {
    Serial.println("[IR] No frame data to save");
    return;
  }
  
  if (!irSessionFile) {
    Serial.println("[IR] Session file not open");
    return;
  }
  
  // Allocate buffer for entire 12x16 grid (~2KB)
  static char irBuffer[4096];  // 4KB static buffer
  int pos = 0;
  
  // Write timestamp header
  pos += snprintf(irBuffer + pos, sizeof(irBuffer) - pos, "%u\n", timestamp);
  
  // Write 12x16 grid (12 rows x 16 columns)
  const int width = 16;
  const int height = 12;
  
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int idx = row * width + col;
      if (col < width - 1) {
        pos += snprintf(irBuffer + pos, sizeof(irBuffer) - pos, "%u,", irFrame[idx]);
      } else {
        pos += snprintf(irBuffer + pos, sizeof(irBuffer) - pos, "%u\n", irFrame[idx]);
      }
    }
  }
  
  // Chunked write: Write in 10KB chunks to avoid SD card stalls
  const size_t CHUNK_SIZE = 10240;  // 10KB chunks
  size_t bytesWritten = 0;
  
  while (bytesWritten < pos) {
    size_t chunkSize = (pos - bytesWritten > CHUNK_SIZE) ? CHUNK_SIZE : (pos - bytesWritten);
    irSessionFile.write((uint8_t*)(irBuffer + bytesWritten), chunkSize);
    bytesWritten += chunkSize;
  }
  
  Serial.printf("[IR] 12x16 grid saved (timestamp=%u, %d bytes)\n", timestamp, pos);
}