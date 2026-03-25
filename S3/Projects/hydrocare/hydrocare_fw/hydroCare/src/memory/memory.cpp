#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <cstdio>
#include "memory/memory.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "ble/ble.h"
#include "config/config.h"

String currentFileName = "/data.csv";
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

// Set the filename (now supports both legacy and new path structure)
void setFileName(String name) {
  if (!name.startsWith("/")) {
    // If using new session folder structure
    if (sessionFolder.length() > 0) {
      currentFileName = sessionFolder + "/" + name;
    } else {
      currentFileName = "/" + name;
    }
  } else {
    currentFileName = name;
  }
  if (!currentFileName.endsWith(".csv")) {
    currentFileName += ".csv";
  }
  Serial.println("Filename set to: " + currentFileName);
}

// Create the file and write the Header (e.g., Lux,PIR,Distance)
void openFileAndHeader(String header) {
  File file = SD.open(currentFileName, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.println(header)) {
    Serial.println("Header written to: " + currentFileName);
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Initialize the mic&accel CSV file with header
void openMicAccelFile() {
  String micAccelFile = getMicAccelPath();
  
  // Create header showing the data type and first few sample indices
  String header = "sample_type,val_1,val_2,val_3,val_4,val_5,...,val_1000";
  
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

// Append a row of data to the CSV
void logData(String data) {
  // FILE_APPEND is crucial so you don't overwrite previous data
  File file = SD.open(currentFileName, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  
  if (file.println(data)) {
    Serial.println("Data logged: " + data);
  } else {
    Serial.println("Append failed");
  }
  file.close(); // Always close after writing to prevent data corruption
}

// Log 1000 microphone & accelerometer samples to mic&accel CSV
// Pattern: Row 1 = mic, Row 2 = accelX, Row 3 = accelY, Row 4 = accelZ, repeat
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount) {
  String micAccelFile = getMicAccelPath();
  
  if (sampleCount == 0) {
    Serial.println("No mic/accel samples to log");
    return;
  }
  
  // Ensure we don't exceed 1000 samples
  if (sampleCount > 1000) sampleCount = 1000;
  
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