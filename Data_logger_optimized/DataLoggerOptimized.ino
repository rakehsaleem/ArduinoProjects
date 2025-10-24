/**
 * @file DataLoggerOptimized.ino
 * @author M. Rakeh Saleem
 * @brief Optimized drone data logging system
 * @date Created: Mar 2019 | Optimized: 2020
 * @version 2.0
 * 
 * Features:
 * - Modular sensor management
 * - Improved error handling
 * - Better file organization
 * - Enhanced logging capabilities
 * - Configuration management
 */

#include "SensorConfig.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SD_t3.h>
#include <TimeLib.h>
#include <LIDARLite.h>
#include <DS1307RTC.h>
#include "BN55.h"

// Sensor Manager Class
class SensorManager {
private:
    BN55* imu;
    LIDARLite* lidar;
    DS1307RTC* rtc;
    bool initialized;
    
public:
    SensorManager() : initialized(false) {
        imu = new BN55(BNO055_I2C_ADDR);
        lidar = new LIDARLite();
        rtc = new DS1307RTC();
    }
    
    bool init() {
        Wire.begin();
        
        if (!imu->init()) {
            Serial.println("IMU initialization failed");
            return false;
        }
        
        if (!lidar->begin(0, true)) {
            Serial.println("LIDAR initialization failed");
            return false;
        }
        
        lidar->configure(0);
        
        // Initialize RTC
        bool parse = false;
        bool config = false;
        
        if (getDate(__DATE__) && getTime(__TIME__)) {
            parse = true;
            if (rtc->write(tm)) {
                config = true;
            }
        }
        
        if (!parse || !config) {
            Serial.println("RTC configuration failed");
            return false;
        }
        
        initialized = true;
        return true;
    }
    
    struct SensorData {
        float distance;
        float yaw, roll, pitch;
        float temperature;
        String timestamp;
    };
    
    SensorData readSensors() {
        SensorData data;
        
        if (!initialized) {
            return data;
        }
        
        // Read IMU data
        imu->readEul();
        data.yaw = imu->euler.x;
        data.roll = imu->euler.y;
        data.pitch = imu->euler.z;
        
        // Read temperature
        imu->readTemp();
        data.temperature = imu->temp.c;
        
        // Read LIDAR distance
        data.distance = lidar->distance() / 100.0; // Convert to meters
        
        // Get timestamp
        tmElements_t tm;
        if (rtc->read(tm)) {
            data.timestamp = String(tm.Hour) + ":" + 
                           String(tm.Minute) + ":" + 
                           String(tm.Second);
        }
        
        return data;
    }
};

// Data Logger Class
class DataLogger {
private:
    File logFile;
    String fileName;
    int logNumber;
    
public:
    bool init() {
        const int chipSelect = BUILTIN_SDCARD;
        
        if (!SD.begin(chipSelect)) {
            Serial.println("SD card initialization failed");
            return false;
        }
        
        // Find next available log file
        logNumber = findNextLogNumber();
        fileName = "Log" + String(logNumber) + ".txt";
        
        logFile = SD.open(fileName, FILE_WRITE);
        if (!logFile) {
            Serial.println("Failed to create log file");
            return false;
        }
        
        // Write header
        logFile.println("No.\tDate\t\tTime\t\tDist(m)\t\tYaw\tRoll\tPitch\tTemp(C)");
        logFile.flush();
        
        Serial.println("Logging to: " + fileName);
        return true;
    }
    
    void logData(int count, const SensorManager::SensorData& data, 
                 const String& gpsLon, const String& gpsLat) {
        if (!logFile) return;
        
        String dataLine = String(count) + "\t" +
                         data.timestamp + "\t\t" +
                         String(data.distance, 2) + "\t\t" +
                         gpsLon + "\t" + gpsLat + "\t" +
                         String(data.yaw, 1) + "\t" +
                         String(data.roll, 1) + "\t" +
                         String(data.pitch, 1) + "\t" +
                         String(data.temperature, 1);
        
        logFile.println(dataLine);
        logFile.flush();
        
        Serial.println(dataLine);
    }
    
    void close() {
        if (logFile) {
            logFile.close();
        }
    }
    
private:
    int findNextLogNumber() {
        for (int i = 0; i < 1000; i++) {
            String testFile = "Log" + String(i) + ".txt";
            if (!SD.exists(testFile)) {
                return i;
            }
        }
        return 0;
    }
};

// Global objects
SensorManager sensorManager;
DataLogger dataLogger;

// Pin definitions
#define WHITE_LED PIN_WHITE_LED
#define YELLOW_LED PIN_YELLOW_LED
#define RED_LED PIN_RED_LED

// GPS data buffers
char gpsLon[11];
char gpsLat[10];

void setup() {
    Serial.begin(115200);
    
    // Wait for serial connection
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("Drone Data Logger v2.0");
    Serial.println("Press any key to continue...");
    while (Serial.read() != '/') {
        delay(10);
    }
    
    // Initialize sensors
    if (!sensorManager.init()) {
        Serial.println("Sensor initialization failed!");
        return;
    }
    
    // Initialize data logger
    if (!dataLogger.init()) {
        Serial.println("Data logger initialization failed!");
        return;
    }
    
    // Setup LED pins
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(WHITE_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(WHITE_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    
    Serial.println("Setup completed successfully!");
}

void loop() {
    Serial.println("Initializing camera...");
    
    // Camera control sequence
    digitalWrite(WHITE_LED, LOW);
    delay(1000);
    digitalWrite(RED_LED, LOW);
    delay(2000);
    
    // Log sensor data
    logSensorData();
    
    // Reset camera
    digitalWrite(WHITE_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
}

void logSensorData() {
    // Read GPS data if available
    String gpsLonStr = "0.0000000";
    String gpsLatStr = "0.0000000";
    
    if (Serial1.available()) {
        Serial1.readBytes(gpsLon, 11);
        Serial1.readBytes(gpsLat, 10);
        gpsLonStr = String(gpsLon);
        gpsLatStr = String(gpsLat);
    }
    
    // Log 10 data points
    for (int i = 1; i <= 10; i++) {
        SensorManager::SensorData data = sensorManager.readSensors();
        dataLogger.logData(i, data, gpsLonStr, gpsLatStr);
        delay(100);
    }
}

// Time parsing functions (from original code)
bool getTime(const char *str) {
    int Hour, Min, Sec;
    if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
    tm.Hour = Hour;
    tm.Minute = Min;
    tm.Second = Sec;
    return true;
}

bool getDate(const char *str) {
    char Month[12];
    int Day, Year;
    uint8_t monthIndex;
    
    if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
    for (monthIndex = 0; monthIndex < 12; monthIndex++) {
        if (strcmp(Month, MONTH_NAMES[monthIndex]) == 0) break;
    }
    if (monthIndex >= 12) return false;
    tm.Day = Day;
    tm.Month = monthIndex + 1;
    tm.Year = CalendarYrToTm(Year);
    return true;
}
