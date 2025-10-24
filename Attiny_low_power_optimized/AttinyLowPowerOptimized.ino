/**
 * @file AttinyLowPowerOptimized.ino
 * @author M. Rakeh Saleem
 * @brief Optimized ultra-low power ATtiny85 system
 * @date Created: Sep 2019 | Optimized: 2020
 * @version 2.0
 * 
 * Features:
 * - Ultra-low power consumption with sleep modes
 * - MAX5479 digital potentiometer control
 * - DS1342 RTC with alarm functionality
 * - Modular design with shared libraries
 * - Improved error handling and logging
 */

#include "SensorConfig.h"
#include "PowerManager.h"
#include "I2CManager.h"

// Pin definitions
#define ST_P AT85_ST_P
#define LED AT85_LED

// Timing constants
#define KEEP_RUNNING KEEP_RUNNING_MS

// Global variables
const unsigned long startTime = 0;

void setup() {
    // Initialize power management
    PowerManager::init();
    
    // Initialize I2C communication
    I2CManager::init();
    
    // Configure sensors
    configureSensors();
    
    // Setup LED and strain power pins
    pinMode(LED, OUTPUT);
    pinMode(ST_P, OUTPUT);
    
    // Initial LED sequence
    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
    digitalWrite(ST_P, LOW);
    
    delay(3000);
}

void loop() {
    // Configure I2C devices
    configureI2CDevices();
    
    delay(200);
    
    // Enter sleep mode
    PowerManager::enterSleep();
    
    // Stay awake for specified duration
    while (PowerManager::shouldStayAwake(KEEP_RUNNING)) {
        digitalWrite(LED, HIGH);
        delay(KEEP_RUNNING);
    }
    
    digitalWrite(LED, LOW);
}

void configureSensors() {
    // Configure MAX5479 potentiometers
    I2CManager::configureMAX5479(MAX5479_ADDR_U2, 0xFF, 0x80); // Strain gauge
    I2CManager::configureMAX5479(MAX5479_ADDR_U1, 0x40, 0x20); // External sensors
    
    // Configure DS1342 RTC
    I2CManager::configureDS1342(0x46, 0x00, 0x00); // 6 AM
    
    // Set alarms
    I2CManager::setDS1342Alarm(1, 0x46, 0x01, 0x00); // Alarm 1: 6:01 AM
    I2CManager::setDS1342Alarm(2, 0xE6, 0x04, 0x00); // Alarm 2: 6:04 PM
    
    // Clear status flags
    I2CManager::clearDS1342Status();
}

void configureI2CDevices() {
    // Reconfigure devices periodically
    configureSensors();
}
