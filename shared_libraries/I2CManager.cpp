/**
 * @file I2CManager.cpp
 * @author M. Rakeh Saleem
 * @brief I2C communication manager implementation
 * @date Created: 2020
 * @version 1.0
 */

#include "I2CManager.h"

void I2CManager::init() {
    Wire.begin();
}

bool I2CManager::configureMAX5479(uint8_t address, uint8_t waValue, uint8_t wbValue) {
    // Configure WA register
    if (!sendCommand(address, MAX5479_CMD_WA, waValue)) {
        return false;
    }
    
    // Configure WB register
    if (!sendCommand(address, MAX5479_CMD_WB, wbValue)) {
        return false;
    }
    
    return true;
}

bool I2CManager::configureDS1342(uint8_t hour, uint8_t minute, uint8_t second) {
    Wire.beginTransmission(DS1342_RTC_ADDR);
    Wire.write(DS1342_TIME_ADDR);
    Wire.write(second);
    Wire.write(minute);
    Wire.write(hour);
    return Wire.endTransmission() == 0;
}

bool I2CManager::setDS1342Alarm(uint8_t alarmNum, uint8_t hour, uint8_t minute, uint8_t second) {
    uint8_t alarmAddr = (alarmNum == 1) ? DS1342_ALARM1_ADDR : DS1342_ALARM2_ADDR;
    
    Wire.beginTransmission(DS1342_RTC_ADDR);
    Wire.write(alarmAddr);
    Wire.write(second);
    Wire.write(minute);
    Wire.write(hour);
    Wire.write(0x80); // Alarm enable flag
    return Wire.endTransmission() == 0;
}

bool I2CManager::clearDS1342Status() {
    Wire.beginTransmission(DS1342_RTC_ADDR);
    Wire.write(DS1342_STATUS_ADDR);
    Wire.write(0x00);
    return Wire.endTransmission() == 0;
}

bool I2CManager::sendCommand(uint8_t address, uint8_t cmd, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(cmd);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}
