/**
 * @file I2CManager.h
 * @author M. Rakeh Saleem
 * @brief I2C communication manager for sensor control
 * @date Created: 2020
 * @version 1.0
 */

#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <Wire.h>
#include "SensorConfig.h"

class I2CManager {
public:
    /**
     * @brief Initialize I2C communication
     */
    static void init();
    
    /**
     * @brief Configure MAX5479 digital potentiometer
     * @param address I2C address of the potentiometer
     * @param waValue Value for WA register
     * @param wbValue Value for WB register
     * @return true if successful
     */
    static bool configureMAX5479(uint8_t address, uint8_t waValue, uint8_t wbValue);
    
    /**
     * @brief Configure DS1342 RTC
     * @param hour Hour value
     * @param minute Minute value
     * @param second Second value
     * @return true if successful
     */
    static bool configureDS1342(uint8_t hour, uint8_t minute, uint8_t second);
    
    /**
     * @brief Set DS1342 alarm
     * @param alarmNum Alarm number (1 or 2)
     * @param hour Hour value
     * @param minute Minute value
     * @param second Second value
     * @return true if successful
     */
    static bool setDS1342Alarm(uint8_t alarmNum, uint8_t hour, uint8_t minute, uint8_t second);
    
    /**
     * @brief Clear DS1342 status flags
     * @return true if successful
     */
    static bool clearDS1342Status();
    
private:
    /**
     * @brief Send I2C command
     * @param address Device address
     * @param cmd Command byte
     * @param data Data byte
     * @return true if successful
     */
    static bool sendCommand(uint8_t address, uint8_t cmd, uint8_t data);
};

#endif // I2C_MANAGER_H
