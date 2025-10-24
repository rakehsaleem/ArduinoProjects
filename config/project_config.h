/**
 * @file project_config.h
 * @author M. Rakeh Saleem
 * @brief Project configuration file for Arduino projects
 * @date Created: 2020
 * @version 1.0
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// Project Information
#define PROJECT_NAME "Arduino Projects Collection"
#define PROJECT_VERSION "2.0"
#define AUTHOR "M. Rakeh Saleem"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// Debug Configuration
#define DEBUG_ENABLED true
#define SERIAL_BAUD_RATE 115200

// Timing Configuration
#define DEFAULT_DELAY_MS 100
#define SENSOR_READ_INTERVAL_MS 1000
#define LOG_INTERVAL_MS 20
#define SLEEP_TIMEOUT_MS 30000

// Error Codes
#define ERROR_NONE 0
#define ERROR_SENSOR_INIT 1
#define ERROR_I2C_COMM 2
#define ERROR_SD_CARD 3
#define ERROR_LORA_COMM 4
#define ERROR_RTC_CONFIG 5

// Status LED Configuration
#define LED_BLINK_FAST_MS 100
#define LED_BLINK_SLOW_MS 500
#define LED_BLINK_ERROR_MS 200

// File System Configuration
#define MAX_LOG_FILES 1000
#define LOG_FILE_PREFIX "Log"
#define LOG_FILE_EXTENSION ".txt"

// Communication Configuration
#define LORA_FREQUENCY 868.0
#define LORA_TX_POWER 13
#define LORA_TIMEOUT_MS 3000

// Sensor Configuration
#define SENSOR_RETRY_COUNT 3
#define SENSOR_TIMEOUT_MS 1000

#endif // PROJECT_CONFIG_H
