/**
 * @file SensorConfig.h
 * @author M. Rakeh Saleem
 * @brief Common sensor configurations and I2C addresses
 * @date Created: 2020
 * @version 1.0
 */

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

// I2C Addresses
#define BNO055_I2C_ADDR 0x28
#define DS1342_RTC_ADDR 0x68
#define DS1307_RTC_ADDR 0x68
#define MAX5479_ADDR_U1 0x29
#define MAX5479_ADDR_U2 0x28

// MAX5479 Command Bytes
#define MAX5479_CMD_WA 0x11
#define MAX5479_CMD_WB 0x12

// DS1342 Memory Addresses
#define DS1342_TIME_ADDR 0x00
#define DS1342_ALARM1_ADDR 0x07
#define DS1342_ALARM2_ADDR 0x0B
#define DS1342_CONTROL_ADDR 0x0E
#define DS1342_STATUS_ADDR 0x0F

// Pin Definitions (Teensy/Arduino)
#define PIN_WHITE_LED 35
#define PIN_YELLOW_LED 36
#define PIN_RED_LED 37
#define PIN_BUILTIN_LED 13

// Pin Definitions (ATtiny85)
#define AT85_ST_P PB3
#define AT85_LED PB4
#define AT85_EDS A6

// Timing Constants
#define LOG_INTERVAL_MS 20
#define SYNC_INTERVAL_MS 5
#define KEEP_RUNNING_MS 3000
#define SLEEP_WAKE_TIME_MS 300

// Month Names Array
const char* const MONTH_NAMES[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

// Sleep Mode Constants (ATtiny85)
#define BODS_BIT 7
#define BODSE_BIT 2

#endif // SENSOR_CONFIG_H
