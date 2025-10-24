/**
 * @file hardware_config.h
 * @author M. Rakeh Saleem
 * @brief Hardware configuration file for different platforms
 * @date Created: 2020
 * @version 1.0
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// Platform Detection
#if defined(__AVR_ATtiny85__)
    #define PLATFORM_ATTINY85
#elif defined(TEENSYDUINO)
    #define PLATFORM_TEENSY
#elif defined(ARDUINO_ARCH_NRF52)
    #define PLATFORM_NRF52
#else
    #define PLATFORM_ARDUINO
#endif

// Pin Definitions by Platform
#ifdef PLATFORM_ATTINY85
    // ATtiny85 Pin Definitions
    #define PIN_LED PB4
    #define PIN_ST_P PB3
    #define PIN_EDS A6
    #define PIN_INTERRUPT PB1
    
    // ATtiny85 Specific Configuration
    #define I2C_SPEED 100000
    #define ADC_RESOLUTION 10
    
#elif defined(PLATFORM_TEENSY)
    // Teensy Pin Definitions
    #define PIN_WHITE_LED 35
    #define PIN_YELLOW_LED 36
    #define PIN_RED_LED 37
    #define PIN_BUILTIN_LED 13
    #define PIN_SD_CARD BUILTIN_SDCARD
    #define PIN_ADC_TRIGGER A9
    
    // Teensy Specific Configuration
    #define I2C_SPEED 400000
    #define ADC_RESOLUTION 16
    
#elif defined(PLATFORM_NRF52)
    // NRF52 Pin Definitions
    #define PIN_ADC_CONCRETE A5
    #define PIN_ADC_BATTERY A4
    
    // NRF52 Specific Configuration
    #define I2C_SPEED 400000
    #define ADC_RESOLUTION 14
    #define ADC_REFERENCE_VOLTAGE 3.0
    
#else
    // Arduino Pin Definitions
    #define PIN_BUILTIN_LED 13
    #define PIN_SDA A4
    #define PIN_SCL A5
    
    // Arduino Specific Configuration
    #define I2C_SPEED 100000
    #define ADC_RESOLUTION 10
#endif

// Common Pin Definitions
#define PIN_DHT11 A0
#define PIN_LORA_RESET 4
#define PIN_LORA_CS 10

// Hardware Feature Detection
#ifdef PLATFORM_TEENSY
    #define HAS_SD_CARD true
    #define HAS_MULTIPLE_SERIAL true
    #define HAS_HIGH_RES_ADC true
#else
    #define HAS_SD_CARD false
    #define HAS_MULTIPLE_SERIAL false
    #define HAS_HIGH_RES_ADC false
#endif

#ifdef PLATFORM_NRF52
    #define HAS_BLE true
    #define HAS_BEACON true
#else
    #define HAS_BLE false
    #define HAS_BEACON false
#endif

#ifdef PLATFORM_ATTINY85
    #define HAS_SLEEP_MODE true
    #define HAS_LOW_POWER true
#else
    #define HAS_SLEEP_MODE false
    #define HAS_LOW_POWER false
#endif

#endif // HARDWARE_CONFIG_H
