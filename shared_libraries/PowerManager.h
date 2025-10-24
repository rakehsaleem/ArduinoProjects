/**
 * @file PowerManager.h
 * @author M. Rakeh Saleem
 * @brief Power management utilities for ATtiny85
 * @date Created: 2020
 * @version 1.0
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

class PowerManager {
private:
    static bool sleepEnabled;
    static unsigned long wakeTime;
    
public:
    /**
     * @brief Initialize power management
     */
    static void init();
    
    /**
     * @brief Enter power down sleep mode
     */
    static void enterSleep();
    
    /**
     * @brief Wake up from sleep
     */
    static void wakeUp();
    
    /**
     * @brief Check if system should stay awake
     * @param duration Duration to stay awake in milliseconds
     * @return true if should stay awake
     */
    static bool shouldStayAwake(unsigned long duration);
    
    /**
     * @brief Configure pin change interrupt
     * @param pin Pin to use for wake-up interrupt
     */
    static void setupWakeInterrupt(uint8_t pin);
    
    /**
     * @brief Disable ADC to save power
     */
    static void disableADC();
    
    /**
     * @brief Enable ADC
     */
    static void enableADC();
};

// Interrupt Service Routine
ISR(PCINT0_vect) {
    PowerManager::wakeUp();
}

#endif // POWER_MANAGER_H
