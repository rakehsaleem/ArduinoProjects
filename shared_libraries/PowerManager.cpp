/**
 * @file PowerManager.cpp
 * @author M. Rakeh Saleem
 * @brief Power management utilities implementation for ATtiny85
 * @date Created: 2020
 * @version 1.0
 */

#include "PowerManager.h"

bool PowerManager::sleepEnabled = false;
unsigned long PowerManager::wakeTime = 0;

void PowerManager::init() {
    // Configure all pins as inputs to minimize power consumption
    for (uint8_t i = 0; i < 5; i++) {
        pinMode(i, INPUT);
    }
    
    // Enable pin change interrupts
    GIMSK |= _BV(PCIE);
    sei();
}

void PowerManager::enterSleep() {
    sleepEnabled = true;
    wakeTime = millis();
    
    // Disable ADC
    disableADC();
    
    // Set sleep mode to power down
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    
    // Enable interrupts and sleep
    sei();
    sleep_cpu();
    
    // Wake up
    sleep_disable();
    enableADC();
}

void PowerManager::wakeUp() {
    sleepEnabled = false;
}

bool PowerManager::shouldStayAwake(unsigned long duration) {
    return (millis() - wakeTime) < duration;
}

void PowerManager::setupWakeInterrupt(uint8_t pin) {
    PCMSK |= _BV(pin);
}

void PowerManager::disableADC() {
    ADCSRA &= ~_BV(ADEN);
}

void PowerManager::enableADC() {
    ADCSRA |= _BV(ADEN);
}
