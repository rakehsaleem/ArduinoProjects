// Created by M. Rakeh Saleem
// Date created: 12-May-2018
// last modified: 13-Aug-2020
//
// Serial communication for receiving lon. & lat. cooridnates 
/////////////////////////////////////////////////////////////

#define I2C_SLAVE_ADDRESS 0x28
#define C_BYTEWA 0X11
#define C_BYTEWB 0X12
#define ST_P PB3
#define LED PB4
#define KEEP_RUNNING 60000                           //milliseconds
#define BODS 7                            //BOD Sleep bit in MCUCR
#define BODSE 2                           //BOD Sleep enable bit in MCUCR

#include <avr/io.h>
#include <avr/sleep.h>
#include <Wire.h>
const unsigned long starttime=0;

void setup()
{
  Wire.begin();                      // Initialize the i2c master library
  i2c();                                  // function calls
  for (byte i=0; i<5; i++)
  {
    pinMode(i, INPUT);
   }
   GIMSK |= _BV(PCIE);                    // Enable Pin Change Interrupts
   PCMSK |= _BV(PCINT1);                  // Use PB0 as interupt pin
   pinMode(LED, OUTPUT);
   digitalWrite(LED, HIGH);               // Drive it low so it doesn't source current
   delay(10);
   digitalWrite(LED, LOW);
   pinMode(ST_P, OUTPUT);
   digitalWrite(ST_P, HIGH);              // Make ST_P high all time
}
void loop()
{
  //GoToSleep();                                // Sleep function calls
  do
  {
    digitalWrite(LED, HIGH);
    delay(KEEP_RUNNING);
  }
  while(millis()-starttime<KEEP_RUNNING); // keep system awake for 'Preset time' before it goes back to sleep
  digitalWrite(LED, LOW);
}
void GoToSleep(void)
{
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT1);                   // Use PB0 as interrupt pin
  
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();                         // Sets the Sleep Enable bit in MCUCR Register
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep
  sleep_disable();                        // Clearthe Sleep Enable bit in MCUCR Register
  ADCSRA |= _BV(ADEN);                    // ADC on
  sei(); 
}
ISR(PCINT0_vect)                          // Interrupt service routine to execute specific task during interrupt
{
  //GIMSK = 0;                            // disable external interrupts (only need one to wake up)
}
void i2c(void)
{
  Wire.begin();
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(C_BYTEWA);
  Wire.write(0xff);                   // send max resistance value (max tap)
  int Byte1 = Wire.endTransmission();
  Wire.begin();
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(C_BYTEWB);
  Wire.write(0xff);                   // send max resistance value (max tap)
  int Byte2 = Wire.endTransmission();
}
