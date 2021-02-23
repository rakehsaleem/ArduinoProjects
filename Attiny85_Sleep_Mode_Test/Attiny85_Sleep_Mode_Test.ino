// Cretaed by M. Rakeh Saleem
// Date created: --
// last modified: 13-Aug-2020
//
// Attiny85 ultra-low-power sleep configuration 
///////////////////////////////////////////////

#define ST_P A3
#define EDS A6
#define BODS 7                            // BOD Sleep bit in MCUCR
#define BODSE 2                           // BOD Sleep enable bit in MCUCR
#define KEEP_RUNNING 300                  // 3 seconds

#include <avr/io.h>
#include <avr/sleep.h>

const unsigned long starttime=0;

void setup()
{
  for (byte i=0; i<5; i++)
  {
    pinMode(i, INPUT);
    digitalWrite(A4, LOW);   
   }
   GIMSK |= _BV(PCIE0);                   // Enable Pin Change Interrupts
   PCMSK1 |= _BV(PCINT11);                // Use PB1 as interupt pin
   pinMode(EDS, OUTPUT);
   digitalWrite(EDS, HIGH);               // Drive it low so it doesn't source current
   delay(300);
   digitalWrite(EDS, LOW);
   pinMode(ST_P, OUTPUT);
   digitalWrite(ST_P, LOW);               // Make ST_P high all time
   delay(3000);
}

void loop()
{
  GoToSleep();                            // Sleep function calls
  do
  {
    digitalWrite(EDS, HIGH);
    delay(KEEP_RUNNING);
  }
  while(millis()-starttime<KEEP_RUNNING); // keep system awake for 'Preset time' before it goes back to sleep
  digitalWrite(EDS, LOW);
}
void GoToSleep(void)
{
  GIMSK |= _BV(PCIE0);                    // Enable Pin Change Interrupts
  PCMSK1 |= _BV(PCINT11);                 // Use PA1 as interrupt pin
  
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
