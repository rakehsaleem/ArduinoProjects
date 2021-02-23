/*******************************************************************/
//  Created by M. Rakeh Saleem
//  Date created: 25-Sep-2019
//  Last modified: 13-May-2020
//////////////////////////////

#define MAX5479_ADD_U1 0x29
#define MAX5479_ADD_U2 0x28
#define DS1342_ADDR 0x68
#define C_BYTEWA 0X11
#define C_BYTEWB 0X12
#define Time_MemAdd 0x00
#define Alarm1_MemAdd 0x07
#define Alarm2_MemAdd 0x0B
#define Control_MemAdd 0x0E
#define Status_MemAdd 0x0F
#define ST_P PB3
#define LED PB4
#define BODS 7                            // BOD Sleep bit in MCUCR
#define BODSE 2                           // BOD Sleep enable bit in MCUCR
#define KEEP_RUNNING 3000                 // 3 seconds


#include <avr/io.h>
#include <avr/sleep.h>
#include <Wire.h>

const unsigned long starttime=0;

void setup()
{
  //i2c();                                  // I2C communication mode initialization function calls
  for (byte i=0; i<5; i++)
  {
    pinMode(i, INPUT);
    //digitalWrite(PB1, LOW);   
   }
   GIMSK |= _BV(ACIE);                    // Enable Pin Change Interrupts
   PCMSK1 |= _BV(PCINT1);                  // Use PB1 as interupt pin
   pinMode(LED, OUTPUT);
   digitalWrite(LED, HIGH);               // Drive it low so it doesn't source current
   delay(10);
   digitalWrite(LED, LOW);
   pinMode(ST_P, OUTPUT);
   digitalWrite(ST_P, LOW);               // Make ST_P high all time
   delay(3000);
}

void loop()
{
  i2c();
  delay(200);
  GoToSleep();                            // Sleep function calls
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
  GIMSK |= _BV(ACIE);                     // Enable Pin Change Interrupts
  PCMSK1 |= _BV(PCINT1);                   // Use PB0 as interrupt pin
  
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
  // MAX5479 digiPOT configuration for Strain
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U2);
  Wire.write(C_BYTEWA);
  Wire.write(0xff);                   // (0x80)send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  int Byte1 = Wire.endTransmission();
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U2);
  Wire.write(C_BYTEWB);
  Wire.write(0x80);                   //(26) send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  int Byte2 = Wire.endTransmission();

  // MAX5479 digiPOT configuration for External Sensors
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U1);
  Wire.write(C_BYTEWA);
  Wire.write(0x40);                   // (0x80)send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  int Byte3 = Wire.endTransmission();
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U1);
  Wire.write(C_BYTEWB);
  Wire.write(0x20);                   //(26) send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  int Byte4 = Wire.endTransmission();

  // DS 1342 RTC-Clock configuration
  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Control_MemAdd);         // Define Interrupt output(s)
  Wire.write(0x07);
  int Byte5 = Wire.endTransmission();
  
  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Status_MemAdd);          // clear Alarm1 and Alarm2 flag(s)
  Wire.write(0x00);
  int Byte12 = Wire.endTransmission(); 

  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Time_MemAdd);            // Load Initial Timer Values
  Wire.write(0x00);                   // Seconds set to '0'
  Wire.write(0x00);                   // Minutes set to '0'
  Wire.write(0x46);                   // Hours set to '6 A.M' with 12 hours format
  int Byte6 = Wire.endTransmission();

  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Alarm1_MemAdd);          // Alarm1 Values
  Wire.write(0x00);                   // Seconds set to '0' 0x00 
  Wire.write(0x01);                   // Minutes set to '5' 0x03
  Wire.write(0x46);                   // Minutes set to '6' 0x46
  Wire.write(0x80);                   // Alarm1 occurs when hours, minutes and seconds match
  int Byte7 = Wire.endTransmission();

  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Alarm2_MemAdd);          // Alarm2 Values
  Wire.write(0x04);                   // Minutes set to '5' 0x03
  Wire.write(0xE6);                   // Hours set to '6' 0x46
  Wire.write(0x80);                   // Alarm1 occurs when hours, minutes and seconds match
  int Byte8 = Wire.endTransmission();
  
//  TinyWireM.begin();
//  TinyWireM.beginTransmission(DS1342_ADDR);
//  TinyWireM.send(Control_MemAdd);         // Enable Alarm1 and Alarm2 Interrupt
//  TinyWireM.send(0x07);
//  int Byte9 = TinyWireM.endTransmission();
}

void i2cCF(void)
{
  Wire.begin();
  Wire.beginTransmission(DS1342_ADDR);
  Wire.write(Status_MemAdd);          // clear Alarm1 and Alarm2 flag(s)
  Wire.write(0x00);
  int Byte10 = Wire.endTransmission();

//  TinyWireM.begin();
//  TinyWireM.beginTransmission(DS1342_ADDR);
//  TinyWireM.send(Control_MemAdd);         // Enable Alarm1 and Alarm2 Interrupt
//  TinyWireM.send(0x05);
//  int Byte11 = TinyWireM.endTransmission();
}
