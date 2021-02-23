// Cretaed by M. Rakeh Saleem
// Date created: --
// last modified: 24-Dec-2018
//
// Debugging of I2C and SPI comm. simultaneously using Attiny85 
///////////////////////////////////////////////////////////////

#include <tinySPI.h>
#include <TinyWireM.h>

const int LATCH_PIN = 0;
const int DATA_PIN = 1;
const int CLOCK_PIN = 2;

void setup()
{
  TinyWireM.begin();
  
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, HIGH);
}

void loop()
{
  setValue(0xff);
  delay(500);
  setValue(0x00);
  delay(500);
  TinyWireM.beginTransmission(0x24);
  TinyWireM.send(0xff);
  int Byte1 = TinyWireM.endTransmission();
  delay(2000);
}

void setValue(int value)
{
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, value);
  digitalWrite(LATCH_PIN, HIGH);
}
