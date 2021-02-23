// Created by M. Rakeh Saleem
// Date created: 11-Sep-2018
// last modified: 11-Sep-2018
//
// Max. Digital potentiometer programming and debugging 
///////////////////////////////////////////////////////

#define MAX5479_ADD_U1 0x29
#define MAX5479_ADD_U2 0x28
#define C_BYTEWA 0X11
#define C_BYTEWB 0X12

#include <Wire.h>
#include <USIWire.h>
#include <WireS.h>

void setup()
{
  Wire.begin();
}

void loop()
{
  i2c();
  delay(200);
}

void i2c(void)
{
  // MAX5479 digiPOT configuration for Strain
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U2);
  Wire.write(C_BYTEWA);
  Wire.write(0xff);                   // (0x80)send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  Wire.endTransmission();
  Wire.begin();
  Wire.beginTransmission(MAX5479_ADD_U2);
  Wire.write(C_BYTEWB);
  Wire.write(0x80);                   //(26) send (0xff=full, 0x80=half, 0x40=one-fourth, 0x20=one-eighth) resistance value
  int Byte2 = Wire.endTransmission();

  delay(500);
  
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
}
