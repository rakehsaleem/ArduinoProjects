// Created by M. Rakeh Saleem
// Date created: --
// last modified: 4-Mar-2019
//
// Data logger with Analog readings and file handling 
/////////////////////////////////////////////////////

#include <SdFat.h>
#include <math.h>
#include <Wire.h>
const int chipSel = BUILTIN_SDCARD;

SdFat sd;
ofstream logfile;

float temperature;
float pressure;
float altBaro;
const float sea_press = 1013.3;

String bufferString = "";
String SDbufferString = "";

uint64_t Time = 0;
uint64_t timeOld = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  
//  I2C0_F = 0x1A;   //set the I2C speed to 400KHz for Teensy 3.1

  delay(100);
//  initBaro(ADDRESS);

  // initialize the SD card at SPI_FULL_SPEED
  if (!sd.begin(chipSel, SPI_FULL_SPEED )) sd.initErrorHalt();

  // Create a new file in the current working directory
  // and generate a new name if the file already exists
  char fileName[13] = "LOGGER00.CSV";

  for (uint8_t i = 0; i < 150; i++) 
  {
    fileName[6] = i/10 + '0';
    fileName[7] = i%10 + '0';
    if (sd.exists(fileName)) continue;
    logfile.open(fileName);
    break;
  }

  logfile << ("Time,Temperature,Pressure,AltitudeRaw") << flush;   //Write header to logfile
  timeOld = millis();
}

void loop()
{
  int Time = millis();
  int dt = Time - timeOld;
  timeOld = Time;

  int sensorValue = analogRead(A5);
  float voltage = sensorValue * (5.0 / 1023.0);
  Serial.println("Sensor value");
  Serial.println(voltage);

  // make a string for assembling the data to log:
  String dataString = "";

  dataString += String("Time,");
  dataString += String(int(Time));
  dataString += String(",");
  dataString += String("Sensor,");
  dataString += String(voltage);
  dataString += String("\n");  //Create a new line on the SD card

  bufferString += dataString;
  int length = bufferString.length();

  if (length > 512)
  {

    SDbufferString = bufferString.substring(0,511);    //Create a string with 512 bytes for writing on the SD card.
    bufferString = bufferString.substring(511,length); //Remove the 512 bytes for the SD card from the main string. 

    char charBuf[512];
    SDbufferString.toCharArray(charBuf,512);
    logfile << charBuf << flush;      //Write to SD Card  
  }

  Serial.print(int(dt));
  Serial.println(); 
}
