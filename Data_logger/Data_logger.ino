/*******************************************************************/
//  created by M. Rakeh Saleem
//  Date created: 10-Mar-2019
//  last modified: 25-jun-2019
//  Drone Data Logger

/*******************************************************************/
/******************  Include required Libraries  *******************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SD_t3.h>
#include <TimeLib.h>
#include <LIDARLite.h>
#include <DS1307RTC.h>
#include "BN55.h"

/*******************************************************************/
/************  Define constants, variables and object  *************/
#define White 35
#define Yellow 36
#define Red 37
#define A 0X28
#define LOG_INTERVAL 20
#define SYNC_INTERVAL 5

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

long interval = 50;
const int chipSelect = BUILTIN_SDCARD;
const int ledPin = 13;
uint32_t syncTime = 0;
int ledState = LOW;
char buf1[11];
char buf2[10];

tmElements_t tm;
File logfile;
BN55 mySensor(A);
LIDARLite myLidar;

/*******************************************************************/
/************************  Setup Function  *************************/
void setup()
{
  bool parse=false;
  bool config=false;

  if (getDate(__DATE__) && getTime(__TIME__))
  {
    parse = true;
    if (RTC.write(tm))
    {
      config = true;
      }
      }
      Wire.begin();
      setSyncProvider(getTeensy3Time);
      mySensor.init();
      myLidar.begin(0,true);
      myLidar.configure(0);
      Serial.begin(115200);
      while (!Serial)
      {
        ;
        }
        Serial.println("Press any key to continue...");
        while (Serial.read() != '/')
        {
          //don't do anything
          }
          if (parse && config) {
            Serial.print("DS1307 configured Time=");
            Serial.print(__TIME__);
            Serial.print(", Date=");
            Serial.println(__DATE__);
            }
            else if (parse) {
              Serial.println("DS1307 Communication Error :-{");
              Serial.println("Please check your circuitry");
              }
              else {
                Serial.print("Could not parse info from the compiler, Time=\"");
                Serial.print(__TIME__);
                Serial.print("\", Date=\"");
                Serial.print(__DATE__);
                Serial.println("\"");
                }
                if (timeStatus() != timeSet)
                {
                  Serial.println("Unable to sync with the RTC");
                }
                else
                {
                  Serial.println("RTC has set the system time");
                }
                Serial.println("Initializing SD card...");
                if (!SD.begin(chipSelect))
                {
                  Serial.println("Card failed, or not present !!");
                  return;
                  }
                  Serial.println("SD card initialized !!");
                  pinMode(ledPin, OUTPUT);
                  pinMode(Yellow, OUTPUT);
                  pinMode(White, OUTPUT);
                  pinMode(Red, OUTPUT);
                  digitalWrite(Yellow, LOW);
                  digitalWrite(White, HIGH);
                  digitalWrite(Red, HIGH);
                  delay(200);
                  Serial.println("Setup completed !!");
}

/*******************************************************************/
/*************************  Main Loop  *****************************/
void loop()
{
  String dataString = "";
  Serial.println(" ");
  Serial.println("Initializing camera...");
  digitalWrite(White, LOW);
  delay(1000);
  digitalWrite(Red, LOW);
  delay(2000);
  LogFunc();
  RESET();
}

/*******************************************************************/
/*************************  Log Function  **************************/
void LogFunc()
{
    time_t t = processSyncMessage();
    if (t != 0)
    {
      Teensy3Clock.set(t);
      setTime(t);
    }
    String Year = (tm.Year) + 1970;
    String Mon = tm.Month;
    String Day ;//= day();
    String Hou ;//= hour();
    String Min ;//= minute();
    String Sec ;//= second();
    String lon = buf1;
    String lat = buf2;

  logfile.flush();                //physically save any bytes written to the file to the SD card
  char filename[] = "Log000.txt";
  for (uint16_t i = 0; i < 1000; i++)
  {
    byte Hun = i / 100;
    filename[3] = i / 100 + '0';
    filename[4] = (i - Hun * 100) / 10 + '0';
    filename[5] = i % 10 + '0';
    if (!SD.exists(filename))
    {
      logfile = SD.open(filename, FILE_WRITE);
      Serial.print("Logging to: ");
      Serial.println(filename);
      break;  // leave the loop!
    }
  }
  if (! logfile)
  {
          Serial.println("Error: Log file could not created");
  }
  if (logfile)
  {
    logfile.println("No.\tDate\t\tTime\t\tDist(m)\t\tLongitude\tLattitude\t\tYaw\tRoll\tPitch");
    for (int k=1; k<11; k++)
    {
      time_t t = processSyncMessage();
      if (t != 0)
      {
        Teensy3Clock.set(t);
        setTime(t);
      }
      if (RTC.read(tm)) {
      String Yea = tm.Year;
      String Mon = tm.Month;
      String Day = tm.Day;
      String Hou = tm.Hour;
      String Min = tm.Minute;
      String Sec = tm.Second;
      } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
    delay(9000);
  }
      String lon = buf1;
      String lat = buf2;

      mySensor.readEul();
      String yaw = (mySensor.euler.x);
      String rol = (mySensor.euler.y);
      String pit = (mySensor.euler.z);

      mySensor.readAccel();
      String Accx = (mySensor.accel.x);
      String Accy = (mySensor.accel.y);
      String Accz = (mySensor.accel.z);

      mySensor.readGyro();
      String Gyrox = (mySensor.gyro.x);
      String Gyroy = (mySensor.gyro.y);
      String Gyroz = (mySensor.gyro.z);

      float rdist = myLidar.distance();
      String dist = rdist/100;

      if (Serial1.available())
      {
        Serial1.readBytes(buf1, 11);
        Serial1.readBytes(buf2, 10);
      }
      
      RTC.read(tm);
      String count = k;
      String dataline = (count + "\t" + Year + "-" + tm.Month + "-" + tm.Day + "\t\t" + tm.Hour + ":" + tm.Minute + ":" + tm.Second + "\t\t" + dist + "\t\t" + lon + "\t" + lat + "\t" + yaw  + "\t" + rol  + "\t" + pit);
      logfile.println(dataline);
      delay(100);
      Serial.println(dataline);
    }
    logfile.close();
  }
}

/*******************************************************************/
/******************  Reset and Time Function  **********************/
void RESET()
{
  digitalWrite(White, HIGH);
  digitalWrite(Red, HIGH);
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*******************************************************************/
/****************  Time Sync Messages Processing  ******************/
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage()
{
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1546300800; // Jan 1 2019

  if (Serial.find(TIME_HEADER))
  {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME)
    {
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void print2digits(int number)
{
  if (number >= 0 && number < 10)
  {
    Serial.write('0');
    }
  Serial.print(number);
}
