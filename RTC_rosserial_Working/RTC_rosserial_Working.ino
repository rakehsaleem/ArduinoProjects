/*******************************************************************/
//  Created by M. Rakeh Saleem
//  Date created: 27-Jun-2019
//  last modified: 12-Aug-2019 by JWP
//  Rosserial working with RTC, IMU and teensy (fully configured)
//
//  relesase note  ###
//  - rosservice added (name: /test_srv)
//  - $ rosservice call /test_srv "input: '1'"     : will start the program 
//  - $ rosservice call /test_srv "input: '2'"     : will stop the program

/*******************************************************************/
/******************  Include required Libraries  *******************/
#include <ros.h>
#include <std_msgs/String.h>
#include <ADC.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SD_t3.h>
#include <TimeLib.h>
#include <LIDARLite.h>
#include <DS1307RTC.h>
#include "BN55.h"
#include <rosserial_arduino/Test.h>

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

const int readPin = A9; // ADC0
int led = 13;
long interval = 50;
const int chipSelect = BUILTIN_SDCARD;
String pack;
String lon,lat,alt,sec,nsec ;
int ind1,ind2,ind3,ind4,ind5;
const int ledPin = 13;
uint32_t syncTime = 0;
int ledState = LOW;
char buf1[11];
char buf2[10];

ros::NodeHandle  nh;
int flag_srv=0;  // for the trigger from PC
using rosserial_arduino::Test;

ADC *adc = new ADC(); // adc object
tmElements_t tm;
File logfile;
BN55 mySensor(A);
LIDARLite myLidar;

/*******************************************************************/
/********************  Ros Subscriber Function  ********************/
std_msgs::String msgS;

void messageCb(const std_msgs::String& msg){
  msgS.data = msg.data;
  pack = msgS.data;
  
  ind1 = pack.indexOf(',');  //finds location of first ,
  sec = pack.substring(0, ind1);   //captures first data String
  
  ind2 = pack.indexOf(',', ind1+1 );   //finds location of second ,
  nsec = pack.substring(ind1+1, ind2);   //captures second data String
  
  ind3 = pack.indexOf(',', ind2+1 );
  lat = pack.substring(ind2+1, ind3);
  
  ind4 = pack.indexOf(',', ind3+1 );
  lon = pack.substring(ind3+1, ind4); //captures remain part of data after last ,
  
  ind5 = pack.indexOf(',', ind4+1 );
  alt = pack.substring(ind4+1);
  
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  nh.loginfo("subs ok");
}

ros::Subscriber<std_msgs::String> sub("/gps/string", messageCb);

/*******************************************************************/
/********************  Ros Service Function  ***********************/
void callback(const Test::Request & req, Test::Response & res){
  String temp = req.input;
  if(temp[0]=='1'){
    flag_srv=1;
    nh.loginfo("camera start");}

  else if(temp[0]=='2'){
    flag_srv=2;
    nh.loginfo("camera stop");}

  else
    res.output = "none";
}

ros::ServiceServer<Test::Request, Test::Response> server("test_srv",&callback);

/*******************************************************************/
/************************  Setup Function  *************************/
void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(sub);
  nh.advertiseService(server);
  nh.loginfo("from teensy");
  
  bool parse = false;
  bool config = false;

  if (getDate(__DATE__) && getTime(__TIME__))
  {
    parse = true;
    if (RTC.write(tm))
    {
      config = true;
    }
  }
  Wire.begin();
  mySensor.init();
  myLidar.begin(0, true);
  myLidar.configure(0);
  Serial.begin(115200);
  //Serial1.begin(115200);
  while (!Serial)
  {
    ;
  }
  if (parse && config) {
//    Serial.print("DS1307 configured Time=");
//    Serial.print(__TIME__);
//    Serial.print(", Date=");
//    Serial.println(__DATE__);
  }
  else if (parse) {
//    Serial.println("DS1307 Communication Error :-{");
//    Serial.println("Please check your circuitry");
  }
  else {
//    Serial.print("Could not parse info from the compiler, Time=\"");
//    Serial.print(__TIME__);
//    Serial.print("\", Date=\"");
//    Serial.print(__DATE__);
//    Serial.println("\"");
  }
//  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect))
  {
//    Serial.println("Card failed, or not present !!");
    return;
  }
//  Serial.println("SD card initialized !!");
  pinMode(ledPin, OUTPUT);
  pinMode(Yellow, OUTPUT);
  pinMode(White, OUTPUT);
  pinMode(Red, OUTPUT);
  digitalWrite(Yellow, LOW);
  digitalWrite(White, HIGH);
  digitalWrite(Red, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(readPin, INPUT);
    pinMode(A10, INPUT); //Diff Channel 0 Positive
    pinMode(A11, INPUT); //Diff Channel 0 Negative

    #if ADC_NUM_ADCS>1
    pinMode(A12, INPUT); //Diff Channel 3 Positive
    pinMode(A13, INPUT); //Diff Channel 3 Negative
    #endif
    //Serial.begin(9600);
    adc->setAveraging(16); // set number of averages
    adc->setResolution(16); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
    adc->startContinuous(readPin, ADC_0);
    
    #if ADC_NUM_ADCS>1
    adc->setAveraging(16, ADC_1); // set number of averages
    adc->setResolution(16, ADC_1); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED, ADC_1); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1); // change the sampling speed
    #endif
    delay(500);
  //delay(2000);
  //Serial.println("Setup completed !!\n");
  Serial.println("Press any key to continue...");
  while (Serial.read() != '/')
  {
    //don't do anything
  }
}
//void(*resetFunc)(void)=0;

/*******************************************************************/
/*************************  Main Loop  *****************************/
int value = 0;
int conval = 0;
void loop()
{
  String dataString = "";
  digitalWrite(White, LOW);
  nh.loginfo("FOCUSING CAMERA");
  //delay(500);
  digitalWrite(Red, LOW);
  nh.loginfo("CAMERA SHOOTING BEGIN");
  //delay(20);
  //RESET();
  //LogFunc();
  if (readtrigger() != true)
  {
    
    Serial.println("Trigger");
    LogFunc();
  }
  else Serial.println(" ------------- ");
  delay(50);
  while (Serial.read() == '*')
  {
//    Serial.println(" ");
//    Serial.println("S T O P");
//    Serial.println(" ");
    _reboot_Teensyduino_();
  }
  RESET();
}

/*******************************************************************/
/*************************  Log Function  **************************/
bool readtrigger()
{
  long fval = 0;
  if (adc->isComplete(ADC_0))
  {
    value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
    //Serial.println(value);
    //fval = (value*3.3/65535);
    //Serial.println(fval);
    Serial.println(value*3.3/adc->getMaxValue(ADC_0), DEC);
  }
  if (value > 2000) return true;
  else return false;
}

/*******************************************************************/
/**********************  Trigger Function  *************************/
void LogFunc()
{
  String Year = (tm.Year) + 1970;
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
    //
    }
  if (logfile)
  {
    logfile.println("No.\tDate\tTime\t\tDist(m)\t\tLongitude\tLattitude\t\tYaw\tRoll\tPitch");
      String lon = buf1;
      String lat = buf2;

      mySensor.readEul();
      String yaw = (mySensor.euler.x);
      String rol = (mySensor.euler.y);
      String pit = (mySensor.euler.z);

      float rdist = myLidar.distance();
      String dist = rdist / 100;

      RTC.read(tm);
      print2digits(tm.Hour);
      String count = 1;
      String dataline = (count + "\t" + Year + "-" + tm.Month + "-" + tm.Day + "\t" + tm.Hour + ":" + tm.Minute + ":" + tm.Second + "\t\t" + dist + "\t\t" + lon + "\t" + lat + "\t" + yaw  + "\t" + rol  + "\t" + pit);
      logfile.println(dataline);
      delay(100);
      Serial.println(dataline);
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

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  //Serial.print(number);
}
