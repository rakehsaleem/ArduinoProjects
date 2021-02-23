/*******************************************************************/
//  Created by M. Rakeh Saleem
//  Date created: 7-Aug-2019
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
const int chipSelect = BUILTIN_SDCARD;
String pack;
String lon,lat,alt,sec,nsec ;
int ind1,ind2,ind3,ind4,ind5;
uint16_t lognumber;
uint16_t indexnumber=1;
int iprev=0;

ros::NodeHandle  nh;
tmElements_t tm;
static File logfile;

char filename[10]; 
char bufnh[20];
bool dataflag = true;
static BN55 mySensor(A);
LIDARLite myLidar;

void Acquire(uint16_t index,bool save=true);
int flag_srv=0;  // for the trigger from PC
using rosserial_arduino::Test;

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
/********************  Ros Service Function  ********************/

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

//FLASH WITHOUT DELAY
class Shutter
{
  // Class Member Variables
  // These are initialized at startup
  int ShutterPin;  // the number of the Shutter pin
  long OnTime;     // milliseconds of Sutter on
  long OffTime;    // milliseconds of Shtter off
   
  // These maintain the current state
  int ShutterState;                 // ledState used to set the LED
  unsigned long previousMillis;     // will store last time LED was updated
 
  // Constructor - creates a Flasher 
  // and initializes the member variables and state
  public:
  Shutter(int pin, unsigned long on, unsigned long off)
  {
    ShutterPin = pin;
    pinMode(ShutterPin, OUTPUT); //red
    
    OnTime = on;
    OffTime = off;
    
    ShutterState = HIGH;
    previousMillis = 0;
    }
  
 void Update()
  {
    // check to see if it's time to change the state of the LED
    unsigned long long currentMillis = millis();
     
    if((ShutterState == LOW) && (currentMillis - previousMillis >= OnTime))
    {
      //idel_offcycle
      ShutterState = HIGH;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      //digitalWrite(White, HIGH);//Release focus!
      //digitalWrite(ShutterPin, ShutterState);  // Update the Shutter
      //delay(10);//
      digitalWrite(White, LOW);//Press focus!
      nh.loginfo("FOCUSING CAMERA");
    }
    else if ((ShutterState == HIGH) && (currentMillis - previousMillis >= OffTime))
    {
      //activate
      ShutterState = LOW;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      //modified
      digitalWrite(White, LOW);// White and Shutter pin needs to be pressed together 
      digitalWrite(ShutterPin, ShutterState);   // Update the Shutter
      nh.loginfo("CAMERA SHOOTING BEGIN");
    }
  }
 
int State()
{
  return   ShutterState;
  }
};
 
//Class measurement update;
class Sensor
{
   unsigned long long  updateInterval;      // interval between updates
   unsigned long long lastUpdate;           // last update of position
   int  index; 
  
  public:
  Sensor(unsigned long long interval)
  {
    updateInterval = interval;
    lastUpdate=0;
    index=1;
    }
  
  int Update()
  {
    if((millis()-lastUpdate)>updateInterval)
    {
      lastUpdate=millis();
      Acquire(index);
      index+=1;
      }
      return index;
    }
};
  
// Shutter init
Shutter Shot(Red, 2000, 500);//510 focus, 2000 shoot
Sensor sensor(200);//update every 195 ms //need to adjust
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
  if (parse && config)
  {
//    Serial.print("DS1307 configured Time=");
  }
  else if (parse)
  {
//    Serial.println("DS1307 Communication Error");
  }
  else
  {
//    Serial.print("Could not parse info from the compiler, Time=\"");
  }
//  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect))
  {
//    Serial.println("Card failed, or not present !!");
    return;
  }
  pinMode(Yellow, OUTPUT);
  pinMode(White, OUTPUT);
  //pinMode(Red, OUTPUT);
  digitalWrite(Yellow, LOW);
  digitalWrite(White, HIGH);
  digitalWrite(Red, HIGH);
  lognumber=LogInit();      //get the inital number for file saving
  LogUpdate(lognumber);
  indexnumber=1;
 
  digitalWrite(White, LOW);//Press focus!
  delay(500); //initial focus

//  while(!nh.connected()) nh.spinOnce();
//  delay(10000);
//  Serial.println("Press any key to continue...");
//  while (Serial.read() != '/')
//  {
//    //don't do anything
//  }
}

/*******************************************************************/
/*************************  Main Loop  *****************************/
void loop()
{
  String dataString = "";  
//if (indexnumber==1)
//nh.loginfo("Start");
//Serial.println(" ");
//Serial.println("Initializing camera...");

//service call from PC - 1: start, 2: stop
  switch(flag_srv)
  { 
    case 1:
    Shot.Update();//start shooting at interval of 2.5 s.  0.5s off, 2s on. 
//  snprintf(bufnh, 20, "id:%d st:%d", indexnumber, Shot.State());
//  nh.loginfo(bufnh);
    if (Shot.State()== LOW )
    {
      mySensor.init();
      while(1)
      {
        nh.spinOnce();
        indexnumber=sensor.Update();
             
//      if(iprev !=indexnumber){
//      nh.loginfo("Inside Loop");
//      snprintf(bufnh, 20, "id:%d st:%d", indexnumber, Shot.State());
//      nh.loginfo(bufnh);
//      iprev=indexnumber;
//      }
//      snprintf(bufnh, 20, "index: %d", indexnumber);
//      nh.loginfo(bufnh);

        if (indexnumber % 11 == 0)
        //delay(50);
        {break;}
        }
        RESET();//release shutter
//      nh.loginfo("Release Shutter \r\n");
//      snprintf(bufnh, 20, "id:%d st:%d", indexnumber, Shot.State());
//      nh.loginfo(bufnh);

        dataflag=true;    
        }  
        else
        {
          if (dataflag==true)
          {
//          nh.loginfo("ELSE \r\n");
//          snprintf(bufnh, 20, "index: %d", indexnumber);
//          nh.loginfo(bufnh);
            logfile.close();
            nh.loginfo("Close File \r\n");
            LogUpdate(++lognumber);//SD writing takes some time.
            snprintf(bufnh, 20, "Log index: %d", lognumber);
            nh.loginfo(bufnh);
            snprintf(bufnh, 20, "id:%d Shot:%d", indexnumber,Shot.State());
            nh.loginfo(bufnh);
            dataflag=false;//file closed and opened
            }
          }
          break;
    case 2:
          RESET();
          delay(1000);
          break;  
    default:
          break;   
  }
  nh.spinOnce();
}

/*******************************************************************/
/*************************  Log Function  **************************/
int LogInit()
{
  logfile.flush();                //physically save any bytes written to the file to the SD card
  for (lognumber = 0; lognumber < 2000; lognumber++)
  {  
  sprintf(filename, "%d.txt",lognumber)  ;  
  if (!SD.exists(filename))
    {
//      logfile = SD.open(filename, FILE_WRITE);
//      Serial.print("Logging to: ");
//      Serial.println(filename);
      break;  // leave the loop!
    }
  }
 return lognumber;
}

void LogUpdate(uint16_t logindex)
{
  sprintf(filename, "%d.txt",logindex)    ;
  snprintf(bufnh, 20, "%s", filename);
  nh.loginfo(bufnh);
  logfile = SD.open(filename, FILE_WRITE);
//  logfile.println("No.\t\tTime\t\trosTime\t\tDist(m)\t\tLongitude\t\tLattitude\t\tAltitude\t\tYaw\tRoll\tPitch");
  logfile.println("No.,Time,rosTime,Dist(m),Longitude,Lattitude,Altitude,Yaw,Roll,Pitch");
}
 
void Acquire(uint16_t index,bool save=true)
{
  mySensor.readEul();
  String yaw = (mySensor.euler.x);
  String rol = (mySensor.euler.y);
  String pit = (mySensor.euler.z);
  float rdist = myLidar.distance(false);
  String dist = rdist / 100;
  RTC.read(tm);
  String count = (index+10)%11;
//  String dataline = (count + "\t" +"\t\t" + tm.Hour + ":" + tm.Minute + ":" + tm.Second + "\t\t" + sec + '.' + nsec + "\t\t" + dist + "\t\t" + lon + "\t" + lat + "\t" + alt + "\t" + yaw  + "\t" + rol  + "\t" + pit);
  String dataline = (count + "," + tm.Hour + ":" + tm.Minute + ":" + tm.Second + "," + sec + '.' + nsec + "," + dist + "," + lon + "," + lat + "," + alt + "," + yaw  + "," + rol  + "," + pit);
//  String dataline = (count + "\t" + pack);
  if((index+10)%11<10) 
  {
    logfile.println(dataline);
    nh.loginfo((count+"/"+yaw+","+rol+","+pit).c_str());
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
