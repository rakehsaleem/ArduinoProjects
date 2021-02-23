// Modified by M. Rakeh Saleem
// Date created: --
// last modified: 19-Nov-2019
//
// LidarLite V3 and BNO055 IMU configuration 
////////////////////////////////////////////

#include <Wire.h>
#include <LIDARLite.h>       
#include "BN55.h"

#define A 0X28
#define White 35
#define Yellow 36
#define Red 37

LIDARLite myLidarLite;
BN55 mySensor(A);

void setup(){
  Serial.begin(9600);
  Wire.begin();
  mySensor.init();
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); // Change this number to try out alternate configuration
  pinMode(Yellow, OUTPUT);
  pinMode(White, OUTPUT);
  pinMode(Red, OUTPUT);
  digitalWrite(Yellow, LOW);
  digitalWrite(White, HIGH);
  digitalWrite(Red, HIGH);
  }

void loop(){
  float dist = myLidarLite.distance(false);
  String res = dist/100;
  Serial.print(dist);
  Serial.print(" cm   ");
  Serial.print(res);
  Serial.println(" m");
  Serial.println(" ");  
  digitalWrite(White, LOW);
  delay(1000);
  digitalWrite(Red, LOW);
  delay(2000);
  //Euler angles
  mySensor.readEul();
  Serial.print("Yaw (dg): "); Serial.print(mySensor.euler.x); Serial.print("  Roll (dg): "); Serial.print(mySensor.euler.y); Serial.print("  Pitch (dg): "); Serial.println(mySensor.euler.z);
  mySensor.readTemp();
  Serial.print("Temp (C): "); Serial.print(mySensor.temp.c);Serial.print(" Temp (F): "); Serial.println(mySensor.temp.f);
  
  float dist1 = myLidarLite.distance();
  String res1 = dist1/100;
  Serial.print(dist1);
  Serial.print(" cm   ");
  Serial.print(res1);
  Serial.println(" m");
  Serial.println(" ");
  digitalWrite(Red, HIGH);
  delay(500);
}
