// Created by M. Rakeh Saleem
// Date created: 24-May-2019
// last modified: 24-May-2019
//
// Serial communication for receiving lon. & lat. cooridnates 
/////////////////////////////////////////////////////////////

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
float lat = 37.4980608;
float lon = 126.9653503;
char str1[15];
char str2[15];

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  inputString.reserve(200);
}

void loop(){
  sprintf(str1, "%.7f", lon);
  sprintf(str2, "%.7f", lat);
  Serial1.print(str1);
  Serial1.print("\t");
  Serial1.println(str2);
  if (stringComplete){
    Serial2.println(inputString);
    inputString = "";
    stringComplete = false;
    }
}

void serialEvent(){
  while (Serial2.available()){
    char inChar = (char)Serial2.read();
    inputString += inChar;
    if (inChar == '\n'){
      stringComplete = true;
    }
  }
}
