// Created by M. Rakeh Saleem
// Date created: 19-Mar-2020
// last modified: 19-Mar-2020
//
// Bluetooth based smart concrete system
////////////////////////////////////////

#include<bluefruit.h>

int adc_conc = A5;
int adc_batt = A4; //NEW!!!!
float V_conc = 0, V_batt = 3.0;
int Raw_conc = 0, Raw_batt = 0;
float Rconc = 0; //float
int R1 = 996; // [k ohm]

char V1[9];
// Get the raw 14-bit, 0..3000mV ADC value
float mv_per_lsb = 3300.0F/16384.0F; // 10-bit ADC with 3.3V input range
//float mv_per_lsb1 = 3600.0F/256.0F; // 10-bit ADC with 3.6V input range

uint8_t beaconUuid[16] = 
{ 
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0, 
};

BLEBeacon beacon(beaconUuid); // beacon 객체 생성

void setup() {
  Serial.begin(115200);
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 14-bit //(0..16384)
  analogReadResolution(14); // Can be 8, 10, 12 or 14
  
  Bluefruit.begin(); // begin(uint8_t prph_count, uint8_t central_count) -> 최적의 SRAM사용을 위해 연결될 peripheral과 central 수를 세는 것 뿐임...??
  Bluefruit.setTxPower(0);
  beacon.setManufacturer(0x004C);
  beacon.setRssiAt1m(-54);
  startAdv();    
}

void startAdv(void)
{
  Bluefruit.Advertising.setBeacon(beacon);
  Bluefruit.Advertising.setInterval(1600, 9600);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(10);      // 10초 지난 뒤에 fast advertising mode(1600ms)에서 slow advertising mode(9600ms)로 변화
}

void loop()
{
  Raw_conc = analogRead(adc_conc); //concrete voltage
  Raw_batt = analogRead(adc_batt); //battery voltage

  //Serial.println(Raw_conc);
  
  V_conc = (float)Raw_conc * mv_per_lsb;
  V_batt = (float)Raw_batt * mv_per_lsb;

  V_conc = V_conc/1000;
  
  Serial.print("Concrete voltage and Battery voltage are: ");
  Serial.print(V_conc);
  Serial.print("mV, ");
  Serial.print(V_batt);
  Serial.println("mV");
  
  //calculate Rconc
  Rconc = (V_conc*R1)/(V_batt-V_conc); //kohm unit
  Serial.print("Concrete Resistance : ");
  Serial.println(Rconc);
  Serial.println(V);
  sprintf(V1, "%.9f", V);// = (double)(V/1000);
  Serial.print(" [");
  Serial.println(V1);
  Serial.println(" ");

  beacon.setMajorMinor(__swap16(Rconc),__swap16((int)(V_conc*10))); //
  Bluefruit.Advertising.setBeacon(beacon);
  Bluefruit.Advertising.start(10); // Stop advertising entirely after 10 seconds 
  Bluefruit.Advertising.getData();  //what is get data? -->add meaning here
  
  delay(2000);
}
