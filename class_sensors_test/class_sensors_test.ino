#include "qwiic_sensors.h"
#include <Wire.h>
#include <iostream>
#include <vector>
#include <memory>
#include <cstdio>
#include <fstream>
#include <cassert>
#include <functional>
#define qwiic Wire

//std::vector<std::unique_ptr<QWIIC_Sensor> > sensors(2);

QWIIC_Sensor *sensors[5];
SX1509_Sensor digitalIO;
unsigned long currentMillis;
unsigned long priorMillis = 0;
int sensorCount = 0;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Artemis class based logger test");
  Wire.begin();
  Wire.setClock(400000);
  sensors[0] = new SX1509_Sensor();
  sensors[1] = new ADS1015_Sensor();
  sensors[2] = new BNO080_Sensor();

  sensors[0]->Initialize(0x3E);
  sensors[0]->SetMinSampleInterval(1);
  ((SX1509_Sensor*)sensors[0])->LogPin(0, true);
  ((SX1509_Sensor*)sensors[0])->LogPin(1, true);
  ((SX1509_Sensor*)sensors[0])->LogPin(2, true);
//
  sensors[1]->Initialize(0x48);
  sensors[1]->SetMinSampleInterval(1);
  ((ADS1015_Sensor*)sensors[1])->LogPin(0, false);
  ((ADS1015_Sensor*)sensors[1])->LogPin(1, false);
  ((ADS1015_Sensor*)sensors[1])->LogPin(2, false);
  ((ADS1015_Sensor*)sensors[1])->LogPin(3, true);
  
  sensors[2]->Initialize(0x4B);
  sensors[2]->SetMinSampleInterval(5);
  ((BNO080_Sensor*)sensors[2])->LogAcceleration(true);
  ((BNO080_Sensor*)sensors[2])->LogLinearAcceleration(true);

  sensorCount = 3;
}

void loop() 
{
  /**/
  unsigned long currentMillis = millis();
  unsigned long printMillis = currentMillis - priorMillis;
  priorMillis = currentMillis;
  bool printTime = false;
  for(int sensorIdx = 0; sensorIdx < sensorCount; sensorIdx++)
  {
    currentMillis = millis();
    if(sensors[sensorIdx]->SampleSensor(currentMillis))
    {
      sensors[sensorIdx]->WriteToCSV();
      printTime = true;
    }
  }
  if(printTime)
  {
    Serial.printf("\t%ld mS\t%ld delta mS", currentMillis, printMillis);
    Serial.print("\n");
  }
}
