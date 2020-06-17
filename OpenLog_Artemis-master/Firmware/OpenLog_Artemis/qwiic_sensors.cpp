#include "qwiic_sensors.h"
//
// base QWIIC sensor class
//
//QWIIC_Sensor::QWIIC_Sensor()
//{
//	available = false;
//	online = false;
//	log = false;
//	i2cAddress = 0x00;
//}
bool QWIIC_Sensor::IsAvailable()
{
	return available;
}
void QWIIC_Sensor::SetAvailable(bool avail)
{
	available = avail;
}
bool QWIIC_Sensor::IsOnline()
{
	return online;
}
void QWIIC_Sensor::SetOnline(bool ol)
{
	online = ol;
}
bool QWIIC_Sensor::IsLogged()
{
	return log;
}
void QWIIC_Sensor::SetLogged(bool logSens)
{
	log = logSens;
}
void QWIIC_Sensor::SetMinSampleInterval(unsigned long minInterval)
{
  minSampleInterval = minInterval;
}
bool QWIIC_Sensor::SampleSensor(unsigned long millisNow)
{
  if((millisNow - lastSampleMillis) < minSampleInterval)
  {
    return false;
  }
  return ReadSensor();
}
bool QWIIC_Sensor::ReadSensor()
{
  return false;
}
//
// Sparkfun SX1509 QWIIC digital IO
//
SX1509_Sensor::SX1509_Sensor()
{
}
bool SX1509_Sensor::Initialize(int address)
{
  i2cAddress - address;
  digitalSensor_SX1509.begin(i2cAddress);
  if (digitalSensor_SX1509.begin(i2cAddress) == true)
  {
    available = true;
    Serial.printf("digitalSensor_SX1509 at 0x%02X started\n", i2cAddress);
    for(int pinIdx = 0; pinIdx < 16; pinIdx++)
    {
      if(logPins[pinIdx] == true)
      {
        digitalSensor_SX1509.pinMode(pinIdx, INPUT);     
      }
    }
  }
  else
  {
    available = false;
    Serial.printf("digitalSensor_SX1509 at 0x%02X not started\nCheck wiring\n?", i2cAddress);
  }
  return available;
}
bool SX1509_Sensor::ReadSensor()
{
  pinVals = digitalSensor_SX1509.readWord(0x10);
  return false;
}
String SX1509_Sensor::WriteToCSV()
{
  String outputData;
  for(int pinIdx = 0; pinIdx < 16; pinIdx++)
  {
    //outputData += " " + (String)pinIdx;
    if(logPins[pinIdx] == true)
    {
      outputData += (pinVals & (1 << pinIdx)) > 0 ?  "L," : "H,";
    }
    else
    {
      outputData += "X,";
    }
  }
  return outputData;
}
