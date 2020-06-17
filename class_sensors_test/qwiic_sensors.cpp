#include "qwiic_sensors.h"
//
// base QWIIC sensor class
//
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
  if(!available || 
     (millisNow - lastSampleMillis) < minSampleInterval)
  {
    return false;
  }
  lastSampleMillis = millisNow;
  return ReadSensor();
}
bool QWIIC_Sensor::ReadSensor()
{
  return false;
}
void QWIIC_Sensor::WriteToCSV()
{
}
void QWIIC_Sensor::WriteToBinary()
{
  
}
bool QWIIC_Sensor::Initialize(int addr)
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
  i2cAddress = address;
  if (digitalSensor_SX1509.begin(i2cAddress) == true)
  {
    available = true;
    Serial.printf("digitalSensor_SX1509 at 0x%02X started\n", i2cAddress);
    sensorName = String("SX1509_0x");
    sensorName.concat(String(i2cAddress, HEX));
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
    Serial.printf("digitalSensor_SX1509 at 0x%02X not started\nCheck wiring\n", i2cAddress);
  }
  return available;
}
bool SX1509_Sensor::ReadSensor()
{
  pinVals = digitalSensor_SX1509.readWord(0x10);
  return true;
}
void SX1509_Sensor::LogPin(int pinNum, bool logPin)
{
  if((pinNum < 0) || (pinNum > 15))
  {
    return;
  }
  logPins[pinNum] = logPin;
}
void SX1509_Sensor::WriteToCSV()
{
  Serial.print(sensorName);
  Serial.print(" ");
  for(int pinIdx = 0; pinIdx < 16; pinIdx++)
  {
    if(logPins[pinIdx] == true)
    {
      Serial.printf("D%02d %s, ", pinIdx, (pinVals & (1 << pinIdx)) > 0 ?  " L" : " H"); 
    }
    //else
    //{
    //  Serial.printf("D%02d %s,", pinIdx, " -,"); 
    //}
  }
}
void SX1509_Sensor::WriteToBinary()
{}
//
// Sparkfun ADS1015 QWIIC analog to digital
//
ADS1015_Sensor::ADS1015_Sensor()
{
}
bool ADS1015_Sensor::Initialize(int address)
{
  i2cAddress = address;
  if (analogSensor_ADS1015.begin(i2cAddress) == true)
  {
    analogSensor_ADS1015.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS); 
    analogSensor_ADS1015.setSampleRate(ADS1015_CONFIG_RATE_3300HZ);
    available = true;
    Serial.printf("analogSensor_ADS1015 at 0x%02X started\n", i2cAddress);
    sensorName = String("ADS1505_0x");
    sensorName.concat(String(i2cAddress, HEX));
  }
  else
  {
    available = false;
    Serial.printf("analogSensor_ADS1015 at 0x%02X not started\nCheck wiring\n", i2cAddress);
  }
  return available;
}
bool ADS1015_Sensor::ReadSensor()
{
  for(int pinIdx = 0; pinIdx <4; pinIdx++)
  {
	if(!logPins[pinIdx])
	{
	  pinVals[pinIdx] = 0;
	  continue;
	}
	pinVals[pinIdx] = analogSensor_ADS1015.getSingleEnded(pinIdx);
  }
  return true;
}
void ADS1015_Sensor::LogPin(int pinNum, bool logPin)
{
  if((pinNum < 0) || (pinNum > 4))
  {
    return;
  }
  logPins[pinNum] = logPin;
}
void ADS1015_Sensor::WriteToCSV()
{
  Serial.print(sensorName);
  Serial.print(" ");

  for(int pinIdx = 0; pinIdx < 4; pinIdx++)
  {
    if(logPins[pinIdx] == true)
    {
      Serial.printf("A%02d %04d, ", pinIdx, pinVals[pinIdx]);
    }
    //else
    //{
    //  Serial.printf("A%02d ----, ", pinIdx, pinVals[pinIdx]);
    //}
  }
}
void ADS1015_Sensor::WriteToBinary()
{
	
}
//
//
//
BNO080_Sensor::BNO080_Sensor()
{	
}
bool BNO080_Sensor::Initialize(int address)
{
  i2cAddress = address;
  if (imuSensor_BNO080.begin(i2cAddress, Wire) == true)
  {
    available = true;
    Serial.printf("imuSensor_BNO080 at 0x%02X started\n", i2cAddress);
    sensorName = String("BNO080_0x");
    sensorName.concat(String(i2cAddress, HEX));
	if(logRotation)
	{
      imuSensor_BNO080.enableRotationVector(minSampleInterval);
	}
	if(logAccel)
	{
      imuSensor_BNO080.enableAccelerometer(minSampleInterval);
    }
	if(logGyro)
	{
      imuSensor_BNO080.enableGyro(minSampleInterval);	
	}
  }
  else
  {
    available = false;
    Serial.printf("imuSensor_BNO080 at 0x%02X not started\nCheck wiring\n", i2cAddress);
  }
  return available;
}
void BNO080_Sensor::LogRotation(bool log)
{
	logRotation = log;
	if(logRotation && available)
	{
      imuSensor_BNO080.enableRotationVector(minSampleInterval);
	}
}
void BNO080_Sensor::LogAcceleration(bool log)
{
	logAccel = log;
	if(logAccel && available)
	{
      imuSensor_BNO080.enableAccelerometer(minSampleInterval);
	}
}
void BNO080_Sensor::LogLinearAcceleration(bool log)
{
	logLinearAccel = log;
	if(logLinearAccel && available)
	{
      imuSensor_BNO080.enableLinearAccelerometer(minSampleInterval);
	}
}
void BNO080_Sensor::LogGyro(bool log)
{
	logGyro = log;
	if(logGyro && available)
	{
      imuSensor_BNO080.enableGyro(minSampleInterval);
	}
}
void BNO080_Sensor::LogMagnetometer(bool log)
{
	logMagnetometer = log;
	if(logMagnetometer && available)
	{
      imuSensor_BNO080.enableMagnetometer(minSampleInterval);
	}
}

bool BNO080_Sensor::ReadSensor()
{
	if(imuSensor_BNO080.dataAvailable() == true)
	{
    if(logRotation || logGyro || logAccel || logLinearAccel || logMagnetometer)
    {
      Serial.print(sensorName);
      Serial.print(" ");
    }
	  if(logRotation)
	  {
        quatI = imuSensor_BNO080.getQuatI();
        quatJ = imuSensor_BNO080.getQuatJ();
        quatK = imuSensor_BNO080.getQuatK();
        quatReal = imuSensor_BNO080.getQuatReal();
        quatRadianAccuracy = imuSensor_BNO080.getQuatRadianAccuracy();
	  }
      if (logGyro)
      {
        gyroX = imuSensor_BNO080.getGyroX();
        gyroY = imuSensor_BNO080.getGyroY();
        gyroZ = imuSensor_BNO080.getGyroZ();
      }
      if (logAccel)
      {
        accelX = imuSensor_BNO080.getAccelX();
        accelY = imuSensor_BNO080.getAccelY();
        accelZ = imuSensor_BNO080.getAccelZ();
      }
	  if(logLinearAccel)
	  {
        linAccelX = imuSensor_BNO080.getLinAccelX();
        linAccelY = imuSensor_BNO080.getLinAccelY();
        linAccelZ = imuSensor_BNO080.getLinAccelZ();
        linAccelAccuracy = imuSensor_BNO080.getLinAccelAccuracy();
	  }
      if (logMagnetometer)
      {
        magX = imuSensor_BNO080.getMagX();
        magY = imuSensor_BNO080.getMagY();
        magZ = imuSensor_BNO080.getMagZ();
        magAccuracy = imuSensor_BNO080.getMagAccuracy();
      }
	  return true;
	}
	return false;
}
void BNO080_Sensor::WriteToCSV()
{
  if(logRotation)
  {
	Serial.print(", quat ");
    Serial.print(quatI);
    Serial.print(",");
    Serial.print(quatJ);
    Serial.print(",");
    Serial.print(quatK);
    Serial.print(",");
    Serial.print(quatReal);
    Serial.print(",");
    Serial.print(quatRadianAccuracy);
  }
  if(logGyro)
  {
	Serial.print(", gyro ");
    Serial.print(gyroX);
    Serial.print(",");
    Serial.print(gyroY);
    Serial.print(",");
    Serial.print(gyroZ);
  }
  if(logAccel)
  {
	Serial.print(", accel ");
    Serial.print(accelX);
    Serial.print(",");
    Serial.print(accelY);
    Serial.print(",");
    Serial.print(accelZ);
  }
  if(logLinearAccel)
  {
	Serial.print(", linAccel ");
    Serial.print(linAccelX);
    Serial.print(",");
    Serial.print(linAccelY);
    Serial.print(",");
    Serial.print(linAccelZ);
  }
  if(logMagnetometer)
  {
	Serial.print(", mag ");
    Serial.print(magX);
    Serial.print(",");
    Serial.print(magY);
    Serial.print(",");
    Serial.print(magZ);
  }
}
void BNO080_Sensor::WriteToBinary()
{}	
