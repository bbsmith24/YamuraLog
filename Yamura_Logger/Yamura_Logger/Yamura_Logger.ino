/*
 SD card attached to SPI bus as follows:
  UNO                  SD Breakout
  MOSI - D11           DI
  MISO - D12           DO
  SCK  - D13           SCK
  CS   - D8            CS
  3.3V                 VCC
  GND                  GND

  DUE (pin 1 has dot, odds on cpu side, evens on header side
  MOSI - SPI Header4   DI
  MISO - SPI Header1   DO
  SCK  - SPI Header3   SCK 
  CS   - pin 53        CS
  3.3V                 VCC
  GND                  GND

  start/stop D9
  LEDs       D10-D12
    
  I2C sensor(s) on QWIIC bus
  A2D, digital, counter, GPS, accelerometer, IMU I2C sensors on QWIIC bus
*/
// I2C support
#include <Wire.h>
// SPI and SD card support
#include <SPI.h>
#include <SD.h>
// GPS over I2C support
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // Use Library Manager to install Sparkfun QWIIC GPS or from https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include <TinyGPS++.h>                                // From: https://github.com/mikalhart/TinyGPSPlus
// accelerometer over I2C support
#include "SparkFun_MMA8452Q.h"                        // Use Library Manager to install Sparkfun Sparkfun MMA8452Q or from: http://librarymanager/All#SparkFun_MMA8452Q

//#define DEBUGSTR      // log to logger and serial port

#define STARTBUTTON  9 // start/stop button
#define GPSPIN      10 //GPS Status LED
#define ACCELPIN    11 //Accel Status LED
#define LOGGERPIN   12 //Logger Status LED

#define ANALOG1 0x08
#define ANALOG2 0x09
#define DEVICECOUNT 16
// device types
#define GPS 0
#define ACCEL 1
#define I2CATOD 2
#define I2CDIGITAL 3
#define I2CCOUNTER 4
// SD card chip select pin
#define CHIPSELECT 53
#define STDMODE
//#define FASTMODE

// for quick conversion between numeric and byte values
union DataPacket
{
  float f;          // 4 bytes
  unsigned long ul; // 4 bytes
  long sl;          // 4 bytes
  word uw[2];       // 2 x 2bytes           
  int  sw[2];       // 2 x 2bytes           
  char c[4];        // 4 bytes
};

I2CGPS myI2CGPS;     // hook gps object to the library
TinyGPSPlus gps;     // declare gps object
MMA8452Q accel;      // declare 3 axis accel MMA8452 object

// gps values
DataPacket gpsTime;
DataPacket gpsLat;
DataPacket gpsLong;
DataPacket gpsSpeed;
DataPacket gpsHeading;
DataPacket gpsSats;
// accelerometer value
DataPacket xVal;
DataPacket yVal;
DataPacket zVal;
// timestamp
DataPacket timestamp;
// channel ID packet
DataPacket channelID;


String logFileName;

int devicesPresent = 0;
bool deviceValid[DEVICECOUNT];
bool devicePresent[DEVICECOUNT];
int deviceAddress[DEVICECOUNT];
int deviceType[DEVICECOUNT];
// I2C devices value
DataPacket deviceVal[DEVICECOUNT];

File dataFile;
bool loggerRun = false;
bool waitmsg = true;
bool runmsg = true;
//
//
//
void setup()
{
  pinMode(GPSPIN, OUTPUT);
  pinMode(ACCELPIN, OUTPUT);
  pinMode(LOGGERPIN, OUTPUT);
  pinMode(STARTBUTTON, INPUT_PULLUP);
  loggerRun = false;
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(GPSPIN, HIGH);
    delay(100);
    digitalWrite(ACCELPIN, HIGH);
    delay(100);
    digitalWrite(GPSPIN, LOW);
    digitalWrite(LOGGERPIN, HIGH);
    delay(100);
    digitalWrite(ACCELPIN, LOW);
    delay(100);
    digitalWrite(LOGGERPIN, LOW);
  }
  Serial.begin(9600);
  #ifdef DEBUGSTR
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  Serial.println("Team Yamura data logger");

  // device setup
  // should be done from a file on the logger card
  for(int deviceIdx =0; deviceIdx< DEVICECOUNT; deviceIdx++)
  {
    deviceValid[deviceIdx] = false;
    devicePresent[deviceIdx] = false;
    deviceAddress[deviceIdx] = 0;
    deviceType[deviceIdx] = -1;
    deviceVal[deviceIdx].ul = 0;
  }
  devicePresent[0] = true;
  deviceAddress[0] = 0;
  deviceType[0] = GPS;
  devicePresent[1] = true;
  deviceAddress[1] = 0;
  deviceType[1] = ACCEL;
  devicePresent[2] = true;
  deviceAddress[2] = 0x08;
  deviceType[2] = I2CATOD;
  devicesPresent = 3;

  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  bool sdNotFoundMsg = true;
  if (!SD.begin(CHIPSELECT)) 
  {
    if(sdNotFoundMsg)
    {
      Serial.println("microSD card failed, or not present");
      sdNotFoundMsg = false;
    }
  }
  Serial.println("microSD card initialized.");
  
  //Initialize I2C
  Serial.println("I2C bus initialization");
  Wire.begin(); 
  #ifdef STDMODE
  Serial.println("I2C clock 100K");
  Wire.setClock(100000); // standard mode
  #endif
  #ifdef FASTMODE
  Serial.println("I2C clock 400K");
  Wire.setClock(400000);  // fast mode
  #endif
  digitalWrite(GPSPIN, LOW);
  digitalWrite(ACCELPIN, LOW);
  digitalWrite(LOGGERPIN, LOW);

  // initialize GPS
  Serial.println("GPS initialization");
  if (myI2CGPS.begin() == false)
  {
    Serial.println("GPS failed to respond. Please check wiring");
    //while (1); //Freeze!
  }
  else
  {
    Serial.println("GPS found");
  }
  // 10Hz sampling
  //Serial.println("10Hz GPS sample rate");
  //myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(220, ",100"));
  // 1Hz sampling
  Serial.println("1Hz GPS sample rate");
  myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(220, ",1000"));
  // initialize accelerometer

  Serial.println("Accelerometer initialization");
  if(accel.begin() == false)
  {
    Serial.println("Accelerometer failed to respond. Please check wiring");
  }
  else
  {
    Serial.println("Accelerometer found");
  }

  // all initialized, ready to go!
  Serial.println("Yamura Logger 2.0");
  loggerRun = false;
}
//
//
//
void loop()
{
  CheckStart();
  if(loggerRun)
  {
    CheckSensors();
    LogData();
  }
}
void CheckStart()
{
  int buttonVal = digitalRead(STARTBUTTON);
  if((buttonVal == LOW) && (loggerRun == true))
  {
    digitalWrite(LOGGERPIN, LOW);
    Serial.println("Stop");
    CloseDataFile();
    loggerRun = false;
    runmsg = true;
    delay(1000);
  }
  else if((buttonVal == LOW) && (loggerRun == false))
  {
    digitalWrite(LOGGERPIN, HIGH);
    Serial.println("Start");
    OpenDataFile();
    loggerRun = true;
    waitmsg = true;
  }
  else if(buttonVal == HIGH)
  {
    if ((loggerRun == false) && (waitmsg))
    {
      Serial.println("waiting for start...");
      waitmsg = false;
    }
    else if  ((loggerRun == true) && (runmsg))
    {
      Serial.println("running");
      runmsg = false;
    }
  }
}
void CheckSensors()
{
  Serial.print("check sensors ");
  //
  // check all devices present
  //
  for(int deviceIdx =0; deviceIdx< devicesPresent; deviceIdx++)
  {
    // channel not read yet
    deviceValid[deviceIdx] = false;
    if(deviceType[deviceIdx] == GPS)
    {
      Serial.print(" GPS ");
      // do this read in a loop of all channels defined
      // check GPS
      while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
      {
        gps.encode(myI2CGPS.read()); //Feed the GPS parser
      }
      // invalid data from GPS - skip
      if(!gps.time.isUpdated())
      {
        deviceValid[deviceIdx] = false;
        Serial.print("--");
      }
      else
      {
        deviceValid[deviceIdx] = true;
        // satellites in view
        gpsSats.ul = gps.satellites.value();
        // location
        gpsLat.f = gps.location.lat();
        gpsLong.f = gps.location.lng();
        gpsSpeed.f = gps.speed.mph();
        // heading
        gpsHeading.f = gps.course.deg();
        Serial.print("OK");
      }
    }
    else if (deviceType[deviceIdx] == ACCEL)
    {
      Serial.print(" ACCEL ");
      // invalid data from accelerometer - skip
      if (!accel.available()) 
      {
        deviceValid[deviceIdx] = false;
        Serial.print("--");
        continue;
      }
      Serial.print("OK");
      deviceValid[deviceIdx] = true;
      xVal.f = accel.getCalculatedX();
      yVal.f = accel.getCalculatedY();
      zVal.f = accel.getCalculatedZ();
    }
    else if ((deviceType[deviceIdx] == I2CATOD) ||
             (deviceType[deviceIdx] == I2CDIGITAL) ||
             (deviceType[deviceIdx] == I2CCOUNTER))
    {
      Serial.print(" I2C");
      Serial.print(String(deviceIdx));
      // request  bytes from slave device
      // slave may send less than requested
      int availableBytes = Wire.requestFrom(deviceAddress[deviceIdx], 4);
      if(availableBytes < 4)
      {
        deviceValid[deviceIdx] = false;
        Serial.print(" --");
        continue;
      }
      deviceValid[deviceIdx] = true;
      Serial.print(" OK");
      for(int byteCnt = 0; byteCnt < availableBytes; byteCnt++) 
      {
        // receive a byte as character
        deviceVal[deviceIdx].c[byteCnt] = Wire.read(); 
      }
    }
  }
  Serial.println(" X");
}
//
// log new data
//
void LogData()
{
  bool firstValid = false;
  //
  // check all devices present
  //
  for(int deviceIdx = 0; deviceIdx < devicesPresent; deviceIdx++)
  {
    if(deviceValid[deviceIdx])
    {
      // add time before first valid output
      if(!firstValid)
      {
        LogTime(micros());
      }
      if(deviceType[deviceIdx] == GPS) 
      {
        LogGPSData(deviceIdx);
      }
      else if (deviceType[deviceIdx] == ACCEL)
      {
        LogAccelData(deviceIdx);
      }
      else if (deviceType[deviceIdx] == I2CATOD)
      {
        LogI2CAnalog(deviceIdx);
      }
      else if (deviceType[deviceIdx] == I2CDIGITAL)
      {
        LogI2CDigital(deviceIdx);
      }
      else if(deviceType[deviceIdx] == I2CCOUNTER)
      {
        LogI2CCounter(deviceIdx);
      }
    }
  }
}
//
// Log Accelerometer data
//
void LogAccelData(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("ACC");
  dataFile.write(channelID.c, 4);
  dataFile.write(xVal.c, 4);
  dataFile.write(yVal.c, 4);
  dataFile.write(zVal.c, 4);
  dataFile.flush();
}
//
// Log GPS data
//
void LogGPSData(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("GPS");
  dataFile.write(channelID.c, 4);
  dataFile.write(gpsLat.c, 4);
  dataFile.write(gpsLong.c, 4);
  dataFile.write(gpsSpeed.c, 4);
  dataFile.write(gpsHeading.c, 4);
  dataFile.write(gpsSats.c, 4);
  dataFile.flush();
}
//
// analog, digital, or counter channel on I2C bus
//
void LogI2CAnalog(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("A");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
//
//
void LogI2CDigital(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("D");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
//
//
void LogI2CCounter(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("C");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
//
//
void LogTime(unsigned long t)
{
  timestamp.ul = t;
  dataFile.write("T");
  dataFile.write(timestamp.c, 4);
}
void OpenDataFile()
{
  int logFileIdx = 0;
  logFileName = "LOG0000.YLG";
  Serial.println("Finding next data file...");
  Serial.print("Testing ");
  Serial.println(logFileName);
  while(SD.exists(logFileName))
  {
    logFileIdx++;
    logFileName = "LOG";
    if(logFileIdx < 10)
    {
      logFileName += "000" + String(logFileIdx);
    }
    else if(logFileIdx < 100)
    {
      logFileName += "00" + String(logFileIdx);
    }
    else if(logFileIdx < 1000)
    {
      logFileName += "0" + String(logFileIdx);
    }
    else if(logFileIdx < 10000)
    {
      logFileName += String(logFileIdx);
    }
    else
    {
       Serial.println("Delete some files!");
      while(true){}
    }
    logFileName += ".YLG";
    Serial.print("Testing ");
    Serial.println(logFileName);
  }
  Serial.print("Opening ");
  Serial.println(logFileName);
  dataFile = SD.open(logFileName, FILE_WRITE);
  /**/
}
void CloseDataFile()
{
  Serial.print("Closing ");
  Serial.println(logFileName);
  dataFile.close();
}
