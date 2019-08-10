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
#include <Wire.h>                                     // TWI I2C device
#include <SPI.h>                                      // SPI device
#include <SD.h>                                       // SD card support
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // QWIIC GPS device
#include <TinyGPS++.h>                                // GPS sentence translator
#include "SparkFun_MMA8452Q.h"                        // QWIIC accelerometer

//#define DEBUGSTR      // log to logger and serial port

// digital in/out defs
#define STARTBUTTON  9  // start/stop button
#define GPSPIN      10  // GPS Status LED
#define READYPIN    11  // Accel Status LED
#define LOGGERPIN   12  // Logger Status LED
#define CHIPSELECT  53  // SD card chip select pin (DUE)
//#define CHIPSELECT  8  // SD card chip select pin (UNO)

// device info
#define ANALOG1 0x08
#define ANALOG2 0x09
#define DEVICECOUNT 16
// device types
#define GPS 0
#define ACCEL 1
#define I2CATOD 2
#define I2CDIGITAL 3
#define I2CCOUNTER 4
//#define STDMODE     // 100K bus
#define FASTMODE    // 400K bus

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

I2CGPS myI2CGPS;        // hook gps object to the library
TinyGPSPlus gpsDecoder; // declare gps translator object
MMA8452Q accel;         // declare 3 axis accel MMA8452 object

// gps values
DataPacket gpsTime;
DataPacket gpsLat;
DataPacket gpsLong;
DataPacket gpsSpeed;
DataPacket gpsHeading;
DataPacket gpsSats;
// accelerometer values
DataPacket xVal;
DataPacket yVal;
DataPacket zVal;
// timestamp
DataPacket timestamp;
// channel ID packet
DataPacket channelID;
// device data
int devicesPresent = 0;               // total on I2C bus
bool devicePresent[DEVICECOUNT];      // true if device is present
bool deviceValid[DEVICECOUNT];        // true if last sample of device was valid
int deviceAddress[DEVICECOUNT];       // I2C address (if appropriate)
int deviceType[DEVICECOUNT];          // device type
DataPacket deviceVal[DEVICECOUNT];    // I2C devices value (GPS, ACCEL, IMU handled differently due to multiple values/sample

// log file info
File dataFile;
String logFileName;

// logger state
bool loggerRun = false;
bool waitmsg = true;
bool runmsg = true;

//
// initialization
//
void setup()
{
  loggerRun = false;  // not logging at startup, wait for button push
  // i/o pin config
  pinMode(GPSPIN, OUTPUT);
  pinMode(READYPIN, OUTPUT);
  pinMode(LOGGERPIN, OUTPUT);
  pinMode(STARTBUTTON, INPUT_PULLUP);
  // 'cylon eyes' at startup
  digitalWrite(READYPIN, LOW);
  digitalWrite(GPSPIN, LOW);
  digitalWrite(READYPIN, LOW);
  digitalWrite(LOGGERPIN, LOW);
  for(int i = 0; i < 20; i++)
  {
    digitalWrite(GPSPIN, LOW);
    digitalWrite(LOGGERPIN, HIGH);
    delay(10);
    digitalWrite(LOGGERPIN, LOW);
    digitalWrite(GPSPIN, HIGH);
    delay(10);
    digitalWrite(GPSPIN, LOW);
    digitalWrite(READYPIN, HIGH);
    delay(10);
    digitalWrite(READYPIN, LOW);
    digitalWrite(GPSPIN, HIGH);
    delay(10);
  }
  digitalWrite(READYPIN, LOW);
  digitalWrite(GPSPIN, LOW);
  digitalWrite(LOGGERPIN, LOW);
  // serial port initialization
  Serial.begin(9600);
  #ifdef DEBUGSTR
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  Serial.println("Team Yamura Logger 2.0");

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

  // all initialized, ready to go!
  Serial.println("Logger ready!");
  digitalWrite(READYPIN, HIGH);
}
//
// main loop - check for start/stop
// if started, check sensors and log data
// if stopped, look for data download request
//
void loop()
{
  CheckStart();
  if(loggerRun)
  {
    CheckSensors();
    LogData();
  }
  else
  {
    // TODO - check for request to upload data
  }
}
//
// look for start/stop button press
//
void CheckStart()
{
  // read button state
  int buttonVal = digitalRead(STARTBUTTON);
  // pressed, was running. stop and close file
  if((buttonVal == LOW) && (loggerRun == true))
  {
    digitalWrite(LOGGERPIN, LOW);
    Serial.println("Stop");
    CloseDataFile();
    loggerRun = false;
    runmsg = true;
    delay(500);
  }
  // pressed, was stop. start and open next file
  else if((buttonVal == LOW) && (loggerRun == false))
  {
    digitalWrite(LOGGERPIN, HIGH);
    Serial.println("Start");
    OpenDataFile();
    loggerRun = true;
    waitmsg = true;
    delay(500);
  }
  // released
  else if(buttonVal == HIGH)
  {
    // not running, wait message not already output
    // show waiting message
    if ((loggerRun == false) && (waitmsg))
    {
      Serial.println("waiting for start...");
      waitmsg = false;
    }
    // running, run message not already output
    // show running message
    else if  ((loggerRun == true) && (runmsg))
    {
      Serial.println("running");
      runmsg = false;
    }
  }
}
//
// check all configured sensors
// if valid, store data and set valid flag true
//
void CheckSensors()
{
  Serial.print("check sensors ");
  // no devices valid yet
  for(int deviceIdx =0; deviceIdx< devicesPresent; deviceIdx++)
  {
    deviceValid[deviceIdx] = false;
  }
  //
  // check all devices present
  //
  for(int deviceIdx =0; deviceIdx< devicesPresent; deviceIdx++)
  {
    if(deviceType[deviceIdx] == GPS)
    {
      Serial.print(" GPS ");
      // do this read in a loop of all channels defined
      // check GPS
      while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
      {
        gpsDecoder.encode(myI2CGPS.read()); //Feed the GPS parser
      }
      // invalid data from GPS - skip
      if(!gpsDecoder.time.isUpdated())
      {
        Serial.print("--");
      }
      // valid, save
      else
      {
        deviceValid[deviceIdx] = true;
        // satellites in view
        gpsSats.ul = gpsDecoder.satellites.value();
        // location
        gpsLat.f = gpsDecoder.location.lat();
        gpsLong.f = gpsDecoder.location.lng();
        gpsSpeed.f = gpsDecoder.speed.mph();
        // heading
        gpsHeading.f = gpsDecoder.course.deg();
        Serial.print("OK");
      }
    }
    else if (deviceType[deviceIdx] == ACCEL)
    {
      Serial.print(" ACC ");
      // invalid data from accelerometer - skip
      if (!accel.available()) 
      {
        Serial.print("--");
      }
      // valid - save
      else
      {
        Serial.print("OK");
        deviceValid[deviceIdx] = true;
        xVal.f = accel.getCalculatedX();
        yVal.f = accel.getCalculatedY();
        zVal.f = accel.getCalculatedZ();
      }
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
        Serial.print(" --");
        continue;
      }
      else
      {
        deviceValid[deviceIdx] = true;
        Serial.print(" OK");
        for(int byteCnt = 0; byteCnt < availableBytes; byteCnt++) 
        {
          // receive a byte as character
          deviceVal[deviceIdx].c[byteCnt] = Wire.read(); 
        }
      }
    }
  }
  Serial.println("");
}
//
// log latest valid data
//
void LogData()
{
  // log time
  LogTime(micros());
  //
  // check all devices present
  //
  for(int deviceIdx = 0; deviceIdx < devicesPresent; deviceIdx++)
  {
    if(deviceValid[deviceIdx])
    {
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
// analog channel on I2C bus
//
void LogI2CAnalog(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("A2D");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
// digital channel on I2C bus
//
void LogI2CDigital(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("DIG");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
// counter channel on I2C bus
//
void LogI2CCounter(int deviceIdx)
{
  channelID.ul = deviceIdx;
  dataFile.print("CNT");
  dataFile.write(channelID.c, 4);
  dataFile.write(deviceVal[deviceIdx].c, 4);
  dataFile.flush();
}
//
// log time value
//
void LogTime(unsigned long t)
{
  timestamp.ul = t;
  dataFile.write("T");
  dataFile.write(timestamp.c, 4);
}
//
// find next available log file and open it
// if there are too many files, quit
//
void OpenDataFile()
{
  int logFileIdx = 0;
  logFileName = "LOG0000.YLG";
  Serial.print("Finding next data file");
  while(SD.exists(logFileName))
  {
    Serial.print(".");
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
      digitalWrite(READYPIN, LOW);
      while(true){}
    }
    logFileName += ".YLG";
  }
  Serial.print(" opening ");
  Serial.println(logFileName);
  dataFile = SD.open(logFileName, FILE_WRITE);
}
//
// close the open data file
//
void CloseDataFile()
{
  Serial.print("Closing ");
  Serial.println(logFileName);
  dataFile.close();
}
