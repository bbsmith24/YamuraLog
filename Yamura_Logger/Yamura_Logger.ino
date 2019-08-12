/*
 UNO/Redboard

  start/stop D9
  LEDs       D5-D7
    
  A2D, digital, counter, GPS, accelerometer, IMU I2C sensors on QWIIC bus
*/
#include <Wire.h>                                     // TWI I2C device
#include <SPI.h>
#include <SD.h>
#include "SparkFun_MMA8452Q.h"                        // QWIIC accelerometer
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // QWIIC GPS device

// SD card
#define CHIPSELECT 8

// I2C info (pick 1)
#define STDMODE   // 100K
//#define FASTMODE  // 400K

// device info
#define ANALOG1 0x08

// 4 (1 channel) or 8 (2 channel) A2D message length
#define A2D_MSG_LEN 4
//#define A2D_MSG_LEN 8

// user i/o
#define STARTBUTTON 9     // start/stop button
#define GPSPIN      5  // GPS Status LED
#define READYPIN    6  // Accel Status LED
#define LOGGERPIN   7  // Logger Status LED

// function declarations
void OpenDataFile();
void CloseDataFile();
void CheckStart();
void LogData();

// union for i2c data
union DataPacket
{
  float f;          // 4 bytes
  unsigned long ul; // 4 bytes
  long sl;          // 4 bytes
  word uw[2];       // 2 x 2bytes           
  int  sw[2];       // 2 x 2bytes           
  char c[4];        // 4 bytes
};

// log file info
File dataFile;

DataPacket i2cData;
MMA8452Q accel;         // declare 3 axis accel MMA8452 object
I2CGPS myI2CGPS;        // hook gps object to the library
//TinyGPSPlus gpsDecoder; // declare gps translator object
bool logging = false;
bool waitmsg = true;
bool runmsg = true;

void setup() 
{
  pinMode(STARTBUTTON, INPUT_PULLUP);
  pinMode(GPSPIN, OUTPUT);
  pinMode(READYPIN, OUTPUT);
  pinMode(LOGGERPIN, OUTPUT);
  
  digitalWrite(GPSPIN, LOW);
  digitalWrite(READYPIN, LOW);
  digitalWrite(LOGGERPIN, LOW);
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // see if the card is present and can be initialized
  // mostly likely failure is no card, so flash all LEDs to let user know
  while (!SD.begin(CHIPSELECT)) 
  {
    digitalWrite(GPSPIN, LOW);
    digitalWrite(READYPIN, LOW);
    digitalWrite(LOGGERPIN, LOW);
    delay(100);
    digitalWrite(GPSPIN, HIGH);
    digitalWrite(READYPIN, HIGH);
    digitalWrite(LOGGERPIN, HIGH);
    delay(100);
  }
  Serial.println("SD Card OK");
  digitalWrite(GPSPIN, LOW);
  digitalWrite(READYPIN, LOW);
  digitalWrite(LOGGERPIN, LOW);

  Wire.begin(); 
  #ifdef STDMODE
  Serial.println("I2C clock 100K");
  Wire.setClock(100000); // standard mode
  #endif
  #ifdef FASTMODE
  Serial.println("I2C clock 400K");
  Wire.setClock(400000);  // fast mode
  #endif

  if(accel.begin() == false)
  {
    Serial.println("ACC failed");
  }
  else
  {
    Serial.println("ACC OK");
  }
  
  // initialize GPS
  if (myI2CGPS.begin() == false)
  {
    Serial.println("GPS failed");
    //while (1); //Freeze!
  }
  else
  {
    Serial.println("GPS OK");
    digitalWrite(GPSPIN, HIGH);
  }

  Serial.println("Ready!");
  digitalWrite(READYPIN, HIGH);
}

void loop() 
{
  CheckStart();
  if(logging == true)
  {
    LogData();
  }
}
void LogData()
{
  dataFile.print("T");
  i2cData.ul = micros();
  dataFile.write(i2cData.c, 4);

  // request  bytes from slave device
  // slave may send less than requested
  if(Wire.requestFrom(0x08, A2D_MSG_LEN) == A2D_MSG_LEN)
  {
    dataFile.print("A2D");
    i2cData.ul = 0;
    dataFile.write(i2cData.c, 4);
    for(int byteCnt = 0; byteCnt < A2D_MSG_LEN; byteCnt++) 
    {
      // receive a byte as character
      i2cData.c[byteCnt] = Wire.read(); 
    }
    dataFile.write(i2cData.c, A2D_MSG_LEN);
  }
  //Serial.print(" ACC ");
  if (accel.available()) 
  {
    dataFile.print("ACC");
    i2cData.ul = 1;
    dataFile.write(i2cData.c, 4);
    i2cData.f = accel.getCalculatedX();
    dataFile.write(i2cData.c, 4);
    i2cData.f = accel.getCalculatedY();
    dataFile.write(i2cData.c, 4);
    i2cData.f = accel.getCalculatedZ();
    dataFile.write(i2cData.c, 4);
  }

  //Serial.print(" GPS ");
  if(myI2CGPS.available()>0) //available() returns the number of new bytes available from the GPS module
  {
    char c;
    dataFile.print("GPS");
    i2cData.ul = 2;
    dataFile.write(i2cData.c, 4);
    // read NMEA sentence and write to log - easier to deal with on the viewer side
    while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
    {
      c = myI2CGPS.read();
      dataFile.print(c);
    }
  }
  dataFile.flush();
}
//
// find next available log file and open it
// if there are too many files, quit
//
void OpenDataFile()
{
  int logFileIdx = 0;
  String logFileName = "LOG0000.YLG";
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
      //Serial.println("Delete some files!");
      while(true){}
    }
    logFileName += ".YLG";
  }
  dataFile = SD.open(logFileName, FILE_WRITE);
}
//
// close the open data file
//
void CloseDataFile()
{
  dataFile.close();
}
//
// look for start/stop button press
//
void CheckStart()
{
  // read button state
  int buttonVal = digitalRead(STARTBUTTON);
  // pressed, was running. stop and close file
  if((buttonVal == LOW) && (logging == true))
  {
    digitalWrite(LOGGERPIN, LOW);
    CloseDataFile();
    logging = false;
    runmsg = true;
    delay(500);
  }
  // pressed, was stop. start and open next file
  else if((buttonVal == LOW) && (logging == false))
  {
    digitalWrite(LOGGERPIN, HIGH);
    OpenDataFile();
    logging = true;
    waitmsg = true;
    delay(500);
  }
}
