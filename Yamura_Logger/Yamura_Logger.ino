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

// 4 (1 channel), 8 (2 channel), 12 (3 channel) A2D message length
//#define A2D_MSG_BLOCK 1
//#define A2D_MSG_BLOCK 2
#define A2D_MSG_BLOCK 3

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
  float f[A2D_MSG_BLOCK];          // 4 bytes per MSG_BLOCK
  unsigned long ul[A2D_MSG_BLOCK]; // 4 bytes per MSG_BLOCK
  long sl[A2D_MSG_BLOCK];          // 4 bytes per MSG_BLOCK
  word uw[A2D_MSG_BLOCK][2];       // 2 x 2 bytes per MSG_BLOCK 
  int  sw[A2D_MSG_BLOCK][2];       // 2 x 2 bytes per MSG_BLOCK         
  char c[A2D_MSG_BLOCK][4];        // 4 bytes per MSG_BLOCK
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
  /*
  Supported NMEA Sentences
  0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude    - SKIPPED by viewer
  1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence  - PARSED by viewer
  2 NMEA_SEN_VTG, // GPVTG interval - Course over Ground and Ground Speed         - PARSED by viewer
  3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data                                - PARSED by viewer (but redundant mostly except for satellite count)
  4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites             - SKIPPED by viewer
  5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View                     - SKIPPED by viewer
  6-16 //Reserved
  17 NMEA_SEN_ZDA, // GPZDA interval – Time & Date                                - SKIPPED by viewer
  18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status                      - SKIPPED by viewer
  */
  // set to return only RMC and VTG sentences - bare min info
  //String configString;
  //configString = myI2CGPS.createMTKpacket(314, ",0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
  //myI2CGPS.sendMTKpacket(configString);
  myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(314, ",0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));

  Serial.println("Ready!");
  digitalWrite(READYPIN, HIGH);
}

void loop() 
{
  CheckStart();
  LogData();
}
void LogPrint(char c)
{
  if(logging == true)
  {
    dataFile.print(c);
  }
}
void LogPrint(String s)
{
  if(logging == true)
  {
    dataFile.print(s);
  }
}
void LogWrite(char* b, int len)
{
  if(logging == true)
  {
    dataFile.write(b, len);
  }
}
void LogData()
{
  //dataFile.print("T");
  LogPrint("T");
  i2cData.ul[0] = micros();
  //dataFile.write(i2cData.c[0], 4);
  LogWrite(i2cData.c[0], 4);

  // request  bytes from slave device
  // slave may send less than requested
  int a2dcnt = Wire.requestFrom(ANALOG1, A2D_MSG_BLOCK * 4);
  int byteIdx = 0;
  if(a2dcnt == (A2D_MSG_BLOCK * 4))
  {
    DataPacket channelNum;
    for(int msgCnt = 0; msgCnt < A2D_MSG_BLOCK; msgCnt++) 
    {
      for(int byteCnt = 0; byteCnt < 4; byteCnt++) 
      {
        // receive a byte as character
        i2cData.c[0][byteCnt] = Wire.read(); 
      }
      LogPrint("A2D");
      channelNum.ul[0] = msgCnt;
      LogWrite(channelNum.c[0], 4);
      LogWrite(i2cData.c[0], 4);
      /*
       * 
      Serial.print("A2D");
      Serial.print(msgCnt);
      Serial.print(" ");
      Serial.print(i2cData.ul[0]);
      Serial.print(" ");
      Serial.print(i2cData.c[0]);
      Serial.print(" ");
      Serial.print(i2cData.c[1]);
      Serial.print(" ");
      Serial.print(i2cData.c[2]);
      Serial.print(" ");
      Serial.print(i2cData.c[3]);
      Serial.print(" ");
      */
    }
    //Serial.println("");
  }
  //else if (a2dcnt != (A2D_MSG_BLOCK * 4))
  //{
  //  Serial.println("received " + String(a2dcnt) + " bytes");  
  //}
  //Serial.print(" ACC ");
  if (accel.available()) 
  {
    LogPrint("ACC");//dataFile.print("ACC");
    i2cData.ul[0] = 1;
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedX();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedY();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedZ();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
  }

  //Serial.print(" GPS ");
  if(myI2CGPS.available()>0) //available() returns the number of new bytes available from the GPS module
  {
    char c;
    LogPrint("GPS");//dataFile.print("GPS");
    i2cData.ul[0] = 2;
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    // read NMEA sentence and write to log - easier to deal with on the viewer side
    while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
    {
      c = myI2CGPS.read();
      LogPrint(c);//dataFile.print(c);
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
