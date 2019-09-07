/*
 UNO/Redboard
 * microSD sheild connects to SPI
 * 
 *         Uno      RedboardT
 ** MOSI - pin 11*  SPI/MOSI
 ** MISO - pin 12   SPI/MISO
 ** CLK -  pin 13   SPI/CLK
 ** CS -   pin  8   8

  start/stop D9
  LEDs       D5-D7
    
  A2D, digital, counter, GPS, accelerometer, IMU I2C sensors on QWIIC bus
*/
// for Redboard Turbo serial com
#define Serial SerialUSB

#include <Wire.h>                                     // TWI I2C device
#include <SPI.h>
#include <SD.h>
#include "SparkFun_MMA8452Q.h"                        // QWIIC accelerometer
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // QWIIC GPS device

// SD card
#define CHIPSELECT 8

// debug messages to serial
#define DEBUGSTR

// I2C info (pick 1)
#define STDMODE   100000 // 100K
#define FASTMODE  400000 // 400K

// device info
#define ANALOG1 0x08

// 4 (1 channel), 8 (2 channel), 12 (3 channel) A2D message length
//#define A2D_MSG_BLOCK 1
//#define A2D_MSG_BLOCK 2
#define A2D_MSG_BLOCK 4

// user i/o
#define GPSPIN      2  // GPS Status LED
#define READYPIN    3  // Accel Status LED
#define LOGGERPIN   4  // Logger Status LED
#define STARTBUTTON 5     // start/stop button

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

union IntPacket
{
  int  intVal;
  char charBuf[4];
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
String logFileName = "LOG0000.YLG";

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
  Serial.begin(115200);  
  #ifdef DEBUGSTR
  while (!Serial) { ; }   // wait for serial port to connect. Needed for native USB port only
  #endif

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
  //Serial.println("I2C clock " + FASTMODE);
  Wire.setClock(FASTMODE);

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
  // timestamp
  LogPrint("T");
  i2cData.ul[0] = micros();
  LogWrite(i2cData.c[0], 4);

  // request  bytes from slave device
  // slave may send less than requested
  // this is the 4 channel cockpit sensor pack
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
    }
  }
  // accelerometer
  if (accel.available()) 
  {
    LogPrint("ACC");
    i2cData.ul[0] = 1;
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedX();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedY();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
    i2cData.f[0] = accel.getCalculatedZ();
    LogWrite(i2cData.c[0], 4);//dataFile.write(i2cData.c[0], 4);
  }
  // GPS
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
  // push data to SD card
  dataFile.flush();
}
//
// find next available log file and open it
// if there are too many files, quit
//
void OpenDataFile()
{
  IntPacket logFileIdx;
  //int logFileIdx = 0;
  logFileIdx.intVal = 0;
  File counterFile;
  if(SD.exists("logCount.ini"))
  {
    counterFile = SD.open("logCount.ini", FILE_READ);
    counterFile.read(logFileIdx.charBuf, 4);
    counterFile.close();   
    logFileIdx.intVal = logFileIdx.intVal; 
    SD.remove("logCount.ini");
 
    #ifdef DEBUGSTR
    Serial.print("Log index from  file ");
    Serial.println(logFileIdx.intVal);
    #endif
  }
  do
  //while(SD.exists(logFileName))
  {
    logFileIdx.intVal++;
    logFileName = "LOG";
    if(logFileIdx.intVal < 10)
    {
      logFileName += "000" + String(logFileIdx.intVal);
    }
    else if(logFileIdx.intVal < 100)
    {
      logFileName += "00" + String(logFileIdx.intVal);
    }
    else if(logFileIdx.intVal < 1000)
    {
      logFileName += "0" + String(logFileIdx.intVal);
    }
    else if(logFileIdx.intVal < 10000)
    {
      logFileName += String(logFileIdx.intVal);
    }
    else
    {
      //Serial.println("Delete some files!");
      while(true){}
    }
    logFileName += ".YLG";
  } while(SD.exists(logFileName));
  #ifdef DEBUGSTR
  Serial.print("Opening ");
  Serial.println(logFileName);
  #endif
  counterFile = SD.open("logCount.ini", FILE_WRITE);
  counterFile.write(logFileIdx.charBuf, 4);
  counterFile.close();    

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
    #ifdef DEBUGSTR
    Serial.println("Stop");
    #endif
    digitalWrite(LOGGERPIN, LOW);
    CloseDataFile();
    logging = false;
    runmsg = true;
    delay(500);
  }
  // logging stopped
  else if(logging == false)
  {
    // button pressed. open next file and start logging
    if(buttonVal == LOW)
    {
      digitalWrite(LOGGERPIN, HIGH);
      #ifdef DEBUGSTR
      Serial.println("Start");
      #endif

      OpenDataFile();
      logging = true;
      waitmsg = true;
      delay(500);
      #ifdef DEBUGSTR
      Serial.println("Logging");
      #endif
    }
    // no button press - check for serial input from computer
    else
    {
      if (Serial.available() > 0) 
      {
        int incomingByte = 0;
        while(Serial.available() > 0) 
        {
          incomingByte = Serial.read();
        }
        // manage data files on card
        ManageDataFiles();
      }
    }
  }
}
void ManageDataFiles()
{
  int fileModeByte;
  int incomingByte;
  // all log files are in root folder
  File root;
  File entry;
  root = SD.open("/");
  
  while(true)
  {
    entry = root.openNextFile();
    if(!entry)
    {
      break;
    }
    // send file name to PC
    logFileName = entry.name();
    String testName = logFileName;
    if((testName.endsWith("YLG")) || 
       (testName.endsWith("TXT")) || 
       (testName.endsWith("ylg")) ||
       (testName.endsWith("txt")))
    {
      Serial.println(logFileName);
      // wait for response 
      // U = upload
      // D = upload then delete
      // X = delete
      // L = list
      // ignore anything else (skips the file)
      while(Serial.available() == 0) 
      { ; }
      fileModeByte = Serial.read();
      // upload or upload and delete
      if((fileModeByte == 'U') || (fileModeByte == 'D'))
      {
        // re-open the file for reading:
        dataFile = SD.open(logFileName);
        // read from the file until there's nothing else in it:
        while (dataFile.available()) 
        {
          Serial.write(dataFile.read());
        }
        // close the file:
        dataFile.close();
        // wait for handshake at end
        while(Serial.available() == 0) 
        { ; }
        incomingByte = Serial.read();
      }
      // delete after upload, or clearing all
      if((fileModeByte == 'D') || (fileModeByte == 'X'))
      {
        dataFile.close();
        SD.remove(logFileName);
      }
      if(fileModeByte == 'L')
      {
        dataFile.close();
      }
    }
  } 
  root.close();
}
