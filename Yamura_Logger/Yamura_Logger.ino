/*
*/

#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"   // Use Library Manager to install Sparkfun QWIIC Openlog
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // Use Library Manager to install Sparkfun QWIIC GPS or from https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include "SparkFun_MMA8452Q.h"                        // Use Library Manager to install Sparkfun Sparkfun MMA8452Q or from: http://librarymanager/All#SparkFun_MMA8452Q
#include <TinyGPS++.h>                                // From: https://github.com/mikalhart/TinyGPSPlus

I2CGPS myI2CGPS;     // hook gps object to the library
TinyGPSPlus gps;     // declare gps object
OpenLog myLog;       // declare logger object
MMA8452Q accel;      // declare 3 axis accel MMA8452 object
char outStr[256];
String xValStr;
String yValStr;
String zValStr;
String gpsLatStr;
String gpsLongStr;
String gpsSpeedStr;
String gpsHeadingStr;
//
int gpsDay;
int gpsMonth;
int gpsYear;
int gpsHour;
int gpsMinute;
int gpsSecond;
int gpsCentisecond;
int gpsSats;
bool loggerRun = true;
#define gpsPin 10    //GPS Status LED
#define  accelPin 11  //Accel Status LED
#define  loggerPin 12 //Logger Status LED
#define  button 2 //Logger Status LED
#define RedBoardT
//
//
//
void printBoth(char* s)
{
  if(loggerRun)
  {
    myLog.print(s);
    myLog.syncFile();
  }
  int buttonVal = digitalRead(button);
  if((buttonVal == LOW) && (loggerRun == true))
  {
    loggerRun = false;
    digitalWrite(loggerPin, LOW);
    myLog.println("Stop");
    myLog.syncFile();
    delay(1000);
  }
  else if((buttonVal == LOW) && (loggerRun == false))
  {
    loggerRun = true;
    digitalWrite(loggerPin, HIGH);
    myLog.println("Start");
    myLog.syncFile();
    delay(1000);
  }
}
//
//
//
void printlnBoth(char* s)
{
  int buttonVal = digitalRead(button);
  if(loggerRun)
  {
    myLog.println(s);
    myLog.syncFile();
  }
  if((buttonVal == LOW) && (loggerRun == true))
  {
    loggerRun = false;
    digitalWrite(loggerPin, LOW);
    myLog.println("Stop");
    myLog.syncFile();
    delay(1000);
  }
  else if((buttonVal == LOW) && (loggerRun == false))
  {
    loggerRun = true;
    digitalWrite(loggerPin, HIGH);
    myLog.println("Start");
    
    delay(1000);
  }
}
//
//
//
void setup()
{
  pinMode(gpsPin, OUTPUT);
  pinMode(accelPin, OUTPUT);
  pinMode(loggerPin, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  loggerRun = false;
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(gpsPin, HIGH);
    delay(100);
    digitalWrite(accelPin, HIGH);
    delay(100);
    digitalWrite(gpsPin, LOW);
    digitalWrite(loggerPin, HIGH);
    delay(100);
    digitalWrite(accelPin, LOW);
    delay(100);
    digitalWrite(loggerPin, LOW);
  }
  #ifdef RedBoardT
  SerialUSB.begin(115200);
  #else
  Serial.begin(115200);
  #endif
  //Initialize I2C
  Wire.begin(); 
  
  // open logger
  myLog.begin();
  /*  
  while ( == false)
  {
    sprintf(outStr, "Logger failed to respond. Please check wiring and memory card\n");
    printlnBoth(outStr);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  */
  digitalWrite(gpsPin, LOW);
  digitalWrite(accelPin, LOW);
  digitalWrite(loggerPin, LOW);
 
  // initialize GPS
  if (myI2CGPS.begin() == false)
  {
    sprintf(outStr, "GPS failed to respond. Please check wiring");
    printlnBoth(outStr);
    while (1); //Freeze!
  }
  sprintf(outStr, "GPS found");
  printlnBoth(outStr);

  // 10Hz sampling
  //myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(220, ",100"));
  // 1Hz sampling
  myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(220, ",1000"));
  
  // initialize accelerometer
  if(accel.begin() == false)
  {
    sprintf(outStr, "Accelerometer failed to respond. Please check wiring");
    printlnBoth(outStr);
    while (1); //Freeze!
  }
  sprintf(outStr, "Accelerometer found");
  printlnBoth(outStr);
  
  sprintf(outStr, "Team Yamura data logger");
  printlnBoth(outStr);
  #ifdef RedBoardT
  SerialUSB.println(outStr);
  #else
  Serial.println(outStr);
  #endif
  /*
  myLog.searchDirectory("*.txt"); //Give me all the txt files in the directory
  String fileName = myLog.getNextDirectoryItem();
  while (fileName != "") //getNextDirectoryItem() will return "" when we've hit the end of the directory
  {
    Serial.println(fileName);
    fileName = myLog.getNextDirectoryItem();
  }
  */
  // all initialized, ready to go!
}
//
//
//
void loop()
{
  //if(loggerRun)
  //{
    while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
    {
      gps.encode(myI2CGPS.read()); //Feed the GPS parser
    }

    if (gps.time.isUpdated()) //Check to see if new GPS info is available
    {
      displayGPSInfo();
    }
    else
    {
      displayAccelInfo();
    }
  //}
  /*
  if (Serial.available())
  {
    loggerRun = !loggerRun;
    if(loggerRun)
    {
      sprintf(outStr, "Start");
      printlnBoth(outStr);
      Serial.println(outStr);
    }
    else
    {
      sprintf(outStr, "Stop");
      printlnBoth(outStr);
      Serial.println(outStr);
    }
    while(Serial.available())
    {
      byte incoming = Serial.read();
    }
  }
  */
}
//
//Display new GPS info
//
void displayAccelInfo()
{
  if (accel.available()) 
  {
    digitalWrite(accelPin, HIGH);
    xValStr = String(accel.getCalculatedX());
    yValStr = String(accel.getCalculatedY());
    zValStr = String(accel.getCalculatedZ());
  }
  else
  {
    digitalWrite(accelPin, LOW);
    xValStr = String(0);
    yValStr = String(0);
    zValStr = String(0);
  }
  //              accel x/y/z
  sprintf(outStr, "%ld %s\t%s\t%s", micros(), xValStr.c_str(), yValStr.c_str(), zValStr.c_str());
  printlnBoth(outStr);
}
//
//Display new GPS info
//
void displayGPSInfo()
{
  gpsDay = 0;
  gpsMonth = 0;
  gpsYear =  0;
  gpsHour = 0;
  gpsMinute = 0;
  gpsSecond = 0;
  gpsCentisecond = 0;
  gpsSats = gps.satellites.value();
  
  if (gps.time.isValid())
  {
    gpsDay = gps.date.day();
    gpsMonth = gps.date.month();
    gpsYear =  gps.date.year();
    gpsHour = gps.time.hour();
    gpsMinute = gps.time.minute();
    gpsSecond = gps.time.second();
    gpsCentisecond = gps.time.centisecond();
  }
  // location
  if (gps.location.isValid())
  {
    digitalWrite(gpsPin, HIGH);
    gpsLatStr = String(gps.location.lat(),6);
    gpsLongStr = String(gps.location.lng(),6);
  }
  else 
  {
    digitalWrite(gpsPin, LOW);
    gpsLatStr = String(0.0,6);
    gpsLongStr = String(0.0,6);
  }
  // speed
  if(gps.speed.isValid())
  {
    gpsSpeedStr = String(gps.speed.mph(),2);
  }
  else
  {
    gpsSpeedStr = String(0.0,2);
  }
  // heading
  if(gps.course.isValid())
  {
    gpsHeadingStr = String(gps.course.deg(),6);
  }
  else
  {
    gpsHeadingStr = String(0.0,6);
  }
  if (accel.available()) 
  {   
    digitalWrite(accelPin, HIGH);
    xValStr = String(accel.getCalculatedX(), 3);
    yValStr = String(accel.getCalculatedY(), 3);
    zValStr = String(accel.getCalculatedZ(), 3);
  }
  else
  {
    digitalWrite(accelPin, LOW);
    xValStr = String(0.0, 3);
    yValStr = String(0.0, 3);
    zValStr = String(0.0, 3);
  }
  //gpsLatStr = String(gpsLat,6);
  //gpsLongStr = String(gpsLong,6);
  //gpsSpeedStr = String(gpsSpeed,3);
  //gpsHeadingStr = String(gpsHeading,6);
  //               date            time                pos    spd  hdg sats accel x/y/z
  sprintf(outStr, "%ld %02d/%02d/%04d\t%02d:%02d:%02d.%02d\t%s\t%s\t%s\t%s\t%d\t%s\t%s\t%s", micros(), gpsMonth, gpsDay, gpsYear, 
                                                                                            gpsHour, gpsMinute, gpsSecond, gpsCentisecond, 
                                                                                            gpsLatStr.c_str(), gpsLongStr.c_str(),
                                                                                            gpsSpeedStr.c_str(),
                                                                                            gpsHeadingStr.c_str(),
                                                                                            gpsSats,
                                                                                            xValStr.c_str(), yValStr.c_str(), zValStr.c_str());
  printlnBoth(outStr);
  myLog.syncFile();
}
