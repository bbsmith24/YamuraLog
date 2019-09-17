/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/
#include <SD.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

//#define LINEAR
#define ACCEL
#define CHIPSELECT 8

BNO080 myIMU;
File dataFile;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  //Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
//  //=================================
//  delay(100); //  Wait for BNO to boot
//  // Start i2c and BNO080
//  Wire.flush();   // Reset I2C
//  IMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
//  Wire.begin(4, 5); 
//  Wire.setClockStretchLimit(4000);
//  //=================================

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  SD.begin(CHIPSELECT);
  SD.remove("testIMU.txt");
  dataFile = SD.open("testIMU.txt", FILE_WRITE);

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  dataFile.println(F("Rotation vector enabled"));
  
  #ifdef ACCEL
  myIMU.enableAccelerometer(50); //Send data update every 50ms
  Serial.println(F("Accelerometer  enabled"));
  Serial.println(F("Output in form i j k real accuracy x y z"));
  
  dataFile.println(F("Accelerometer  enabled"));
  dataFile.println(F("Output in form i j k real accuracy x y z"));
  #endif

  #ifdef LINEAR
  myIMU.enableLinearAccelerometer(50); //Send data update every 50ms
  Serial.println(F("Linear Accelerometer  enabled"));
  Serial.println(F("Output in form i j k real accuracy xl yl zl accuracy"));

  dataFile.println(F("Linear Accelerometer  enabled"));
  dataFile.println(F("Output in form i j k real accuracy xl yl zl accuracy"));
  #endif

  dataFile.flush();
}

void loop()
{
  
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    #ifdef ACCEL
    float x = myIMU.getAccelX();
    float y = myIMU.getAccelY();
    float z = myIMU.getAccelZ();
    #endif

    #ifdef LINEAR
    float xl = myIMU.getLinAccelX();
    float yl = myIMU.getLinAccelY();
    float zl = myIMU.getLinAccelZ();
    byte linAccuracy = myIMU.getLinAccelAccuracy();
    #endif

    Serial.print(micros());
    Serial.print(F("\t"));
    Serial.print(quatI, 2);
    Serial.print(F("\t"));
    Serial.print(quatJ, 2);
    Serial.print(F("\t"));
    Serial.print(quatK, 2);
    Serial.print(F("\t"));
    Serial.print(quatReal, 2);
    Serial.print(F("\t"));
    Serial.print(quatRadianAccuracy, 2);

    dataFile.print(quatI, 2);
    dataFile.print(F("\t"));
    dataFile.print(quatJ, 2);
    dataFile.print(F("\t"));
    dataFile.print(quatK, 2);
    dataFile.print(F("\t"));
    dataFile.print(quatReal, 2);
    dataFile.print(F("\t"));
    dataFile.print(quatRadianAccuracy, 2);


    #ifdef ACCEL
    Serial.print(F("\t"));
    Serial.print(x, 4);
    Serial.print(F("\t"));
    Serial.print(y, 4);
    Serial.print(F("\t"));
    Serial.print(z, 4);

    dataFile.print(F("\t"));
    dataFile.print(x, 4);
    dataFile.print(F("\t"));
    dataFile.print(y, 4);
    dataFile.print(F("\t"));
    dataFile.print(z, 4);
    #endif
    
    #ifdef LINEAR
    Serial.print(F("\t"));
    Serial.print(xl, 4);
    Serial.print(F("\t"));
    Serial.print(yl, 4);
    Serial.print(F("\t"));
    Serial.print(zl, 4);
    Serial.print(F("\t"));
    Serial.print(linAccuracy, 4);
    
    dataFile.print(F("\t"));
    dataFile.print(xl, 4);
    dataFile.print(F("\t"));
    dataFile.print(yl, 4);
    dataFile.print(F("\t"));
    dataFile.print(zl, 4);
    dataFile.print(F("\t"));
    dataFile.print(linAccuracy, 4);
    #endif

    Serial.println();
    dataFile.println();
    dataFile.flush();
  }
}
