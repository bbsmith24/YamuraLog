/*****************************************************************
  LSM9DS1_Basic_I2C.ino
  SFE_LSM9DS1 Library Simple Example Code - I2C Interface
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: April 30, 2015
  https://github.com/sparkfun/LSM9DS1_Breakout

  The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
  accelerometer, gyroscope, and magnetometer. Very cool! Plus it
  functions over either SPI or I2C.

  This Arduino sketch is a demo of the simple side of the
  SFE_LSM9DS1 library. It'll demo the following:
  How to create a LSM9DS1 object, using a constructor (global
  variables section).
  How to use the begin() function of the LSM9DS1 class.
  How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
  How to calculate actual acceleration, rotation speed,
  magnetic field strength using the calcAccel(), calcGyro()
  and calcMag() functions.
  How to use the data from the LSM9DS1 to calculate
  orientation and heading.

  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
	LSM9DS1 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND
  (CSG, CSXM, SDOG, and SDOXM should all be pulled high.
  Jumpers on the breakout board will do this for you.)

  The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
  off the 3.3V rail! I2C pins are open-drain, so you'll be
  (mostly) safe connecting the LSM9DS1's SCL and SDA pins
  directly to the Arduino.

  Development environment specifics:
	IDE: Arduino 1.6.3
	Hardware Platform: SparkFun Redboard
	LSM9DS1 Breakout Version: 1.0

  This code is beerware. If you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!

  Distributed as-is; no warranty is given.
*****************************************************************/
#define defaultMaxWait 250

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SparkFun_ADS1015_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <SparkFunSX1509.h> // Include SX1509 library
#include "SparkFun_BNO080_Arduino_Library.h"
//#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
#include <SparkFun_I2C_GPS_Arduino_Library.h>         // QWIIC GPS device

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

ADS1015 adcSensor;

const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

BNO080 bno080IMU;

//SFE_UBLOX_GPS myGPS;
I2CGPS myI2CGPS;        // hook gps object to the library

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
// #define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
//#define PRINT_CALCULATED
#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
static unsigned long lastGPS = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
char c;

void setup()
{
  bool ready = false;
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  while(!ready)
  {
    ready = imu.begin();
    if(ready)
    {
      break;
    }
    Serial.println("Failed to communicate with LSM9DS1. Retry...");
    delay(500);
  }
  Serial.println("LSM9DS1 device found.");
  //if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  //{
  //  Serial.println("Failed to communicate with LSM9DS1.");
  //  Serial.println("Double-check wiring.");
  //  Serial.println("Default settings in this sketch will " \
  //                 "work for an out of the box LSM9DS1 " \
  //                 "Breakout, but may need to be modified " \
  //                 "if the board jumpers are.");
  //  while (1);
  //}
  ready = false;
  while(!ready)
  {
    ready = adcSensor.begin();
    if(ready)
    {
      break;
    }
    Serial.println("ADS1015 device not found. Retry...");
    delay(500);
  }
  Serial.println("ADS1015 device found.");
  adcSensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS); 

  ready = false;
  while(!ready)
  {
    ready = io.begin(SX1509_ADDRESS);
    if(ready)
    {
      break;
    }
    Serial.println("SX1509 device not found. Retry...");
    delay(500);
  }
  Serial.println("SX1509 device found.");
  io.pinMode(0, INPUT);
  io.pinMode(1, INPUT);
  io.pinMode(2, INPUT);
  io.pinMode(3, INPUT);

  ready = false;
  while(!ready)
  {
    ready = bno080IMU.begin();
    if(ready)
    {
      break;
    }
    Serial.println("BNO080 device not found. Retry...");
    delay(500);
  }
  Serial.println("BNO080 device found.");  
  //if (bno080IMU.begin() == false)
  //{
  //  Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
  //  while (1);
  //}
  //bno080IMU.enableRotationVector(50); //Send data update every 50ms
  bno080IMU.enableRotationVector(2); //Send data update every 5ms
  bno080IMU.enableAccelerometer(2); //Send data update every 5ms
  bno080IMU.enableLinearAccelerometer(2);
/*
  ready = false;
  while(!ready)
  {
    ready = myGPS.begin();
    if(ready)
    {
      break;
    }
    Serial.println(F("Ublox GPS not ready."));
    delay(500);
  }
  Serial.println(F("Ublox GPS ready."));
  
  myGPS.setNavigationFrequency(15);  //Set the number of nav solutions sent per second
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  myGPS.setAutoPVT(true);
  */
  ready = false;
  while (!ready)
  {
    ready = myI2CGPS.begin(Wire, 400000);
    if(ready)
    {
      break;
    }
    Serial.println(F("GPS not ready."));
    delay(500);
  }
  myI2CGPS.sendMTKpacket(myI2CGPS.createMTKpacket(314, ",0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
  String configString = myI2CGPS.createMTKpacket(220, ",100");
  myI2CGPS.sendMTKpacket(configString);
  lastPrint = millis();
}

void loop()
{
  Serial.print(millis()-lastPrint);
  lastPrint = millis();
  Serial.print("\t");
 
 
  /// Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    printGyro(true);  // Print "G: gx, gy, gz"
  }
  else
  {
    printGyro(false);
      Serial.print("--- --- ---");
  }
  Serial.print("\t");
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    printAccel(true); // Print "A: ax, ay, az"
  }
  else
  {
    printAccel(false); // Print "A: ax, ay, az"
  }
/*  Serial.print("\t");
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    printMag(true);   // Print "M: mx, my, mz"
  }
  else
  {
    printMag(false);   // Print "M: mx, my, mz"
  }
  */
  Serial.print("\t");

  uint16_t channel_A3 = adcSensor.getSingleEnded(3);
  Serial.print("A3 ");
  Serial.print(channel_A3);

  Serial.print(" D0-3 ");
  Serial.print(io.digitalRead(0));
  Serial.print(" ");
  Serial.print(io.digitalRead(1));
  Serial.print(" ");
  Serial.print(io.digitalRead(2));
  Serial.print(" ");
  Serial.print(io.digitalRead(3));
  
/*  if (bno080IMU.dataAvailable() == true)
  {
    float quatI = bno080IMU.getQuatI();
    float quatJ = bno080IMU.getQuatJ();
    float quatK = bno080IMU.getQuatK();
    float quatReal = bno080IMU.getQuatReal();
    float quatRadianAccuracy = bno080IMU.getQuatRadianAccuracy();

    float x = bno080IMU.getAccelX();
    float y = bno080IMU.getAccelY();
    float z = bno080IMU.getAccelZ();

    float xl = bno080IMU.getLinAccelX();
    float yl = bno080IMU.getLinAccelY();
    float zl = bno080IMU.getLinAccelZ();

    Serial.print(" accel ");
    Serial.print(x, 2);
    Serial.print("/");
    Serial.print(xl, 2);
    Serial.print("\t");
    Serial.print(y, 2);
    Serial.print("/");
    Serial.print(yl, 2);
    Serial.print("\t");
    Serial.print(z, 2);
    Serial.print("/");
    Serial.print(zl, 2);
    Serial.print("\t");


    Serial.print(" quant ");
    Serial.print(quatI, 2);
    Serial.print("\t");
    Serial.print(quatJ, 2);
    Serial.print("\t");
    Serial.print(quatK, 2);
    Serial.print("\t");
    Serial.print(quatReal, 2);
    Serial.print("\t");
    Serial.print(quatRadianAccuracy, 2);
  }
  else
  {
    Serial.print(" accel ");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print(" quant ");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
    Serial.print("\t");
    Serial.print("-----");
  }
  */
  if(myI2CGPS.available())
  {
    Serial.print("\tGPS\t");
    Serial.print(millis()-lastGPS);
    Serial.print("\t");
    lastGPS = millis();
    
    while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
    {
      c = myI2CGPS.read();
      //Serial.print(c, HEX);
    }
  }

  /*
  //bool dataReady = myGPS.checkUblox();
  bool dataReady = myGPS.getPVT();;
  //if(dataReady == true)
  {
    long latitude = myGPS.getLatitude();
    long longitude = myGPS.getLongitude();
    long altitude = myGPS.getAltitude();
    byte SIV = myGPS.getSIV();

    Serial.print(F("Lat: "));
    Serial.print(latitude);

    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.print(F(" SIV: "));
    Serial.print(SIV);
    Serial.print(" new ");
    Serial.print(dataReady ? "TRUE" : "FALSE");
  }
  
  else
  {
    Serial.print(F("Lat: "));
    Serial.print("----------");

    Serial.print(F(" Long: "));
    Serial.print("----------");
    Serial.print(F(" (degrees * 10^-7)"));

    Serial.print(F(" Alt: "));
    Serial.print("----------");
    Serial.print(F(" (mm)"));

    Serial.print(F(" SIV: "));
    Serial.print("----------");
  }
  */
  Serial.println(" ");

//  if ((lastPrint + PRINT_SPEED) < millis())
//  {
//    // Print the heading and orientation for fun!
//    // Call print attitude. The LSM9DS1's mag x and y
//    // axes are opposite to the accelerometer, so my, mx are
//    // substituted for each other.
//    printAttitude(imu.ax, imu.ay, imu.az,
//                  -imu.my, -imu.mx, imu.mz);
//    Serial.println();
//
//    lastPrint = millis(); // Update lastPrint time
//  }
}

void printGyro(bool updated)
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  //Serial.print("G: ");
  if(updated)
  {
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print("\t");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print("\t");
  Serial.print(imu.calcGyro(imu.gz), 2);
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print("\t");
  Serial.print(imu.gy);
  Serial.print("\t");
  Serial.print(imu.gz);
#endif
  }
  else
  {
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
  }
  Serial.print("\tdeg/s");
}

void printAccel(bool updated)
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //Serial.print("A: ");
  if(updated)
  {
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print("\t");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print("\t");
    Serial.print(imu.calcAccel(imu.az), 2);
#elif defined PRINT_RAW
    Serial.print(imu.ax);
    Serial.print("\t");
    Serial.print(imu.ay);
    Serial.print("\t");
    Serial.print(imu.az);
#endif
  }
  else
  {
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
 
  }
  Serial.print("\tg");
}

void printMag(bool updated)
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  //Serial.print("M: ");
  if(updated)
  {
#ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcMag helper function to convert a raw ADC value to
    // Gauss. Give the function the value that you want to convert.
    Serial.print(imu.calcMag(imu.mx), 2);
    Serial.print("\t");
    Serial.print(imu.calcMag(imu.my), 2);
    Serial.print("\t");
    Serial.print(imu.calcMag(imu.mz), 2);
#elif defined PRINT_RAW
    Serial.print(imu.mx);
    Serial.print("\t");
    Serial.print(imu.my);
    Serial.print("\t");
    Serial.print(imu.mz);
#endif
  }
  else
  {
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
    Serial.print("\t");
    Serial.print("------");
  }
  Serial.print("\tgauss");
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
