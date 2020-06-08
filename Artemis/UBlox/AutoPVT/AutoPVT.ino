/*
  Configuring the GPS to automatically send position reports over I2C
*/

#include <Wire.h>                           //Needed for I2C to GPS
#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
#include <SparkFunSX1509.h>                 // Include SX1509 library
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

SerLCD lcd; // Initialize the library with default I2C address 0x72

unsigned int lastUpdate = 0;
unsigned int deltaTime = 0;
// GPS 
SFE_UBLOX_GPS gpsChannel;
long latitude = 0;
long longitude = 0;
long altitude = 0;
long groundSpeed = 0;
long heading = 0;
byte SIV = 0;
int PDOP = 0;
uint16_t gpsYear; 
uint8_t gpsMonth; 
uint8_t gpsDate; 
uint8_t gpsHour; 
uint8_t gpsMinute; 
uint8_t gpsSecond; 
uint8_t gpsMillisecond; 
// SX1509 digital IO
SX1509 digitalChannels; // Create an SX1509 object to be used throughout


void setup()
{
  Serial.begin(115200);

  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setBacklight(0, 255, 0); //Set backlight to bright white
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("Hello, World!");

  while (!Serial); //Wait for user to open terminal
  
  Serial.println("SparkFun Ublox Example");
  Wire.begin();
  //Connect to the Ublox module using Wire port
  bool isReady = gpsChannel.begin();
  if (!isReady) 
  {
    delay(500);
    Serial.println(F("Ublox GPS not detected at default I2C address. Retrying..."));
    isReady = gpsChannel.begin();
  }
  gpsChannel.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  gpsChannel.setNavigationFrequency(40); //Produce x solutions per second
  gpsChannel.setAutoPVT(true);           //Tell the GPS to "send" each solution
  gpsChannel.saveConfiguration();        //Save the current settings to flash and BBR
  lcd.clear(); 
  lcd.print("GPS OK ");
  
  isReady = digitalChannels.begin();
  {
    delay(500);
    Serial.println(F("SX1509 digital IO not detected at default I2C address. Retrying..."));
    isReady = digitalChannels.begin();
  }
  lcd.print("DIGITAL OK ");

  digitalChannels.pinMode(0, INPUT);
  digitalChannels.pinMode(1, INPUT);
  digitalChannels.pinMode(2, INPUT);
  digitalChannels.pinMode(3, INPUT);
  
}

void loop()
{
  if(UpdateGPS()) 
  {
    Serial.println("");
  }
  if(UpdateSensors())
  {
    Serial.println("");
  }
  
}
//
bool UpdateGPS()
{
    // Calling getPVT returns true if there actually is a fresh navigation solution available.
  if (gpsChannel.getPVT())
  {
    latitude = gpsChannel.getLatitude();
    longitude = gpsChannel.getLongitude();
    groundSpeed = gpsChannel.getGroundSpeed();     //Returns speed in mm/s
    heading = gpsChannel.getHeading();
    gpsYear = gpsChannel.getYear();
    gpsMonth = gpsChannel.getMonth();
    gpsDate = gpsChannel.getDay();
    gpsHour = gpsChannel.getHour();
    gpsMinute = gpsChannel.getMinute();
    gpsSecond = gpsChannel.getSecond();
    gpsMillisecond = gpsChannel.getMillisecond();
    deltaTime = millis() - lastUpdate;
    lastUpdate = millis();

    if(deltaTime < 10)
    {
      Serial.print("   ");
    }
    else if(deltaTime < 100)
    {
      Serial.print("  ");
    }
    else if(deltaTime < 1000)
    {
      Serial.print(" ");
    }
    Serial.print(deltaTime);
    
    //altitude = gpsChannel.getAltitude();
    SIV = gpsChannel.getSIV();
    //PDOP = gpsChannel.getPDOP();
    Serial.print("\t ");
    Serial.print(gpsYear);
    Serial.print("/");
    if(gpsMonth < 10)
    {
      Serial.print("0");
    }
    Serial.print(gpsMonth);
    Serial.print("/");
    if(gpsDate < 10)
    {
      Serial.print("0");
    }
    Serial.print(gpsDate);
    Serial.print(" ");
    if(gpsHour < 10)
    {
      Serial.print("0");
    }
    Serial.print(gpsHour);
    Serial.print(":");
    if(gpsMinute < 10)
    {
      Serial.print("0");
    }
    Serial.print(gpsMinute);
    Serial.print(":");
    if(gpsSecond < 10)
    {
      Serial.print("0");
    }
    Serial.print(gpsSecond);
    Serial.print(".");
    if(gpsMillisecond < 10)
    {
      Serial.print("00");
    }
    else if(gpsMillisecond < 100)
    {
      Serial.print("0");
    }
    Serial.print(gpsMillisecond);
    Serial.print("\t");

    Serial.print("\tLat/Long ");
    Serial.print(latitude);
    Serial.print("\t");
    Serial.print(longitude);
    
    Serial.print("\tSpeed ");
    Serial.print(groundSpeed);
    Serial.print("\tHeading ");
    Serial.print(heading);

    //Serial.print(F("\tAlt "));
    //Serial.print(altitude);

    Serial.print(F("\tSIV "));
    Serial.print(SIV);

    //Serial.print(F("\tPDOP: ")); //Positional dilution of precision
    //Serial.print(PDOP);
    //Serial.print(F(" (m * 10^-2)"));    

    return true;
  } 
  return false;
}
bool UpdateSensors()
{
  deltaTime = millis() - lastUpdate;
  lastUpdate = millis();
  if(deltaTime < 10)
  {
    Serial.print("   ");
  }
  else if(deltaTime < 100)
  {
    Serial.print("  ");
  }
  else if(deltaTime < 1000)
  {
    Serial.print(" ");
  }
  Serial.print(deltaTime);
  
  Serial.print(" Digital: ");
  Serial.print(digitalChannels.digitalRead(0) == LOW ? "L" : "H");
  Serial.print("\tD1 ");
  Serial.print(digitalChannels.digitalRead(1) == LOW ? "L" : "H");
  Serial.print("\tD2");
  Serial.print(digitalChannels.digitalRead(2) == LOW ? "L" : "H");
  Serial.print("\tD3");
  Serial.print(digitalChannels.digitalRead(3) == LOW ? "L" : "H");
  return true;
}
