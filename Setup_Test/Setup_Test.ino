#include <SD.h>

#define CHIPSELECT 8

enum SensorTypes
{
  GPS_QWIIC,
  ACC_MMA8452Q,
  IMU_VR,
  YL_DRV,
};

File dataFile;
// sensors available
// [0] GPS_QWIIC
// [1] ACC_MMA8452Q
// [2] IMU_VR
// [3] YL_DRV
int sensorAvailable[4];
int sensorAddress[4];

void setup() 
{
  // put your setup code here, to run once:
  // Open serial communications and wait for port to open:
  Serial.begin(115200);  
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) { }

  SD.begin(CHIPSELECT);

}

void loop() 
{
  int incomingByte = 0;
  if (Serial.available() > 0) 
  {
    int incomingByte = 0;
    if(Serial.available() > 0) 
    {
      incomingByte = Serial.read();
    }
    // create default
    if((char)incomingByte == 'C')
    {
      CreateINIFile();
    }
    // read existing
    else if ((char)incomingByte == 'R')
    {
      ReadINIFile();
    }
  }
}
//
// read ini file
// legal devices
// GPS_QWIIC    - address 0x10                                  - Sparkfun QWIIC GPS returns Latitude, Longitude, Speed-GPS, Heading-GPS; calculates Distance-GPS when uploaded
// ACC_MMA8452Q - address 0x1D default, 0x1C with jumper closed - Sparkfun QWIIC Accelerometer returns gX, gY, gZ channels (acceleration in 2G range)
// IMU_VR       - address 0x4B default, 0x4A jumper closed      - Sparkfun QWIIC BN0080 9 dof IMU 
// YL_DRV       - address 0x00 to 0x15 by jumpers on board      - YamuraLog driver box returns A2D0 (analog to digital), A2D1 (analog to digital), A2D2 (digital), A2D3 (digital)
//
// Notes: ACC_MMA8452Q, IMU_VR are mutually exclusive
//
// file format
// device<tab>address in 0x00 format
bool ReadINIFile()
{
  // create a default ini file
  if(!SD.exists("Yamura.ini"))
  {
    Serial.println("No INI file present");
  }
  // read ini file
  else
  {
    Serial.println("read data file");
    // open the file for reading:
    dataFile = SD.open("Yamura.ini", FILE_READ);
    // read from the file until there's nothing else in it:
    while (dataFile.available()) 
    {
      Serial.write(dataFile.read());
    }
    // close the file:
    dataFile.close();
    return true;
  }
}
bool CreateINIFile()
{
  // create a default ini file
  Serial.println("create default data file");
  SD.remove("Yamura.ini");
  dataFile = SD.open("Yamura.ini", FILE_WRITE);
  dataFile.println("GPS_QWIIC\t0x10");
  dataFile.println("ACC_MMA8452Q\t0x1D");
  dataFile.println("YL_DRV\t0x08");
  dataFile.close();
}
