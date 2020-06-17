#ifndef QWIIC_SENSORS_H
#define QWIIC_SENSORS_H
// SX1509 16 channel digital IO devices
#include <SparkFunSX1509.h>                     // QWIIC 8 channel digital IO
#include <SparkFun_ADS1015_Arduino_Library.h> 	// QWIIC 4 channel analog to digital converter. Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <SparkFun_I2C_GPS_Arduino_Library.h>   // QWIIC GPS device
#include "SparkFun_BNO080_Arduino_Library.h"    // QWIIC 9 dof IMU

class QWIIC_Sensor
{
  public:
    bool log = false;
    bool available = false;
    bool online = false;
    int i2cAddress = 0x00;
    unsigned long minSampleInterval = 1000;
    unsigned long lastSampleMillis = 0;
    String sensorName = "QWIIC Base";    
    
    bool IsAvailable();
    void SetAvailable(bool avail);
    bool IsOnline();
    void SetOnline(bool ol);
    bool IsLogged();
    void SetLogged(bool logSens);
    void SetMinSampleInterval(unsigned long minInterval);
    bool SampleSensor(unsigned long millisNow);

    // overridable
    virtual void WriteToCSV();
    virtual void WriteToBinary();
    virtual bool Initialize(int addr);

  private:
    virtual bool ReadSensor();
};
class SX1509_Sensor : public QWIIC_Sensor
{
  public:
    // device
    SX1509 digitalSensor_SX1509;
    unsigned long minSampleInterval = 1;
    // log pin state
    bool logPins[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
    // 16 bits from SX1509 representing pin state (read all at once for I2C speed
    uint16_t pinVals;
    //
    SX1509_Sensor();
    bool Initialize(int address);
    void LogPin(int pinNum, bool logPin);

  //private:
    bool ReadSensor();
    void WriteToCSV();
    void WriteToBinary();
};
class ADS1015_Sensor : public QWIIC_Sensor
{
	  public:
    // device
    ADS1015 analogSensor_ADS1015;
    unsigned long minSampleInterval = 1;
    // log pin state
    bool logPins[4] = {false, false, false, false };
    // 16 bits from SX1509 representing a2d value for pin
    uint16_t pinVals[4] = {0, 0, 0, 0 };
    //
    ADS1015_Sensor();
    bool Initialize(int address);
    void LogPin(int pinNum, bool logPin);

  //private:
    bool ReadSensor();
    void WriteToCSV();
    void WriteToBinary();
};
class BNO080_Sensor : public QWIIC_Sensor
{
  public:
    unsigned long minSampleInterval = 1;
	bool logRotation = false;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
    float quatRadianAccuracy;
	bool logGyro = false;
	float gyroX;
	float gyroY;
	float gyroZ;
	bool logAccel = false;
    float accelX;
    float accelY;
    float accelZ;
	bool logLinearAccel = false;
    float linAccelX;
    float linAccelY;
    float linAccelZ;
    byte linAccelAccuracy;	
	bool logMagnetometer = false;
    float magX;
    float magY;
    float magZ;
    byte magAccuracy;	

   // device
    BNO080 imuSensor_BNO080;
    //
    BNO080_Sensor();
    bool Initialize(int address);
    void LogRotation(bool log);
    void LogAcceleration(bool log);
    void LogLinearAcceleration(bool log);
    void LogGyro(bool log);
	void LogMagnetometer(bool log);
    //private:
    bool ReadSensor();
    void WriteToCSV();
    void WriteToBinary();	
};
#endif
