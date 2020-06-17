#ifndef QWIIC_SENSORS_H
#define QWIIC_SENSORS_H
// SX1509 16 channel digital IO devices
#include <SparkFunSX1509.h>

class QWIIC_Sensor
{
  public:
    bool log = false;
    bool available = false;
    bool online = false;
    int i2cAddress = 0x00;
    unsigned long minSampleInterval = 1000;
    unsigned long lastSampleMillis = 0;
    
    //QWIIC_Sensor();
    virtual bool IsAvailable() = 0;
    void SetAvailable(bool avail);
    bool IsOnline();
    void SetOnline(bool ol);
    bool IsLogged();
    void SetLogged(bool logSens);
    void SetMinSampleInterval(unsigned long minInterval);
    bool SampleSensor(unsigned long millisNow);
    virtual String WriteToCSV();
    virtual void WriteToBinary();

  private:
    bool ReadSensor();
};
class SX1509_Sensor : QWIIC_Sensor
{
  public:
    // device
    SX1509 digitalSensor_SX1509;
    // log pin state
    bool logPins[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
    // 16 bits from SX1509 representing pin state (read all at once for I2C speed
    uint16_t pinVals;
    //
    SX1509_Sensor(int address);

  private:
    bool ReadSensor();
    String WriteToCSV();
    void WriteToBinary();
};
#endif
