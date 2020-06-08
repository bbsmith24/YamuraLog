#include "I2C_Library.h"
//
bool I2C_Library::I2CInit()
{
    if (wiringPiSetupGpio() == -1)
    {
        return false;
    }
    return true;
}
//
bool I2C_Library::I2CInit(uint8_t address)
{
    if (wiringPiSetupGpio() == -1)
    {
        return false;
    }
    deviceAddress = address;
    return true;
}
//
uint8_t I2C_Library::I2CReadByte(uint8_t address, uint8_t subAddress)
{
    deviceAddress = address;
    return I2CReadByte(subAddress);
}
//
uint8_t I2C_Library::I2CReadByte(uint8_t subAddress)
{
    _fd = wiringPiI2CSetup(deviceAddress);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2CSetup failed\n");
        exit(EXIT_FAILURE);
    }
    uint8_t data; // `data` will store the register data
    if(subAddress == 0)
    {
       data = wiringPiI2CRead(_fd);
    }
    else
    {
        wiringPiI2CWrite(_fd, subAddress);
        data = wiringPiI2CRead(_fd);                // Fill Rx buffer with result
    }
    close(_fd);
    return data;                             // Return data read from slave register
}
//
uint8_t I2C_Library::I2CReadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    deviceAddress = address;
    return I2CReadBytes(subAddress, dest, count);
}
//
uint8_t I2C_Library::I2CReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    _fd = wiringPiI2CSetup(deviceAddress);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2C Setup\n");
        exit(EXIT_FAILURE);
    }
    if(subAddress > 0)
    {
        wiringPiI2CWrite(_fd, subAddress);
    }
    //uint8_t temp_dest[count];
    if ((read(_fd, dest, count)) < 0)
    {
        throw 999;
        return 0;
    }
    close(_fd);
    return count;
}
//
uint8_t I2C_Library::I2CReadBytes(uint8_t address, uint8_t subAddress, I2C_Library::I2CWord* dest)
{
    deviceAddress = address;
    I2CReadBytes(subAddress, dest);
    return 2;
}
//
uint8_t I2C_Library::I2CReadBytes(uint8_t subAddress, I2C_Library::I2CWord* dest)
{
    // open port
    _fd = wiringPiI2CSetup(deviceAddress);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2C Setup in I2CReadBytes\n");
        return 0;// exit(EXIT_FAILURE);
    }
    int errno = 0;
    // write register to read
    errno = wiringPiI2CWrite(_fd, subAddress);
    if(errno < 0)
    {
        fprintf(stderr, "Error (%d): wiringPiI2CWrite in I2CReadBytes\n", errno);
        return 0;//exit(EXIT_FAILURE);
    }
    // read 2 bytes from register
    dest->intVal = wiringPiI2CReadReg16(_fd, subAddress);
    return 2;

}
//
uint8_t I2C_Library::I2CWriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    deviceAddress = address;
    return I2CWriteByte(subAddress, data);
}
//
uint8_t I2C_Library::I2CWriteByte(uint8_t subAddress, uint8_t data)
{
    _fd = wiringPiI2CSetup(deviceAddress);
    if (_fd < 0)
    {
        fprintf(stderr, "Error (%d): I2CSetup failed\n", _fd);
        exit(EXIT_FAILURE);
    }
    int errno = 0;
    if(subAddress == 0)
    {
        errno = wiringPiI2CWrite(_fd, data);
        if (_fd < 0)
        {
            fprintf(stderr, "Error (%d): wiringPiI2CWrite in I2CWriteByte\n", _fd);
            return 0;//exit(EXIT_FAILURE);
        }
    }
    else
    {
        errno = wiringPiI2CWriteReg8(_fd, subAddress, data);
        if (_fd < 0)
        {
            fprintf(stderr, "Error (%d): wiringPiI2CWriteReg8 in I2CWriteByte\n", _fd);
            return 0;//exit(EXIT_FAILURE);
        }
    }
    close(_fd);
    return 1;
}
//
uint8_t I2C_Library::I2CWriteBytes(uint8_t address, uint8_t subAddress, I2C_Library::I2CWord source)
{
    deviceAddress = address;
    I2CWriteBytes(subAddress, source);
    return 2;
}
//
uint8_t I2C_Library::I2CWriteBytes(uint8_t subAddress, I2C_Library::I2CWord source)
{
//	I2CWriteByte(deviceAddress, subAddress, source.byteVals[0]);
//	I2CWriteByte(deviceAddress, subAddress, source.byteVals[1]);
//    return 2;
    int errno = 0;
    _fd = wiringPiI2CSetup(deviceAddress);
    if (_fd < 0)
    {
        fprintf(stderr, "Error (%d): I2CSetup failed in I2CWriteBytes\n", _fd);
        return 0;//exit(EXIT_FAILURE);
    }
    if(subAddress == 0)
    {
        errno = wiringPiI2CWrite(_fd, source.intVal);
        if(errno< 0)
        {
            fprintf(stderr, "Error (%d): wiringPiI2CWrite failed in I2CWriteBytes\n", errno);
            return 0;
        }
    }
    else
    {
        errno = wiringPiI2CWriteReg16(_fd, subAddress, source.intVal);
        if(errno < 0)//(value>>8) | (value & 0xFF));
        {
            fprintf(stderr, "Error (%d): wiringPiI2CWriteReg16 failed in I2CWriteBytes\n", errno);
            return 0;
         }
    }
    close(_fd);
    return 2;
}
