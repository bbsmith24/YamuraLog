#include "I2C_Library.h"

bool I2C_Library::I2CInit()
{
    if (wiringPiSetupGpio() == -1)
    {
        return false;
    }
    return true;
}

// Wire.h read and write protocols
void I2C_Library::I2CWriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2CSetup failed\n");
        exit(EXIT_FAILURE);
    }
    wiringPiI2CWriteReg8(_fd, subAddress, data);
    close(_fd);
}

uint8_t I2C_Library::I2CReadByte(uint8_t address, uint8_t subAddress)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2CSetup failed\n");
        exit(EXIT_FAILURE);
    }
    uint8_t data; // `data` will store the register data
    wiringPiI2CWrite(_fd, subAddress);
    data = wiringPiI2CRead(_fd);                // Fill Rx buffer with result
    close(_fd);
    return data;                             // Return data read from slave register
}

uint8_t I2C_Library::I2CReadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0)
    {
        fprintf(stderr, "Error: I2C Setup\n");
        exit(EXIT_FAILURE);
    }
    wiringPiI2CWrite(_fd, subAddress);
    uint8_t temp_dest[count];
    if ((read(_fd, temp_dest, 6)) < 0)
    {
        //fprintf(stderr, "Error: read value\n");
        throw 999;
        return 0;
    }
    close(_fd);
    for (int i = 0; i < count; i++)
    {
        dest[i] = temp_dest[i];
    }
    return count;
}
