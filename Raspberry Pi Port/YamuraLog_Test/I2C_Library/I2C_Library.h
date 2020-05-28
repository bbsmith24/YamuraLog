#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#ifndef __I2C_Library_H__
#define __I2C_Library_H__

class I2C_Library
{
public:
union I2CWord
{
    uint8_t  byteVals[2];
    uint16_t intVal;
};

protected:
    int _fd;
    uint8_t deviceAddress;

public:    ///////////////////
    // I2C Functions //
    ///////////////////
    // I2CInit() -- Initialize the I2C hardware.
    // This function will setup all I2C pins and related hardware.
    bool I2CInit();
    bool I2CInit(uint8_t address);

    // I2CwriteByte() -- Write a byte out of I2C to a register in the device
    // Input:
    //    - address = The 7-bit I2C address of the slave device.
    //    - subAddress = The register to be written to.
    //    - data = Byte to be written to the register.
    uint8_t I2CWriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t I2CWriteByte(uint8_t subAddress, uint8_t data);

    uint8_t I2CWriteBytes(uint8_t address, uint8_t subAddress, I2C_Library::I2CWord data);
    uint8_t I2CWriteBytes(uint8_t subAddress, I2C_Library::I2CWord data);

    // read 1 byte from i2c device
    uint8_t I2CReadByte(uint8_t address, uint8_t subAddress);
    uint8_t I2CReadByte(uint8_t subAddress);
    // read a 2 byte word from i2c device into 2 bytes/1 int16 value
    uint8_t I2CReadBytes(uint8_t address, uint8_t subAddress, I2C_Library::I2CWord* dest);
    uint8_t I2CReadBytes(uint8_t subAddress, I2C_Library::I2CWord* dest);
    // read count bytes from i2c device, typically an array of 2 byte/1 int16 value(s)
    uint8_t I2CReadBytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count);
    uint8_t I2CReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);
};

#endif // SFE_LSM9DS1_H //
