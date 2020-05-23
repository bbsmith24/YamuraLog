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
protected:
    int _fd;

public:    ///////////////////
    // I2C Functions //
    ///////////////////
    // I2CInit() -- Initialize the I2C hardware.
    // This function will setup all I2C pins and related hardware.
    bool I2CInit();

    // I2CwriteByte() -- Write a byte out of I2C to a register in the device
    // Input:
    //    - address = The 7-bit I2C address of the slave device.
    //    - subAddress = The register to be written to.
    //    - data = Byte to be written to the register.
    void I2CWriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

    // I2CreadByte() -- Read a single byte from a register over I2C.
    // Input:
    //    - address = The 7-bit I2C address of the slave device.
    //    - subAddress = The register to be read from.
    // Output:
    //    - The byte read from the requested address.
    uint8_t I2CReadByte(uint8_t address, uint8_t subAddress);

    // I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
    // Input:
    //    - address = The 7-bit I2C address of the slave device.
    //    - subAddress = The register to begin reading.
    //     - * dest = Pointer to an array where we'll store the readings.
    //    - count = Number of registers to be read.
    // Output: No value is returned by the function, but the registers read are
    //         all stored in the *dest array given.
    uint8_t I2CReadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
};

#endif // SFE_LSM9DS1_H //
