// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

#include "Gyroscope.h"
#include "MPU9250.h"    // MPU9250 registers/bits 

//-----------------------------------------------------------
// Private variables/functions
//-----------------------------------------------------------

// MPU9250 is connected to I2C2 at pins P9.19 P9.20
//
// I2C2 bus device path is "/dev/i2c-2/"

// address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS     MPU9250_ADDRESS_AD0_LOW 

static int i2c_fd;     // i2c-2 file descriptor
static double fullscale = 250.0 * M_PI / 180.0;	// 250°/s at power on

// Set I2C slave address
// @param address : slave device address
static void I2C_setSlave(const uint8_t address)
{
    ioctl(i2c_fd, I2C_SLAVE, address);
}
// Read a 8bits register on the I2C device
// @param reg : register offset
// @param data : pointer to 8 bits integer
static void I2C_readRegister8(const uint8_t reg, uint8_t *data)
{
    // Send register address we want to read
    write(i2c_fd, (void *) &reg, 1);
    // Read register value
    read(i2c_fd, data, 1);
}
// Read a 16bits register on the I2C device
// @param reg : register offset
// @param data : pointer to 16 bits integer
static void I2C_readRegister16(const uint8_t reg, uint16_t *data)
{
    // Send register address we want to read
    write(i2c_fd, (void *) &reg, 1);
    // Read register value 2 bytes
    read(i2c_fd, data, 2);
    // convert from BigEndian
    *data = be16toh(*data);
}
// Write to a 8bits register on the I2C device
// @param reg : register offset
// @param data : value
static void I2C_writeRegister8(const uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    write(i2c_fd, buf, 2);
}
//-----------------------------------------------------------
// Public functions
//-----------------------------------------------------------

// Initialize Gyroscope
// <li>Scale : ±250°/s
// <li>Bandwidth : 92Hz
void Gyroscope_initialize(void)
{
    i2c_fd = open("/dev/i2c-2", O_RDWR);
    if(i2c_fd < 0) {
        perror("Gyroscope/I2C_open");
        return;
    }
    // Test MPU9250 presence.
    I2C_setSlave(MPU9250_ADDRESS);
    uint8_t reg = 0;
    I2C_readRegister8(MPU9250_RA_WHO_AM_I, &reg);
    if(reg != MPU9250_ID) {
        perror("Gyroscope/Not present");
        return;
    }
    // Configure LowPassFilter = 92Hz
    I2C_writeRegister8(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_92);
}
// Read Gyro Z axis.
// @return result in Radian/s
double Gyroscope_rotationZ(void)
{
    int16_t reg = 0;
    double rads;
    I2C_setSlave(MPU9250_ADDRESS);
    I2C_readRegister16(MPU9250_RA_GYRO_ZOUT_H, &reg);
    rads = ((double) reg) * fullscale / 32768.0;
    return rads;
}
// Close Gyro
void Gyroscope_terminate(void)
{
    close(i2c_fd);
}
