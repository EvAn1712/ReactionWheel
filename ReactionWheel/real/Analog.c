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
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

#include "Analog.h"
#include "ADS1115.h"    // ADS1115 registers/bits 

//-----------------------------------------------------------
// Private variables/functions
//-----------------------------------------------------------

// ADS1115 are connected to I2C1 at pins P9.17 P9.18
//
// I2C1 bus device path is "/dev/i2c-1/"

static int i2c_fd;                  // i2c-1 file descriptor
static double fullscale = 2.048;    // ±2V at power on

#define POSITION_ADDRESS ADS1115_ADDRESS_ADDR_GND
#define CURRENT_ADDRESS  ADS1115_ADDRESS_ADDR_VDD

// Set I2C slave address
// @param address : slave device address
static void I2C_setSlave(const uint8_t address)
{
    ioctl(i2c_fd, I2C_SLAVE, address);
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

// Write to a 16bits register on the I2C device
// @param reg : register offset
// @param data : register value
static void I2C_writeRegister16(const uint8_t reg, uint16_t data)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data>>8;   // high byte
    buf[2] = data;      // low  byte
    write(i2c_fd, buf, 3);
}
//-----------------------------------------------------------
// Public functions
//-----------------------------------------------------------

// Initialize Analog converters
// <li>Scale : ±4 Volts
// <li>Rate ; 128 Hz
// <li>Input : differential
void Analog_initialize(void)
{
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if(i2c_fd < 0) {
        perror("Analog/I2C_open");
        return;
    }
    // configure for AN0-AN1 4.096V 128Hz
    uint16_t config = ADS1115_MUX_P0_N1 | ADS1115_PGA_4P096 | ADS1115_MODE_CONTINUOUS | ADS1115_RATE_128 | ADS1115_COMP_QUE_DISABLE;
    fullscale = 4.096;
    // Configure Potentiometer.
    uint16_t check=0;
    I2C_setSlave(POSITION_ADDRESS);
    I2C_writeRegister16(ADS1115_RA_CONFIG, config);
    I2C_readRegister16(ADS1115_RA_CONFIG,&check);
    if( check != config )   perror("Analog/Position");
    // Configure Current sensor.
    check=0;
    I2C_setSlave(CURRENT_ADDRESS);
    I2C_writeRegister16(ADS1115_RA_CONFIG, config);
    I2C_readRegister16(ADS1115_RA_CONFIG,&check);
    if( check != config )   perror("Analog/Current");
}
// Read conversion result.
// @return Conversion result in Volts
double Analog_read(void)
{
    int16_t reg = 0;
    double voltage;
    I2C_readRegister16(ADS1115_RA_CONVERSION, &reg);
    voltage = ((double) reg) * fullscale / 32768.0;
    return voltage;
}
// Read Position potentiometer voltage.
// @return Conversion result in Volts
double Position_read(void)
{
    I2C_setSlave(POSITION_ADDRESS);
    return Analog_read();
}
// Read Current sensor voltage.
// @return Conversion result in Volts
double Current_read(void)
{
    I2C_setSlave(CURRENT_ADDRESS);
    return Analog_read();
}
// Close Analog Converters.
void Analog_terminate(void)
{
    close(i2c_fd);
}

