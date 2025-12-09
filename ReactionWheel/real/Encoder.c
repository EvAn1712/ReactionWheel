// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "Encoder.h"

//-----------------------------------------------------------
// Private variables/functions
//-----------------------------------------------------------

// Encoder is connected to EQEP0 at pins P9.42 P9.27 P9.41 P9.25
//
// Base device path is "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/"
// enabled : 0/1
// period : nanoseconds (unsigned integer)
// position : edges count (integer)
// mode : 0/1 (absolute/relative = position/speed)
#define EQEP_ENABLE     "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/enabled"
#define EQEP_PERIOD     "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/period"
#define EQEP_POSITION   "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/position"
#define EQEP_MODE       "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/mode"

static double period = 1.0 / 25.0; // 25Hz
static int position_fd; // position file descriptor

static void textwrite(const char * fname,const char * str)
{
    int fd = open(fname, O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("Encoder/open");
        return;
    }
    write(fd,str,strlen(str));
    close(fd);
}
//-----------------------------------------------------------
// Public functions
//-----------------------------------------------------------

// Initialize the Motor Encoder device.
// <li> Period 25Hz
// <li> Speed measurement mode
// <li> Counting On
void Encoder_initialize(void)
{
    char strValue[64]="";
    snprintf(strValue, sizeof(strValue), "%lu", (long)(1e9 * period));
    textwrite(EQEP_ENABLE,"0");
    textwrite(EQEP_MODE,"1");
    textwrite(EQEP_POSITION,"0");
    textwrite(EQEP_PERIOD,strValue);
    // open position file for multiple read
    position_fd = open(EQEP_POSITION, O_RDONLY | O_NONBLOCK);
    if(position_fd < 0) perror("Encoder/position/open");
    // start counting
    textwrite(EQEP_ENABLE,"1");
}
// Stop the Motor Encoder device.
// <li> Counting Off
// <li> Reset position/speed
// <li> Close device
void Encoder_terminate(void)
{
    close(position_fd);
    textwrite(EQEP_ENABLE,"0");
    textwrite(EQEP_POSITION,"0");
}
// Read the Motor Encoder position.
// @return absolute position : in pulse (x4)
long Encoder_position(void)
{
    char strValue[64] = "";
    long position = 0;
    if(lseek(position_fd, 0L, SEEK_SET) != 0) {
        perror("Encoder/position/lseek");
        return position;
    }
    if(read(position_fd, strValue, sizeof(strValue) - 1) <= 0) {
        perror("Encoder/position/read");
        return position;
    }
    if(sscanf(strValue, "%ld", &position) != 1) {
        perror("Encoder/position/scanf");
    }
    return position;
}
// Read the Motor Encoder speed (in relative mode).
// @return speed : in pulse / second
double Encoder_speed(void)
{
    long pos = Encoder_position() / 4;
    return ((double) pos) / period;
}
