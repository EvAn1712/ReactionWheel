#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>


#include "Motor.h"

//-----------------------------------------------------------
// Private variables/functions
//-----------------------------------------------------------

// Motor is connected to PWM0A at pin P9.22
//
// Base device path is "/sys/class/pwm/pwm-1:0/"
// enable : 0/1
// period/duty cycle : nanoseconds (unsigned integer)
// polarity : "normal" (active High) / "inversed" (active Low)

#define PWM_ENABLE     "/sys/class/pwm/pwm-1:0/enable"
#define PWM_PERIOD     "/sys/class/pwm/pwm-1:0/period"
#define PWM_DUTY       "/sys/class/pwm/pwm-1:0/duty_cycle"
#define PWM_POLARITY   "/sys/class/pwm/pwm-1:0/polarity"

static unsigned long period_ns = 1e9 / 20000.0; // 20KHz
static int duty_fd;     // dutycycle file descriptor

static void textwrite(const char * fname,const char * str)
{
    int fd = open(fname, O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("Motor/open");
        return;
    }
    write(fd,str,strlen(str));
    close(fd);
}
//-----------------------------------------------------------
// Public functions
//-----------------------------------------------------------

// Initialize the Motor PWM device.
// <li> Period 20KHz
// <li> Active High
// <li> Enable output
void Motor_initialize(void)
{
    // A valid period must be set before anything
    char strValue[64]="";
    snprintf(strValue, sizeof(strValue), "%lu", period_ns);
    textwrite(PWM_DUTY,"0");
    textwrite(PWM_PERIOD,strValue);
    textwrite(PWM_ENABLE,"0");
    textwrite(PWM_POLARITY,"normal");
    // set initial duty to 100% (no pulse generated)
    // to avoid falling edge detection on start
    textwrite(PWM_DUTY,strValue);
    // open dutycycle file for multiple write
    duty_fd = open(PWM_DUTY, O_WRONLY | O_NONBLOCK);
    if(duty_fd < 0) perror("Encoder/position/open");
    // Power on
    textwrite(PWM_ENABLE,"1");
}

// Set the Motor duty cycle.
// @param percent : value in % ( 0 to 100 )
// @Note 0% or 100% generate no pulse
void Motor_duty(double percent)
{
    char strValue[64] = "";
    percent = ( percent <   0.0 ?   0.0 : percent);
    percent = ( percent > 100.0 ? 100.0 : percent);
    unsigned long duty = ((double) period_ns) * (percent / 100.0);
    snprintf(strValue, sizeof(strValue), "%lu", duty);
    write(duty_fd,strValue,strlen(strValue));
}
// Power off the Motor.
// <li> Disable output
// <li> Close device
void Motor_terminate(void)
{
    close(duty_fd);
    textwrite(PWM_ENABLE,"0");
    textwrite(PWM_DUTY,"0");
    textwrite(PWM_POLARITY,"normal");
}
