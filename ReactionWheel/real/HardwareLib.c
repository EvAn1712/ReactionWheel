// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#include "ReactionWheel.h"
#include "HardwareLib.h"

#include "Motor.h"
#include "Encoder.h"
#include "Gyroscope.h"
#include "Analog.h"

/* local declarations */

struct {
    float motorSpeedOffset;       // default : 0.0
    float motorSpeedGain;         // default : -0.62832 (2*pi/100)
    float platformSpeedOffset;    // default : 0.0
    float platformSpeedGain;      // default : -1.0
    float platformPositionOffset; // default : 0.0
    float platformPositionGain;   // default : 1.256 (pi/2.5)
    float motorCurrentOffset;     // default : 0.0
    float motorCurrentGain;       // default : 1.6 (1/0.625)
    float motorCommandOffset;     // default : 0.0
    float motorCommandGain;       // default : 1.0
} calibration;

/** Fill calibration parameters with new values.
 * @param array_of_float : array with 10 values.
 */
void updateCalibration(float array_of_float[])
{
    calibration.motorSpeedOffset       = array_of_float[0];
    calibration.motorSpeedGain         = array_of_float[1];
    calibration.platformSpeedOffset    = array_of_float[2];
    calibration.platformSpeedGain      = array_of_float[3];
    calibration.platformPositionOffset = array_of_float[4];
    calibration.platformPositionGain   = array_of_float[5];
    calibration.motorCurrentOffset     = array_of_float[6];
    calibration.motorCurrentGain       = array_of_float[7];
    calibration.motorCommandOffset     = array_of_float[8];
    calibration.motorCommandGain       = array_of_float[9];
}

/** Read calibration parameters from a text file.
 * @param filename : fullpath  to text file.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
int readCalibration(const char * filename)
{
    int i;
    char line[128];
    float value[10];
    FILE * fs = fopen(filename, "r");
    if (fs == NULL) return (EXIT_FAILURE);
    for (i = 0; i < 10; i++) {
        if ( fgets(line, 127, fs) == NULL ) break;
        if ( sscanf(line, "%f", value + i) != 1 ) break;
    }
    fclose(fs);
    if ( i == 10 ) {
        updateCalibration(value);
        return (EXIT_SUCCESS);
    } else
        return (EXIT_FAILURE);
}

/** Apply calibration */
float pulseToMotorSpeed(float pulses)
{
    return (pulses - calibration.motorSpeedOffset) * calibration.motorSpeedGain;
}

/** Apply calibration */
float radsToPlatformSpeed(float rads)
{
    return (rads - calibration.platformSpeedOffset) * calibration.platformSpeedGain;
}

/** Apply calibration */
float voltToPlatformPosition(float volts)
{
    return (volts - calibration.platformPositionOffset) * calibration.platformPositionGain;
}

/** Apply calibration */
float voltToMotorCurrent(float volts)
{
    return (volts - calibration.motorCurrentOffset) * calibration.motorCurrentGain;
}

/** Acquire all sensors with calibration.
 * @param sensor : array of float that receive calibrated sensors values
 * <li> sensor[0] = motor speed (rad/s)
 * <li> sensor[1] = platform speed (rad/s)
 * <li> sensor[2] = platform position (rad)
 * <li> sensor[3] = motor current (A)
 */
void SensorAcquisition(float sensor[4])
{
    sensor[0] = pulseToMotorSpeed     ( Encoder_speed()       );
    sensor[1] = radsToPlatformSpeed   ( Gyroscope_rotationZ() );
    sensor[2] = voltToPlatformPosition( Position_read()       );
    sensor[3] = voltToMotorCurrent    ( Current_read()        );
}

/** Acquire all sensors with calibration.
 * @param sample : pointer to struct that receive calibrated sensors values 
 * <li> motor speed (rad/s)
 * <li> platform speed (rad/s)
 * <li> platform position (rad)
 * <li> motor current (A)
 */
void SampleAcquisition(SampleType *sample)
{
    float sensor[4];
    SensorAcquisition(sensor);
    sample->motorSpeed       = sensor[0];
    sample->platformSpeed    = sensor[1];
    sample->platformPosition = sensor[2];
    sample->motorCurrent     = sensor[3];
}

/** Open and initialize all devices.\n
 * 
 * Exit application if devices are already in use.
 * 
 * Set motor command to zero.\n\n
 * Initialize calibration parameters values
 * with local file "/usr/local/etc/calibration.txt".\n
 * Lock Hardware for other processes.
 */
void HardwareInitialize(void)
{
    /* Disable buffering on stdout to have immediate display with ssh */
    setbuf(stdout, NULL);
    /* Try to create a Lock in shared mem
     * Exit application with error if Lock already exist.
     */
    int lockfd = shm_open("/wheelLock", O_EXCL | O_RDWR | O_CREAT, 0666);	// try to create Lock
    if (lockfd < 0) {
        perror(" Hardware is used by another process");
        exit(EXIT_FAILURE);
    }
    close(lockfd);
    /* Initialize Actuators/Sensors */
    Motor_initialize();
    Encoder_initialize();
    Gyroscope_initialize();
    Analog_initialize();
    /* Initialize calibration parameters */
    if ( readCalibration("/usr/local/etc/calibration.txt") != EXIT_SUCCESS ) {
        rt_printf("Failed to read /usr/local/etc/calibration.txt\r\n");
        rt_printf("Using factory calibration parameters.\r\n");

        /* Motor integrated encoder have 100 steps/round */
        calibration.motorSpeedOffset = 0.0;
        calibration.motorSpeedGain   = -2.0 * M_PI / 100.0;

        /* MPU9250 Gyro Fullscale is +/-250°/s */
        calibration.platformSpeedOffset = 0.0;
        calibration.platformSpeedGain   = -1.0;

        /* Potentiometer on ADS1115 +/-2.5V => +/-pi */
        calibration.platformPositionOffset = 0.0;
        calibration.platformPositionGain   = M_PI / 2.5;

        /* Current sensor (LTS6-NP) on ADS1115 +/-2.5V 625mV/A */
        calibration.motorCurrentOffset = 0.0;
        calibration.motorCurrentGain   = 1.0 / 0.625;

        /* Motor PWM duty cycle 0%=-0.6A 100%=+0.6A */
        calibration.motorCommandOffset = 0.0;
        calibration.motorCommandGain   = 1.0;
    }
}

/** Stop motor command.\n
 * Close all devices.\n
 * Release Hardware for other processes.
 */
void HardwareTerminate(void)
{
    Analog_terminate();
    Gyroscope_terminate();
    Encoder_terminate();
    Motor_terminate();
    shm_unlink("/wheelLock");	// Release Lock
}

/** Set motor command to Zero.
 */
void HardwareReset(void)
{
    ApplySetpointCurrent(0);
}

/** Apply motor command with saturation at +/-0.6A
 * @param current : desired current value in Ampere
 */
void ApplySetpointCurrent(float current)
{
    /* Apply calibration */
    current = (current - calibration.motorCommandOffset) * calibration.motorCommandGain;
    /* process PWM duty cycle from current
     * ] -0.6A  0.0A +0.6A [
     * ]   0%   50%   100% [
     * 0% or 100% --> motor off
     */
    float duty = 100.0 * (current + 0.6) / 1.2;
    /* saturation */
    duty = (duty <  0.1 ?  0.1 : duty);
    duty = (duty > 99.9 ? 99.9 : duty);

    Motor_duty(duty);
}
