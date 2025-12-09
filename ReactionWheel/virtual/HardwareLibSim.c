// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************

#include "ReactionWheel.h"

#include "discreteModel.h"

/** Acquire all sensors.
 * @param sensor : array of float that receive sensors values
 * <li> sensor[0] = motor speed (rad/s)
 * <li> sensor[1] = platform speed (rad/s)
 * <li> sensor[2] = platform position (rad)
 * <li> sensor[3] = motor current (A)
 */
void SensorAcquisition(float sensor[4])
{
    // read sensors values
    rt_mutex_acquire(&discreteModel_Mutex, TM_INFINITE);
    sensor[0] = dModel_Y.MotorSpeed;
    sensor[1] = dModel_Y.PlatformSpeed;
    sensor[2] = dModel_Y.PlatformPosition;
    sensor[3] = dModel_Y.MotorCurrent;
    rt_mutex_release(&discreteModel_Mutex);
}

/** Acquire all sensors.
 * @param sample : pointer to <b>SampleType</b> struct that receive sensors values 
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

/** Open and initialize simulated hardware.\n
 * 
 * Set motor command to zero.\n
 * Set platform position to zero.\n
 */
void HardwareInitialize(void)
{
    /* Disable buffering on stdout to have immediate display with ssh */
    setbuf(stdout, NULL);
    /* Start model task with highest priority */
    discreteModel_Initialize();
}

/** Stop task running simulated hardware.
 */
void HardwareTerminate(void)
{
    discreteModel_Terminate();
}

/** Reset task running simulated hardware.
 */
void HardwareReset(void)
{
    discreteModel_Reset();
}

/** Apply motor command with saturation at +/-0.6A
 * @param current : desired current value in Ampere
 */
void ApplySetpointCurrent(float current)
{
    rt_mutex_acquire(&discreteModel_Mutex, TM_INFINITE);
    dModel_U.Setpoint = current;
    rt_mutex_release(&discreteModel_Mutex);
}
