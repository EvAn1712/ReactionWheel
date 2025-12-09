// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************
#ifndef HARDWARE_LIB_H
#define HARDWARE_LIB_H

typedef struct {
    float motorSpeed;       // Unit : radian/s
    float platformSpeed;    // Unit : radian/s
    float platformPosition; // Unit : radian
    float motorCurrent;     // Unit : ampere
} SampleType; /** Sensors Sample structure \n
 This structure contains 4 floats :
 * <li> motor speed (rad/s)
 * <li> platform speed (rad/s)
 * <li> platform position (rad)
 * <li> motor current (A)
 */

void HardwareInitialize(void);
void HardwareTerminate(void);
void HardwareReset(void);
void SensorAcquisition(float sensor[4]);
void SampleAcquisition(SampleType *sample);
void ApplySetpointCurrent(float current);

#endif

