// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#ifdef __cplusplus
extern "C" {
#endif

void   Gyroscope_initialize(void);
double Gyroscope_rotationZ(void);
void   Gyroscope_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
