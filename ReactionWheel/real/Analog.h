// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#ifndef ANALOG_H
#define ANALOG_H

#ifdef __cplusplus
extern "C" {
#endif

void   Analog_initialize(void);
double Position_read(void);
double Current_read(void);
void   Analog_terminate(void);

#ifdef __cplusplus
}
#endif

#endif

