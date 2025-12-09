// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

void   Encoder_initialize(void);
long   Encoder_position(void);
double Encoder_speed(void);
void   Encoder_terminate(void);

#ifdef __cplusplus
}
#endif

#endif

