// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************
#ifndef DISCRETE_MODEL_H
#define DISCRETE_MODEL_H

#include "ReactionWheel.h"
//--< Simulink Discrete Model includes >-----------------------
#include "dModel.h"
//-------------------------------------------------------------
extern RT_MUTEX discreteModel_Mutex;
extern void discreteModel_Initialize();
extern void discreteModel_Reset();
extern void discreteModel_Terminate();

#endif /* DISCRETE_MODEL_H */

