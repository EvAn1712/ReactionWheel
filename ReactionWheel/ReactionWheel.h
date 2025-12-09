// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// ***********************************************************
#ifndef REACTION_WHEEL_H
#define REACTION_WHEEL_H

/* All the definitions needed for the project */

//--< Linux headers >------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
//--< Xenomai headers >----------------------------------------------
#include <xeno_config.h>
#if CONFIG_XENO_VERSION_MAJOR == 3
//--< Xenomai 3.x headers >----------------------------------------------
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/queue.h>
#include <alchemy/sem.h>
#include <alchemy/event.h>
#include <alchemy/mutex.h>
#define DEFAULTSTACKSIZE      0     // Set the standard stack size for tasks.
#else
#error "Invalid Xenomai Version"
#endif
//--< Xenomai declarations >-----------------------------------------

//--< Application headers >------------------------------------------
#include "HardwareLib.h"
#include "SocketLib.h"
//--< Application declarations >-------------------------------------

typedef struct {
    int   law;        // N law (0=open loop) 
    int   option;     // experiment options 
    int   duration;
    int   acquisitionPeriod;   // ms
    int   lawPeriod;           // ms
    int   setpointProfile;
    float endSetpointValue;
    int   riseDurationSetup;   // ms 
    float maxSpeed;            //rad/s
    float accel;               //rad/s²
    float lawCoeff[10];
} ExperimentParametersType; // Structure containing experiment parameters

//--< WheelTasks.c >-------------------------------------------------
void  ReturnSensorsMeasurement(void);
void  AbortExperiment(void);
void  StartExperiment(void);
//--< WheelLaws.c >-------------------------------------------------
void  InitializeExperiment(SampleType sample);
void  InitializeLaw(void);
float ComputeLaw(SampleType measure);
//--< SetpointLib.c >-------------------------------------------------
void  InitializeSetpoint(float PlatformPosition);
float ComputeNewSetpoint(void);
float ReturnCurrentSetpoint(void);
//--< WheelHMI.c >-------------------------------------------------
void  ManageRequest(void);
bool  ConnectionIsActive(void);
//--< HardwareLib.c >-------------------------------------------------
void  HardwareInitialize(void);
void  HardwareTerminate(void);
void  HardwareReset(void);
void  SampleAcquisition(SampleType *sample);
void  ApplySetpointCurrent(float current);

#endif

