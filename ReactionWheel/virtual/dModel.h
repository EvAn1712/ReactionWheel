/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: dModel.h
 *
 * Code generated for Simulink model 'dModel'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Thu Sep  9 14:03:02 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_dModel_h_
#define RTW_HEADER_dModel_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef dModel_COMMON_INCLUDES_
# define dModel_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* dModel_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_dModel_T RT_MODEL_dModel_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T UnitDelay_DSTATE;           /* '<S1>/Unit Delay' */
  real32_T UnitDelay_DSTATE_k;         /* '<S4>/Unit Delay' */
  real32_T UnitDelay_DSTATE_l;         /* '<S6>/Unit Delay' */
  real32_T UnitDelay_DSTATE_a;         /* '<S2>/Unit Delay' */
  real32_T current;                    /* '<S3>/Discrete Transfer Fcn' */
  real32_T speed;                      /* '<S3>/Discrete-Time Integrator' */
  real32_T speed_h;                    /* '<S5>/Discrete-Time Integrator' */
  real32_T position;                   /* '<S5>/Discrete-Time Integrator1' */
  uint32_T m_bpIndex;                  /* '<S10>/1-D Lookup Table' */
  int8_T DiscreteTimeIntegrator_PrevResetState;/* '<S3>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevResetState_p;/* '<S5>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_SatPortCache;/* '<S5>/Discrete-Time Integrator1' */
  boolean_T Memory_PreviousInput;      /* '<S11>/Memory' */
  boolean_T Memory_PreviousInput_i;    /* '<S15>/Memory' */
} DW_dModel_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[10];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data[10];
} ConstP_dModel_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T Setpoint;                   /* '<Root>/Setpoint' */
} ExtU_dModel_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T MotorCurrent;               /* '<Root>/MotorCurrent' */
  real32_T MotorSpeed;                 /* '<Root>/MotorSpeed' */
  real32_T PlatformPosition;           /* '<Root>/PlatformPosition' */
  real32_T PlatformSpeed;              /* '<Root>/PlatformSpeed' */
} ExtY_dModel_T;

/* Real-time Model Data Structure */
struct tag_RTM_dModel_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_dModel_T dModel_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_dModel_T dModel_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_dModel_T dModel_Y;

/* Constant parameters (default storage) */
extern const ConstP_dModel_T dModel_ConstP;

/* Model entry point functions */
extern void dModel_initialize(void);
extern void dModel_step(void);
extern void dModel_terminate(void);

/* Real-time Model object */
extern RT_MODEL_dModel_T *const dModel_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S8>/cast' : Eliminate redundant data type conversion
 * Block '<S9>/cast' : Eliminate redundant data type conversion
 * Block '<S20>/cast' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'dModel'
 * '<S1>'   : 'dModel/LTS6-NP current sensor'
 * '<S2>'   : 'dModel/MPU9250 gyrometer'
 * '<S3>'   : 'dModel/motor+wheel'
 * '<S4>'   : 'dModel/optical encoder 100 steps//round'
 * '<S5>'   : 'dModel/platform'
 * '<S6>'   : 'dModel/potentiometer'
 * '<S7>'   : 'dModel/voltage control'
 * '<S8>'   : 'dModel/LTS6-NP current sensor/Idealized ADC quantizer'
 * '<S9>'   : 'dModel/MPU9250 gyrometer/Idealized ADC quantizer'
 * '<S10>'  : 'dModel/motor+wheel/Frictions'
 * '<S11>'  : 'dModel/motor+wheel/Stop Detection'
 * '<S12>'  : 'dModel/motor+wheel/Stop Detection/Test Speed'
 * '<S13>'  : 'dModel/motor+wheel/Stop Detection/Test Torque'
 * '<S14>'  : 'dModel/platform/Coulomb & Viscous Friction '
 * '<S15>'  : 'dModel/platform/Stop Detection'
 * '<S16>'  : 'dModel/platform/Unbalanced Mass'
 * '<S17>'  : 'dModel/platform/Stop Detection/IsNonZero'
 * '<S18>'  : 'dModel/platform/Stop Detection/Test Speed'
 * '<S19>'  : 'dModel/platform/Stop Detection/Test Torque'
 * '<S20>'  : 'dModel/potentiometer/Idealized ADC quantizer'
 */
#endif                                 /* RTW_HEADER_dModel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
