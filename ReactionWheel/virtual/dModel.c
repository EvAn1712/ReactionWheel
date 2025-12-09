/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: dModel.c
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

#include "dModel.h"

/* Constant parameters (default storage) */
const ConstP_dModel_T dModel_ConstP = {
  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  { 0.00024617F, 0.00024617F, 0.00024617F, 0.00025812F, 0.00035133F, 0.0005258F,
    0.000717F, 0.0008604F, 0.0010516F, 0.001195F },

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  { 0.5F, 36.0F, 58.0F, 124.0F, 184.0F, 241.0F, 303.0F, 357.0F, 431.0F, 482.0F }
};

/* Block states (default storage) */
DW_dModel_T dModel_DW;

/* External inputs (root inport signals with default storage) */
ExtU_dModel_T dModel_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_dModel_T dModel_Y;

/* Real-time model */
RT_MODEL_dModel_T dModel_M_;
RT_MODEL_dModel_T *const dModel_M = &dModel_M_;
static real32_T look1_iflf_pbinlxpw(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T prevIndex[], uint32_T maxIndex);
static real32_T look1_iflf_pbinlxpw(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T prevIndex[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  uint32_T found;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'on'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    bpIdx = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search using Previous Index */
    bpIdx = prevIndex[0U];
    iLeft = 0U;
    iRght = maxIndex;
    found = 0U;
    while (found == 0U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx - 1U;
        bpIdx = (iRght + iLeft) >> 1U;
      } else if (u0 < bp0[bpIdx + 1U]) {
        found = 1U;
      } else {
        iLeft = bpIdx + 1U;
        bpIdx = (iRght + iLeft) >> 1U;
      }
    }

    frac = (u0 - bp0[bpIdx]) / (bp0[bpIdx + 1U] - bp0[bpIdx]);
  } else {
    bpIdx = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  prevIndex[0U] = bpIdx;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[bpIdx + 1U] - table[bpIdx]) * frac + table[bpIdx];
}

/* Model step function */
void dModel_step(void)
{
  real32_T rtb_torque;
  boolean_T rtb_Memory;
  real32_T rtb_current;
  real32_T rtb_acceleration;
  boolean_T rtb_AND;
  boolean_T rtb_Memory_o;
  real32_T rtb_speed_d;
  real32_T u0;
  real32_T tmp;

  /* Saturate: '<S8>/Saturate volts  to output range' incorporates:
   *  UnitDelay: '<S1>/Unit Delay'
   */
  if (dModel_DW.UnitDelay_DSTATE > 3.99981689F) {
    tmp = 3.99981689F;
  } else if (dModel_DW.UnitDelay_DSTATE < -4.0F) {
    tmp = -4.0F;
  } else {
    tmp = dModel_DW.UnitDelay_DSTATE;
  }

  /* End of Saturate: '<S8>/Saturate volts  to output range' */

  /* Outport: '<Root>/MotorCurrent' incorporates:
   *  Bias: '<S8>/Bias'
   *  Bias: '<S8>/SetRound'
   *  Bias: '<S8>/ZeroBias'
   *  Gain: '<S1>/fullscale'
   *  Gain: '<S1>/resolution'
   *  Gain: '<S8>/Gain'
   *  Rounding: '<S8>/floor'
   */
  dModel_Y.MotorCurrent = (floorf((tmp + 4.0F) * 8192.0F + 0.5F) + -32768.0F) *
    8.0F * 1.52587891E-5F;

  /* Outport: '<Root>/MotorSpeed' incorporates:
   *  Gain: '<S4>/rad//interval'
   *  Gain: '<S4>/rad//s'
   *  Gain: '<S4>/sens'
   *  Gain: '<S4>/steps//interval'
   *  Gain: '<S4>/steps//s'
   *  Rounding: '<S4>/round'
   *  UnitDelay: '<S4>/Unit Delay'
   */
  dModel_Y.MotorSpeed = floorf(15.915494F * -dModel_DW.UnitDelay_DSTATE_k *
    0.04F) * 0.0628318563F * 25.0F;

  /* Saturate: '<S20>/Saturate volts  to output range' incorporates:
   *  UnitDelay: '<S6>/Unit Delay'
   */
  if (dModel_DW.UnitDelay_DSTATE_l > 3.14144874F) {
    tmp = 3.14144874F;
  } else if (dModel_DW.UnitDelay_DSTATE_l < -3.14159274F) {
    tmp = -3.14159274F;
  } else {
    tmp = dModel_DW.UnitDelay_DSTATE_l;
  }

  /* End of Saturate: '<S20>/Saturate volts  to output range' */

  /* Outport: '<Root>/PlatformPosition' incorporates:
   *  Bias: '<S20>/Bias'
   *  Bias: '<S20>/SetRound'
   *  Bias: '<S20>/ZeroBias'
   *  Gain: '<S20>/Gain'
   *  Gain: '<S6>/fullscale'
   *  Gain: '<S6>/resolution'
   *  Rounding: '<S20>/floor'
   */
  dModel_Y.PlatformPosition = (floorf((tmp + 3.14159274F) * 10430.3779F + 0.5F)
    + -32768.0F) * 6.28318548F * 1.52587891E-5F;

  /* Saturate: '<S9>/Saturate volts  to output range' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (dModel_DW.UnitDelay_DSTATE_a > 4.36312342F) {
    tmp = 4.36312342F;
  } else if (dModel_DW.UnitDelay_DSTATE_a < -4.36332321F) {
    tmp = -4.36332321F;
  } else {
    tmp = dModel_DW.UnitDelay_DSTATE_a;
  }

  /* End of Saturate: '<S9>/Saturate volts  to output range' */

  /* Outport: '<Root>/PlatformSpeed' incorporates:
   *  Bias: '<S9>/Bias'
   *  Bias: '<S9>/SetRound'
   *  Bias: '<S9>/ZeroBias'
   *  Gain: '<S2>/fullscale'
   *  Gain: '<S2>/resolution'
   *  Gain: '<S9>/Gain'
   *  Rounding: '<S9>/floor'
   */
  dModel_Y.PlatformSpeed = (floorf((tmp + 4.36332321F) * 7509.87256F + 0.5F) +
    -32768.0F) * 8.72664642F * 1.52587891E-5F;

  /* DiscreteTransferFcn: '<S3>/Discrete Transfer Fcn' */
  rtb_current = 0.00605869386F * dModel_DW.current;

  /* Memory: '<S11>/Memory' */
  rtb_Memory = dModel_DW.Memory_PreviousInput;

  /* DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
   *  Memory: '<S11>/Memory'
   */
  if (dModel_DW.Memory_PreviousInput ||
      (dModel_DW.DiscreteTimeIntegrator_PrevResetState != 0)) {
    dModel_DW.speed = 0.0F;
  }

  /* Lookup_n-D: '<S10>/1-D Lookup Table' incorporates:
   *  Abs: '<S10>/Abs'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   */
  rtb_torque = look1_iflf_pbinlxpw(fabsf(dModel_DW.speed),
    dModel_ConstP.uDLookupTable_bp01Data, dModel_ConstP.uDLookupTable_tableData,
    &dModel_DW.m_bpIndex, 9U);

  /* Signum: '<S10>/Sign' incorporates:
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   */
  if (dModel_DW.speed < 0.0F) {
    tmp = -1.0F;
  } else if (dModel_DW.speed > 0.0F) {
    tmp = 1.0F;
  } else {
    tmp = dModel_DW.speed;
  }

  /* End of Signum: '<S10>/Sign' */

  /* Sum: '<S3>/Sum' incorporates:
   *  Gain: '<S3>/Gain'
   *  Product: '<S10>/Product'
   */
  rtb_torque = 0.0239F * rtb_current - rtb_torque * tmp;

  /* Gain: '<S3>/Gain2' */
  rtb_acceleration = 8025.68213F * rtb_torque;

  /* Logic: '<S13>/AND' incorporates:
   *  Constant: '<S13>/Lower Limit'
   *  Constant: '<S13>/Upper Limit'
   *  RelationalOperator: '<S13>/Lower Test'
   *  RelationalOperator: '<S13>/Upper Test'
   */
  rtb_AND = ((-0.000239F <= rtb_torque) && (rtb_torque <= 0.000239F));

  /* Gain: '<S7>/Gain3' incorporates:
   *  Inport: '<Root>/Setpoint'
   *  Sum: '<S7>/Sum2'
   */
  u0 = (dModel_U.Setpoint - rtb_current) * 300.0F;

  /* Memory: '<S15>/Memory' */
  rtb_Memory_o = dModel_DW.Memory_PreviousInput_i;

  /* DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
   *  Memory: '<S15>/Memory'
   */
  if (dModel_DW.Memory_PreviousInput_i ||
      (dModel_DW.DiscreteTimeIntegrator_PrevResetState_p != 0)) {
    dModel_DW.speed_h = 0.0F;
  }

  rtb_speed_d = dModel_DW.speed_h;

  /* Signum: '<S14>/Sign' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  if (dModel_DW.speed_h < 0.0F) {
    tmp = -1.0F;
  } else if (dModel_DW.speed_h > 0.0F) {
    tmp = 1.0F;
  } else {
    tmp = dModel_DW.speed_h;
  }

  /* End of Signum: '<S14>/Sign' */

  /* Sum: '<S5>/Sum' incorporates:
   *  Bias: '<S16>/Bias'
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'
   *  Gain: '<S14>/Gain'
   *  Gain: '<S14>/Gain1'
   *  Gain: '<S16>/Gain1'
   *  Sum: '<S14>/Sum'
   */
  rtb_torque = (rtb_torque - (0.000717F * tmp + 0.00239F * dModel_DW.speed_h)) -
    (dModel_DW.position + -3.0F) * 0.00022705F;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  dModel_DW.UnitDelay_DSTATE = rtb_current;

  /* Update for UnitDelay: '<S4>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   */
  dModel_DW.UnitDelay_DSTATE_k = dModel_DW.speed;

  /* Update for UnitDelay: '<S6>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'
   */
  dModel_DW.UnitDelay_DSTATE_l = dModel_DW.position;

  /* Update for UnitDelay: '<S2>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  dModel_DW.UnitDelay_DSTATE_a = dModel_DW.speed_h;

  /* Saturate: '<S7>/Saturation1' */
  if (u0 > 9.0F) {
    u0 = 9.0F;
  } else {
    if (u0 < -9.0F) {
      u0 = -9.0F;
    }
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Update for DiscreteTransferFcn: '<S3>/Discrete Transfer Fcn' incorporates:
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   *  Gain: '<S3>/Gain1'
   *  Sum: '<S3>/Sum1'
   */
  dModel_DW.current = (u0 - 0.0238732416F * dModel_DW.speed) - -0.939413071F *
    dModel_DW.current;

  /* Update for Memory: '<S11>/Memory' incorporates:
   *  Constant: '<S12>/Lower Limit'
   *  Constant: '<S12>/Upper Limit'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
   *  Logic: '<S11>/AND'
   *  Logic: '<S12>/AND'
   *  RelationalOperator: '<S12>/Lower Test'
   *  RelationalOperator: '<S12>/Upper Test'
   */
  dModel_DW.Memory_PreviousInput = (rtb_AND && ((-0.5F <= dModel_DW.speed) &&
    (dModel_DW.speed <= 0.5F)));

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' */
  dModel_DW.speed += 0.001F * rtb_acceleration;
  dModel_DW.DiscreteTimeIntegrator_PrevResetState = (int8_T)rtb_Memory;

  /* Update for Memory: '<S15>/Memory' incorporates:
   *  Constant: '<S17>/Constant'
   *  Constant: '<S18>/Lower Limit'
   *  Constant: '<S18>/Upper Limit'
   *  Constant: '<S19>/Lower Limit'
   *  Constant: '<S19>/Upper Limit'
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'
   *  Logic: '<S15>/AND'
   *  Logic: '<S15>/OR'
   *  Logic: '<S18>/AND'
   *  Logic: '<S19>/AND'
   *  RelationalOperator: '<S17>/Compare'
   *  RelationalOperator: '<S18>/Lower Test'
   *  RelationalOperator: '<S18>/Upper Test'
   *  RelationalOperator: '<S19>/Lower Test'
   *  RelationalOperator: '<S19>/Upper Test'
   */
  dModel_DW.Memory_PreviousInput_i = (((-0.000717F <= rtb_torque) && (rtb_torque
    <= 0.000717F) && ((-0.001F <= dModel_DW.speed_h) && (dModel_DW.speed_h <=
    0.001F))) || (dModel_DW.DiscreteTimeIntegrator1_SatPortCache != 0));

  /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S5>/Gain'
   */
  dModel_DW.speed_h += 80.6451645F * rtb_torque * 0.001F;
  dModel_DW.DiscreteTimeIntegrator_PrevResetState_p = (int8_T)rtb_Memory_o;

  /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */
  dModel_DW.position += 0.001F * rtb_speed_d;
  if (dModel_DW.position >= 3.14159274F) {
    dModel_DW.position = 3.14159274F;
    dModel_DW.DiscreteTimeIntegrator1_SatPortCache = 1;
  } else if (dModel_DW.position <= -3.14159274F) {
    dModel_DW.position = -3.14159274F;
    dModel_DW.DiscreteTimeIntegrator1_SatPortCache = -1;
  } else {
    dModel_DW.DiscreteTimeIntegrator1_SatPortCache = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */
}

/* Model initialize function */
void dModel_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(dModel_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&dModel_DW, 0,
                sizeof(DW_dModel_T));

  /* external inputs */
  dModel_U.Setpoint = 0.0F;

  /* external outputs */
  (void) memset((void *)&dModel_Y, 0,
                sizeof(ExtY_dModel_T));

  /* InitializeConditions for Memory: '<S11>/Memory' */
  dModel_DW.Memory_PreviousInput = true;

  /* InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' */
  dModel_DW.DiscreteTimeIntegrator_PrevResetState = 0;

  /* InitializeConditions for Memory: '<S15>/Memory' */
  dModel_DW.Memory_PreviousInput_i = true;

  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  dModel_DW.DiscreteTimeIntegrator_PrevResetState_p = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */
  dModel_DW.DiscreteTimeIntegrator1_SatPortCache = 0;
}

/* Model terminate function */
void dModel_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
