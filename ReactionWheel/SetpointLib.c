// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************

#include "ReactionWheel.h"

// Local variables
static int   SetpointProfile;
static float CurrentSetPoint, FinalSetPoint, SetpointIncrementation, HalfWayProfile, TheoreticalSpeed;
static float MotionDirection, SetpointPeriod, RiseDuration;
static bool  SetpointReached;
static float maxSpeed; // for speed profile
static float accel;    // for speed profile

// Global variables
// declared in WheelHMI.c 
extern ExperimentParametersType ExperimentParameters;

/** Returns the current setPoint\n
 * This function is not used by default
 * @return setpoint in radians
 */
float ReturnCurrentSetpoint(void)
{
    return(CurrentSetPoint);
}

/** Computes and returns a new setPoint\n
 * 
 * This function is called by ComputeLaw in (WheelLaws.c) file
 * 
 * Computes and returns a new set point\n
 * 
 * The setpoint can be : depending on ExperimentParameters (field setPointProfile)
 * <li> 0 : Constant (a step)
 * <li> 1 : A ramp  set point
 * <li> 2 : speed profile
 */
float ComputeNewSetpoint(void)
{
    float setPoint;
    float accelerationSign;
    float practicalSpeed = 0;

    switch(SetpointProfile) {
    default:
    case 0: // Constant setpoint
        setPoint = FinalSetPoint;
        CurrentSetPoint = setPoint;
        break;
    case 1: // Ramp setpoint
        setPoint = CurrentSetPoint + SetpointIncrementation;
        if((setPoint >= FinalSetPoint) && (SetpointIncrementation > 0)) setPoint = FinalSetPoint;
        if((setPoint <= FinalSetPoint) && (SetpointIncrementation < 0)) setPoint = FinalSetPoint;
        CurrentSetPoint = setPoint;
        break;
    case 2: // Speed profile setpoint 
        setPoint = CurrentSetPoint;
        if(SetpointReached == false) {
            if((setPoint < HalfWayProfile) && (MotionDirection == +1)) accelerationSign = +1; // 1ere moitié sens pos
            if((setPoint > HalfWayProfile) && (MotionDirection == +1)) accelerationSign = -1; // 2eme moitié sens pos
            if((setPoint > HalfWayProfile) && (MotionDirection == -1)) accelerationSign = -1; // 1ere moitié sens neg
            if((setPoint < HalfWayProfile) && (MotionDirection == -1)) accelerationSign = +1; // 2eme moitié sens neg

            TheoreticalSpeed = (TheoreticalSpeed + SetpointPeriod * accel * accelerationSign);

            // speed saturation
            if(TheoreticalSpeed > maxSpeed)
                practicalSpeed = maxSpeed;
            else
                practicalSpeed = TheoreticalSpeed;

            setPoint = setPoint + (practicalSpeed * SetpointPeriod);
        }

        if((practicalSpeed < 0) && (MotionDirection > 0) && (SetpointReached == false)) SetpointReached = true; // setpoint detection reached
        if((practicalSpeed > 0) && (MotionDirection < 0) && (SetpointReached == false)) SetpointReached = true; // setpoint detection reached

        if(SetpointReached == true) setPoint = FinalSetPoint;

        CurrentSetPoint = setPoint;
        break;
    }
    return(setPoint);
}

/** 
 * @param PlatformPosition : actual platform position in radians
 *
 * The setpoint can be : depending on ExperimentParameters (field setPointProfile)
 * <li> 0 : Constant (a step)
 * <li> 1 : A ramp  set point
 * <li> 2 : speed profile
 */
void InitializeSetpoint(float PlatformPosition)
{
    SetpointProfile = ExperimentParameters.setpointProfile;
    SetpointPeriod = (float) ((ExperimentParameters.lawPeriod) / 1000.0);
    RiseDuration = (float) ((ExperimentParameters.riseDurationSetup) / 1000.0);
    switch(SetpointProfile) {
    case 0:
        SetpointIncrementation = 0;
        FinalSetPoint = ExperimentParameters.endSetpointValue;
        break;

    case 1:
        FinalSetPoint = ExperimentParameters.endSetpointValue;
        SetpointIncrementation = ((ExperimentParameters.endSetpointValue - PlatformPosition) * SetpointPeriod) / RiseDuration;
        CurrentSetPoint = PlatformPosition;
        break;

    case 2:
        SetpointReached = false;
        HalfWayProfile = (ExperimentParameters.endSetpointValue + PlatformPosition) / 2;
        CurrentSetPoint = PlatformPosition;
        FinalSetPoint = ExperimentParameters.endSetpointValue;
        if(ExperimentParameters.endSetpointValue > PlatformPosition)
            MotionDirection = +1.0;
        else
            MotionDirection = -1.0;
        TheoreticalSpeed = 0;
        maxSpeed = ExperimentParameters.maxSpeed;
        accel = ExperimentParameters.accel;
        break;
    }
}
