// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2024                       *
// *                                                         *
// * Student version                                         *
// ***********************************************************
// Xenomai online documentation : 
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy.html

#include "ReactionWheel.h"

// Local constants
#define VIT_MAX  0.5            // rd/s
#define ACCEL    (VIT_MAX/10)   // rd/s2  

// Local variables
float   Y_n0, Y_n1, Y_n2;
float   U_n0, U_n1, U_n2;
float   Ki, Kp, Kv;             // Law coefficients
float   A, B, C, D, E;
int     LawParameter;
float   PeriodLaw;
float   error,error_n1,inte,inte_n1, error_n2;
float   wheelCommand_n1, wheelCommand_n2;

// Global variables
// declared in WheelHMI.c 
extern ExperimentParametersType ExperimentParameters;

/** Calls InitializeLaw and InitializeSetpoint functions
 * @param sample : sensors values\n
 * @Note This function must be called before each experiment
 * */
void InitializeExperiment(SampleType sample)
{
    InitializeLaw();
    InitializeSetpoint(sample.platformPosition);
}

/** Initialize the law coefficients and parameters
 * 
 * The structure expParameters is initialized by the function UpdateExperimentParameters (in WheelHMI.c file) \n
 * This function is called by "InitializeExperiment" 
 */
void InitializeLaw(void)
{
    Y_n1 = 0;
    U_n1 = 0;
    U_n2 = 0;
    Y_n2 = 0;
    wheelCommand_n1 = 0;
    wheelCommand_n2 = 0;
    error_n1 = 0;
    error_n2 = 0;
    inte_n1 = 0;
    LawParameter = ExperimentParameters.law;
    PeriodLaw = (float) ((ExperimentParameters.lawPeriod) / 1000.0);
    switch(LawParameter) {
    case 10:
        Kv = ExperimentParameters.lawCoeff[0];
        break;
    case 11:
        Ki = ExperimentParameters.lawCoeff[0];
        Kp = ExperimentParameters.lawCoeff[1];
        Kv = ExperimentParameters.lawCoeff[2];
        break;
    case 12:
        A = ExperimentParameters.lawCoeff[0];
        B = ExperimentParameters.lawCoeff[1];
        C = ExperimentParameters.lawCoeff[2];
        Kv = ExperimentParameters.lawCoeff[3];
        break;
    case 20:
        A = ExperimentParameters.lawCoeff[0];
        B = ExperimentParameters.lawCoeff[1];
        C = ExperimentParameters.lawCoeff[2];
        break;
    case 21:
        A = ExperimentParameters.lawCoeff[0];
        B = ExperimentParameters.lawCoeff[1];
        C = ExperimentParameters.lawCoeff[2];
        D = ExperimentParameters.lawCoeff[3];
        E = ExperimentParameters.lawCoeff[4];
        break;
    case 50:
        Kp = ExperimentParameters.lawCoeff[0];
        Kv = ExperimentParameters.lawCoeff[1];
        break;
    case 51:
        Ki =  ExperimentParameters.lawCoeff[0];
        Kp =  ExperimentParameters.lawCoeff[1];
        Kv =  ExperimentParameters.lawCoeff[2];
        break;
    }
 
}

/** Compute a new command 
 * @param measure : sensors values\n
 * @return The command to be sent to the motor
 * <li> This function must be called at each law period
 * <li> The setpoint is updated each time this function is called
 */
float ComputeLaw(SampleType measure)
{
    float wheelCommand, speed, position, error;
    float currentSetpoint;

    speed    = measure.platformSpeed;
    position = measure.platformPosition;

    currentSetpoint = ComputeNewSetpoint();

    error    = currentSetpoint - position;
    switch(LawParameter) {
    default:
    case 0:	   // openLoop
        wheelCommand = currentSetpoint;
        break;

    case 10:   // speed loop
        Kv = 12;
        wheelCommand = (currentSetpoint - speed) * Kv;
        break;

    case 11:   // continuous position loop
        Kp = 2;
        Kv = 2;

        U_n0 =  (error) * Ki;
        Y_n0 = Y_n1 + ((PeriodLaw / 2.0)*(U_n0 + U_n1));
        Y_n1 = Y_n0;
        U_n1 = U_n0;
        wheelCommand = (((error * Kp) + Y_n0) - speed) * Kv;
        break;
    case 12:  // discreet position loop
        

        break;
    case 20:   // lead compensator loop
        A = 3.335;
        B = -3.15;
        C = -0.715;
        
        wheelCommand = A*error + B*error_n1 - C*wheelCommand_n1;
        wheelCommand_n1 = wheelCommand;
        error_n1=error;
        break;
        
    case 21:   // lead  lag compensator loop
        A = 3.324;
        B = -6.315;
        C = 2.997;
        D = -1.67;
        E = 0.671;
        
        wheelCommand = A*error + B*error_n1 + C*error_n2 - D*wheelCommand_n1 - E*wheelCommand_n2;
        wheelCommand_n2 = wheelCommand_n1;
        wheelCommand_n1= wheelCommand;
        error_n2 = error_n1;
        error_n1 = error ;
        break;
        
    case 50:   // state feedback
        Kp = 2.53;
        Kv = 3.7616;
                
        wheelCommand = Kp*(currentSetpoint - position) - Kv*speed; 
        break;

    case 51:   // state feedback with integrator
       /* Ki = 24.1647;
        Kp = 16.4227;
        Kv = 5.3531;*/

        Ki = 9.4041;
        Kp = 10.1015;
        Kv = 3.8551;        

        inte = PeriodLaw * (error+error_n1)/2 + inte_n1;
        wheelCommand = Ki*inte - Kv * speed - Kp*position;
        error_n1=error;
        inte_n1 = inte;
        break;
    }//  end of switch (lawParameter)
    return(wheelCommand);
}
