// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************

#include "ReactionWheel.h"

static bool ActiveConnection_Flag;

ExperimentParametersType ExperimentParameters;

/** Read current connection status
 * @return true if client connected, false otherwise.
 */
bool ConnectionIsActive(void)
{
    return(ActiveConnection_Flag);
}

/** Local use only*/
static void UpdateExperimentParameters(float values[])
{
    int i;
    ExperimentParameters.law = values[0];
    ExperimentParameters.option = values[1];
    ExperimentParameters.duration = round(values[2]*1000.0); // s => ms
    ExperimentParameters.acquisitionPeriod = round(values[3]*1000.0); // s => ms
    ExperimentParameters.lawPeriod  = round(values[4]*1000.0); // s => ms
    ExperimentParameters.setpointProfile = values[5];
    ExperimentParameters.endSetpointValue = values[6];
    ExperimentParameters.riseDurationSetup = round(values[7]*1000.0); // s => ms
    
    ExperimentParameters.maxSpeed = values[8];
    ExperimentParameters.accel = values[9];

    for(i = 0; i < 10; i++) {
        ExperimentParameters.lawCoeff[i] = values[10 + i];
    }
    rt_printf(" duration=%d , LawP=%d, AcqP=%d (ms)\r\n", ExperimentParameters.duration, ExperimentParameters.lawPeriod, ExperimentParameters.acquisitionPeriod);
}

/** Acquire the sensors values.\n
 * Send the values to the Human Interface.\n
 * 
 * This function :
 * <li> Is called by ManageRequest()
 * <li> Is the answer of 'C' request from the Human Interface
 * <li> Is activated only the initialization state of the experiment
 */
void SendPresentSensorsValues(void)
{
    SampleType sample;
    SampleAcquisition(&sample);

    float sensor[4];
    sensor[0] = sample.motorSpeed;
    sensor[1] = sample.platformSpeed;
    sensor[2] = sample.platformPosition;
    sensor[3] = sample.motorCurrent;

    WriteRealArray('T', sensor, 4);
}

/** Manage the dialog with the Human Interface
 * <p> This function is called by HumanMachineInterface_Task().
 */
void ManageRequest(void)
{
#define portNumber 7800
    
    float realsArray[MAX_REAL_ARRAY_SIZE];

    char request, datatype;

    OpenSocketServer(portNumber);   // exit on fail
    
    while(true) { // infinite loop
        rt_printf("Server waiting for connection on port %d\r\n", portNumber);
        
        ActiveConnection_Flag = WaitForConnection(); // Blocking call
        
        if(ActiveConnection_Flag == true)
            rt_printf(" Connection activated\r\n");
        else
            rt_printf(" Failed to activate connection\r\n");
        
        // Reading request loop
        while(ActiveConnection_Flag == true) {
            
            WaitForMessage(); // Blocking call
            request  = ReturnRequest();
            datatype = ReturnDataType();
            switch(request) {
            case 'L': // Set command laws parameters
                if(datatype == REAL_ARRAY_TYPE) {
                    ReturnReceivedRealArray(realsArray);
                    UpdateExperimentParameters(realsArray);
                    WriteResponse('A');
                }
                break;
            case 'D': // Start experiment 
                if(datatype == EMPTY_TYPE) {
                    HardwareReset(); // Reset Simulated wheel
                    StartExperiment();
                    WriteResponse('A');
                }
                break;
            case 'A': // Abort experiment
                if(datatype == EMPTY_TYPE) {
                    AbortExperiment();
                    WriteResponse('A');
                }
                break;
            case 'C': // Read present sensors values 
                if(datatype == EMPTY_TYPE) {
                    SendPresentSensorsValues();
                }
                break;
            case 'B': // Read last samples block 
                if(datatype == EMPTY_TYPE) {
                    ReturnSensorsMeasurement();
                }
                break;
            case ERROR_TYPE: // On read error disconnect
                ActiveConnection_Flag = false;
                Disconnect();
                rt_printf(" Connection closed\r\n");
                break;
            case 'E': // Old Calibration (to be removed)
                WriteResponse('A');
                break;
            default: // Unkown request : do nothing
                break;
            }            
        }
    }
}
