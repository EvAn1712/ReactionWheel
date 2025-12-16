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

//--------------------------------------------
// Global  declarations                      
//--------------------------------------------

#define HUMAN_MACHINE_INTERFACE_PRIORITY     33
#define TIME_MANAGEMENT_PRIORITY             35
#define ACQUISITION_PRIORITY                 36
#define CONTROL_LAW_PRIORITY                 37
//********   This space must be completed if needed *****  

//--------------------------------------------
// Tasks descriptors                          
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__task.html

RT_TASK HumanMachineInterface_TaskDescriptor;
RT_TASK Acquisition_TaskDescriptor;
RT_TASK ControlLaw_TaskDescriptor;
RT_TASK TimeManagement_TaskDescriptor;
RT_TASK ManageRequest_TaskDescriptor;

//-------------------------------------------
//  Semaphores descriptors                   
//-------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__sem.html

RT_SEM StartExperiment_Semaphore; //start
RT_SEM ExitApplication_Semaphore; //finished
//********   This space must be completed if needed *****  

//--------------------------------------------
//  Events descriptors
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__event.html

RT_EVENT ExperimentControl_Event;

#ifndef BIT
#define BIT(n) (1<<n)
#endif
#define EVENT_START    BIT(0)
#define EVENT_ABORT    BIT(1)
#define EVENT_FINISHED BIT(2)  
#define EVENT_LP    BIT(3)
#define EVENT_AP    BIT(4)
#define EVENT_ALL (EVENT_AP | EVENT_LP | EVENT_START | EVENT_ABORT | EVENT_FINISHED)

//--------------------------------------------
// Mutual exclusion semaphores descriptors    
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__mutex.html
RT_MUTEX DataMutex;
//********   This space must be completed if needed *****  

//--------------------------------------------
// Message queues descriptors                 
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__queue.html

//********   This space must be completed if needed *****  
RT_QUEUE SensorData_Queue;

#define SENSOR_QUEUE_SIZE 100
#define SENSOR_MSG_SIZE sizeof(SensorMessage)

typedef struct {
    float motorSpeed;
    float platformSpeed;
    float platformPosition;
    float motorCurrent;
    unsigned long timestamp;
} SensorMessage;


//--------------------------------------------------------------------------
// Global variables communication and synchronization tasks by shared memory 
//--------------------------------------------------------------------------
// bool myFlag
volatile bool experimentRunning = false;
volatile unsigned long ExperimentElapsedMs = 0;
volatile unsigned long lawTickCount = 0;
volatile unsigned long acquisitionTickCount = 0;
//**** This space must be completed if needed *****  


// declared in WheelHMI.c 
extern ExperimentParametersType ExperimentParameters;

//--------------------------------------------------------------------
//
//                  The Tasks
//
//--------------------------------------------------------------------

void HumanMachineInterface_Task()
{
    rt_printf("Starting Human/Machine Interface task\r\n");
    ManageRequest();
}
void TimeManagement_Task()
{
    rt_printf("Starting Time Management task\r\n");


    while(1) {
        rt_task_wait_period(NULL);

        if(experimentRunning == true){
            if(ConnectionIsActive()){
                ExperimentElapsedMs++;
                lawTickCount++;
                acquisitionTickCount++;

                if(ExperimentElapsedMs >= ExperimentParameters.duration){
                    rt_event_signal(&ExperimentControl_Event, EVENT_FINISHED);
                    experimentRunning = false;
                }
                if(lawTickCount >= ExperimentParameters.lawPeriod){
                    rt_event_signal(&ExperimentControl_Event, EVENT_LP);
                    lawTickCount = 0;
                }
                if (acquisitionTickCount >= ExperimentParameters.acquisitionPeriod){
                    rt_event_signal(&ExperimentControl_Event, EVENT_AP);
                    acquisitionTickCount = 0;
                }
            }
        }
    }
}

void ManageLawAndCurves_Task()
{
    rt_printf("Starting Manage Law Acquire task\r\n");

    SampleType sample;
    SensorMessage msg;
    unsigned int maskValue;
    bool first_iteration = true;
    bool periodic_set = false;

    while(1) {
       
        rt_event_wait(&ExperimentControl_Event,EVENT_LP | EVENT_AP | EVENT_START| EVENT_ABORT | EVENT_FINISHED, &maskValue, EV_ANY, TM_INFINITE);
          
        //rt_printf("%d",maskValue);
         
        if (maskValue & EVENT_AP) {
            SampleAcquisition(&sample);
            rt_event_clear(&ExperimentControl_Event, EVENT_AP, NULL);
            msg.motorSpeed = sample.motorSpeed;
            msg.platformSpeed = sample.platformSpeed;
            msg.platformPosition = sample.platformPosition;
            msg.motorCurrent = sample.motorCurrent;
            msg.timestamp = ExperimentElapsedMs;
            rt_queue_write(&SensorData_Queue, &msg, sizeof(SensorMessage), Q_NORMAL);
        }
        
        if (maskValue & EVENT_LP) {
            ApplySetpointCurrent(ComputeLaw(sample));
            rt_event_clear(&ExperimentControl_Event, EVENT_LP, NULL);
        }
        
        if (maskValue & EVENT_START) {
            experimentRunning = true ;
            SampleAcquisition(&sample);
            InitializeExperiment(sample);
            rt_event_clear(&ExperimentControl_Event, EVENT_START, NULL);
        }
        
        if (maskValue & EVENT_ABORT || maskValue & EVENT_FINISHED) {
            ApplySetpointCurrent(0.0);
            experimentRunning = false ; 
            rt_event_clear(&ExperimentControl_Event, EVENT_ABORT, NULL);
            
            rt_event_clear(&ExperimentControl_Event, EVENT_FINISHED, NULL);
        }
        
    /*    if(experimentRunning) {
            
            SampleAcquisition(&sample);
            if (first_iteration) {
                    InitializeExperiment(sample);
                    first_iteration = false;
                }
            
            if (maskValue & EVENT_LP ){
                rt_printf("maskValue\r\n"); }

                float command = ComputeLaw(sample);

                ApplySetpointCurrent(command);
           

            //if (Acquisition) {
                msg.motorSpeed = sample.motorSpeed;
                msg.platformSpeed = sample.platformSpeed;
                msg.platformPosition = sample.platformPosition;
                msg.motorCurrent = sample.motorCurrent;
                msg.timestamp = ExperimentElapsedMs;

                rt_queue_write(&SensorData_Queue, &msg, sizeof(SensorMessage), Q_NORMAL);
            //}
        } else {
            first_iteration = true;
            rt_printf("Else not experiment Running manageLaw\r\n");
            ApplySetpointCurrent(0.0);   
        }*/
    }
}


//-----------------------------------------------------------
// functions called from HMI_Utilities

// This function is called when the user press on Abort during an experiment \n
// called from function "manageRequest" in file WheelHMI.c  
void AbortExperiment(void)
{
    rt_printf("Aborted\r\n");
    
    // Set experiment running flag to false
    experimentRunning = false;
    
    // Signal abort event
    rt_event_signal(&ExperimentControl_Event, EVENT_ABORT);
    
    // Clear queue
    rt_queue_flush(&SensorData_Queue);
    
    // Send 'F' termination character
    float emptyBlock[1] = {0.0};
    WriteRealArray('F', emptyBlock, 1);
}

void StartExperiment(void)
{
    rt_printf("Started\r\n");
    
    // Set experiment running flag
    experimentRunning = true;
    
    // Reset elapsed time
    ExperimentElapsedMs = 0;
    lawTickCount = 0;
    acquisitionTickCount = 0;
    
    // Clear the sensor queue
    rt_queue_flush(&SensorData_Queue);
    
    // Signal start event
    rt_event_signal(&ExperimentControl_Event, EVENT_START);
}

//----------------------------------------------------------

void ReturnSensorsMeasurement()
{
    float sensorBlock[50 * 4];
    char terminationChar;
    int i = 0;
    SampleType sample;

    for (i = 0; i < 50; i++) {
        // Read from the queue
        if (rt_queue_read(&SensorData_Queue, &sample, sizeof(SampleType), TM_NONBLOCK) == sizeof(SampleType)) {
            sensorBlock[i * 4 + 0] = sample.motorSpeed;
            sensorBlock[i * 4 + 1] = sample.platformSpeed;
            sensorBlock[i * 4 + 2] = sample.platformPosition;
            sensorBlock[i * 4 + 3] = sample.motorCurrent;
        } else {
            break; // Exit if no more data is available
        }
    }

    terminationChar = experimentRunning ? 'S' : 'F';

    WriteRealArray(terminationChar, samplesBlock, n * 4);
}

//--------------------------------------------------------------------

// Linux Signals Handler
void stopNow(int sig)
{
    rt_sem_v(&ExitApplication_Semaphore); // give stop semaphore to main task 
}
//--------------------------------------------------------------------
// 
//  main()
// 
//  Description: create and kill all the application's components
// 
//--------------------------------------------------------------------
int main(int argc, char* argv[])
{
    
    // Kernel and peripherals initialization
    HardwareInitialize();
    rt_printf("---- Starting Realtime Application ----\r\n");
    
    // Signal manager to stop the application 
    signal(SIGINT,  stopNow); // Interrupt from keyboard 
    signal(SIGTERM, stopNow); // Termination signal 

    // Global variables initialization       
    // -------------------------------
    experimentRunning = false;
    ExperimentElapsedMs = 0;
    lawTickCount = 0;
    acquisitionTickCount = 0;
    // *** This space must be completed  if needed   *****

    // ------------------------------------- 
    // Events creation               
    rt_event_create(&ExperimentControl_Event, "ExperimentControl", 0, EV_FIFO);
    
    // Message queues creation               
    // ------------------------------------- 
    rt_queue_create(&SensorData_Queue, "SensorQueue",
                    SENSOR_QUEUE_SIZE * SENSOR_MSG_SIZE, SENSOR_QUEUE_SIZE, Q_FIFO);

    // Semaphores creation                 
    // ------------------------------------
    rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
    rt_sem_create(&StartExperiment_Semaphore, "StartExp", 0, S_FIFO);

    // Mutual exclusion semaphore creation                     
    //---------------------------------------------------------
    rt_mutex_create(&DataMutex, "DataMutex");

    // Tasks creation                                          
    //---------------------------------------------------------
    rt_task_create(&HumanMachineInterface_TaskDescriptor, "HumanMachineInterfaceTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);
    rt_task_create(&TimeManagement_TaskDescriptor, "TimeManagementTask", DEFAULTSTACKSIZE, TIME_MANAGEMENT_PRIORITY, 0);
    rt_task_create(&ManageLawAndCurves_TaskDescriptor, "ManageLawAndCurvesTask", DEFAULTSTACKSIZE, CONTROL_LAW_PRIORITY, 0);

    // Tasks starting
    rt_task_start(&HumanMachineInterface_TaskDescriptor, &HumanMachineInterface_Task, NULL);
    rt_task_start(&TimeManagement_TaskDescriptor, &TimeManagement_Task, NULL);
    rt_task_start(&ManageLawAndCurves_TaskDescriptor, &ManageLawAndCurves_Task, NULL);

    //-----------------------------------------------------------
    // Main Task waits on exit semaphore                          
    //-----------------------------------------------------------
    rt_sem_p(&ExitApplication_Semaphore, TM_INFINITE);

    //-----------------------------------------------------------
    // Tasks destruction                                         
    //-----------------------------------------------------------
    rt_printf("\r\n \r\n Destruction of \r\n  - Tasks\r\n  - Queues\r\n  - Semaphores\r\n  - Events\r\n  - Mutexes\r\n");
    rt_task_delete(&HumanMachineInterface_TaskDescriptor);
    rt_task_delete(&TimeManagement_TaskDescriptor);
    rt_task_delete(&ManageLawAndCurves_TaskDescriptor);
    
    //------------------------------------------------------------
    // Semaphores destruction                                     
    //-------------------------------------------------------------
    rt_sem_delete(&ExitApplication_Semaphore);
    rt_sem_delete(&StartExperiment_Semaphore);
    
    //------------------------------------------------------------
    // Events destruction                                 
    //------------------------------------------------------------
    rt_event_delete(&ExperimentControl_Event);
    
    //------------------------------------------------------------
    // Message queues destruction                                 
    //------------------------------------------------------------
    rt_queue_delete(&SensorData_Queue);
    
    //------------------------------------------------------------
    // Mutex destruction                                 
    //------------------------------------------------------------
    rt_mutex_delete(&DataMutex);

    rt_printf(" Application ..... finished--> exit\r\n");
    // Peripherals uninitialization 
    HardwareTerminate();
    return (0);
}
