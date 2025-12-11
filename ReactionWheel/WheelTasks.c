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
#define MANAGE_LAW_ACQUIRE_PRIORITY          37
#define TIME_MANAGEMENT_PERIOD_NS            1000000  // 1ms in nanoseconds
//********   This space must be completed if needed *****  

//--------------------------------------------
// Tasks descriptors                          
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__task.html

RT_TASK HumanMachineInterface_TaskDescriptor;
RT_TASK ManageLawAcquire_TaskDescriptor;
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

    // Set task periodic at 1ms
    rt_task_set_periodic(NULL, TM_NOW, TIME_MANAGEMENT_PERIOD_NS);

    while(1) {
        rt_task_wait_period(NULL);

        if(experimentRunning) {
            // Update elapsed time
            ExperimentElapsedMs++;

            // Check for experiment duration timeout
            if(ExperimentElapsedMs >= ExperimentParameters.duration * 1000) {
                experimentRunning = false;
                rt_event_signal(&ExperimentControl_Event, EVENT_FINISHED);
                rt_sem_v(&ExitApplication_Semaphore);
            }
        }
    }
}

void ManageLawAcquire_Task()
{
    rt_printf("Starting Manage Law Acquire task\r\n");

    SampleType sample;
    SensorMessage msg;
    bool first_iteration = true;
    bool periodic_set = false;

    while(1) {
        // Set task periodic with lawPeriod once experiment starts
        if(experimentRunning && !periodic_set) {
            rt_task_set_periodic(NULL, TM_NOW, (unsigned long long)ExperimentParameters.lawPeriod * 1000000ULL);
            periodic_set = true;
        }
        
        if(periodic_set) {
            rt_task_wait_period(NULL);
        } else {
            rt_task_sleep(10000000); // 10ms sleep while waiting for experiment to start
        }
        
        if(experimentRunning) {
            // 1. Acquire sensor data
            SampleAcquisition(&sample);

            // 2. On first iteration: Initialize experiment
            if (first_iteration) {
                InitializeExperiment(sample);
                first_iteration = false;
            }

            // 3. Compute control law
            float command = ComputeLaw(sample);

            // 4. Apply command to motor
            ApplySetpointCurrent(command);

            // 5. Store data in queue for HMI
            msg.motorSpeed = sample.motorSpeed;
            msg.platformSpeed = sample.platformSpeed;
            msg.platformPosition = sample.platformPosition;
            msg.motorCurrent = sample.motorCurrent;
            msg.timestamp = ExperimentElapsedMs;
            
            rt_queue_write(&SensorData_Queue, &msg, sizeof(SensorMessage), Q_NORMAL);
        } else {
            first_iteration = true;
            periodic_set = false;
        }
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
    
    // Stop motor
    ApplySetpointCurrent(0.0f);
    
    // Clear queue
    rt_queue_flush(&SensorData_Queue);
    
    // Send 'F' termination character
    float emptyBlock[1] = {0.0f};
    WriteRealArray('F', emptyBlock, 1);
}

void StartExperiment(void)
{
    rt_printf("Started\r\n");
    
    // Set experiment running flag
    experimentRunning = true;
    
    // Reset elapsed time
    ExperimentElapsedMs = 0;
    
    // Clear the sensor queue
    rt_queue_flush(&SensorData_Queue);
    
    // Signal start event
    rt_event_signal(&ExperimentControl_Event, EVENT_START);
}

//----------------------------------------------------------

void ReturnSensorsMeasurement()
{
    SampleType sample;
    SensorMessage msg;
    
    float samplesBlock[50 * 4];
    char terminationChar;
    int i;
    float AP = ExperimentParameters.acquisitionPeriod ;
    float LP = ExperimentParameters.lawPeriod ;
    int n = (int)(LP/AP); 
    
    for(i = 0; i < n; i++) {
        if (rt_queue_read(&SensorData_Queue,&msg, sizeof(SensorMessage), TM_INFINITE) >0) {
        
        samplesBlock[(i * 4)]     = msg.motorSpeed;
        samplesBlock[(i * 4) + 1] = msg.platformSpeed;
        samplesBlock[(i * 4) + 2] = msg.platformPosition;
        samplesBlock[(i * 4) + 3] = msg.motorCurrent;
        }
    }
   
    terminationChar = experimentRunning ? 'S' : 'F';  
    
    // terminationChar = 'F' if the experiment is finished
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
    // myFlag = false;
    // *** This space must be completed  if needed   *****

    // ------------------------------------- 
    // Events creation               
    rt_event_create(&ExperimentControl_Event, "ExperimentControl", 0, EV_FIFO);
    
    // Message queues creation               
    // ------------------------------------- 
    // *** This space must be completed  if needed   ******
    rt_queue_create(&SensorData_Queue,"SensorQueue",
                    SENSOR_QUEUE_SIZE * SENSOR_MSG_SIZE, SENSOR_QUEUE_SIZE,Q_FIFO);

    // Semaphores creation                 
    // ------------------------------------
    rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
    rt_sem_create(&StartExperiment_Semaphore, "StartExp", 0, S_FIFO);


    // **** This space must be completed  if needed   ***** 
    

    // Mutual exclusion semaphore creation                     
    //---------------------------------------------------------
    rt_mutex_create(&DataMutex, "DataMutex");
    // **** This space must be completed  if needed   ***** 

    // Tasks creation                                          
    //---------------------------------------------------------
    rt_task_create(&HumanMachineInterface_TaskDescriptor, "ReturnSensorsMeasurementestTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);
    rt_task_create(&TimeManagement_TaskDescriptor, "TimeManagementTask", DEFAULTSTACKSIZE, TIME_MANAGEMENT_PRIORITY, 0);
    rt_task_create(&ManageLawAcquire_TaskDescriptor, "ManageLawAcquireTask", DEFAULTSTACKSIZE, MANAGE_LAW_ACQUIRE_PRIORITY, 0);



    // Tasks starting
    rt_task_start(&HumanMachineInterface_TaskDescriptor, &HumanMachineInterface_Task, NULL);
    rt_task_start(&TimeManagement_TaskDescriptor, &TimeManagement_Task, NULL);
    rt_task_start(&ManageLawAcquire_TaskDescriptor, &ManageLawAcquire_Task, NULL);
    
// **** This space must be completed  if needed   ***** 

    //-----------------------------------------------------------
    // Main Task waits on exit semaphore                          
    //-----------------------------------------------------------
    rt_sem_p(&ExitApplication_Semaphore, TM_INFINITE);

    //-----------------------------------------------------------
    // Tasks destruction                                         
    //-----------------------------------------------------------
    // rt_task_ delete(...);                                 
    rt_printf("\r\n \r\n Destruction of \r\n  - Tasks\r\n  - Queues\r\n  - Semaphores\r\n  - Events\r\n  - Mutexes\r\n");
    // **** This space must be completed  if needed   ***** 
    rt_task_delete(&HumanMachineInterface_TaskDescriptor);
    rt_task_delete(&TimeManagement_TaskDescriptor);
    rt_task_delete(&ManageLawAcquire_TaskDescriptor);
    //------------------------------------------------------------
    // Semaphores destruction                                     
    //-------------------------------------------------------------
    rt_sem_delete(&ExitApplication_Semaphore);
    rt_sem_delete(&StartExperiment_Semaphore);

    // **** This space must be completed  if needed   ***** 
    
    //------------------------------------------------------------
    // Events destruction                                 
    //------------------------------------------------------------
    rt_event_delete(&ExperimentControl_Event);
   
    
    //------------------------------------------------------------
    // Message queues destruction                                 
    //------------------------------------------------------------
    rt_queue_delete(&SensorData_Queue);                                 
    // **** This space must be completed  if needed   *****
    rt_mutex_delete(&DataMutex);

    rt_printf(" Application ..... finished--> exit\r\n");
    // Peripherals uninitialization 
    HardwareTerminate();
    return (0);
}
