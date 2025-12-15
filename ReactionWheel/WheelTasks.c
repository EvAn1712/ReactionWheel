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
#define TIME_MANAGEMENT_PERIOD_NS            1000000  // 1ms in nanoseconds
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
            // Update elapsed time (single writer, volatile ensures visibility to readers)
            ExperimentElapsedMs++;

            // Check for experiment duration timeout
            if(ExperimentElapsedMs >= ExperimentParameters.duration) {
                
                rt_printf("Task finished \r\n");
                
                // Use mutex to protect experimentRunning flag
                rt_mutex_acquire(&DataMutex, TM_INFINITE);
                experimentRunning = false;
                rt_mutex_release(&DataMutex);
                
                rt_event_signal(&ExperimentControl_Event, EVENT_ABORT);
    
                // Clear queue
                rt_queue_flush(&SensorData_Queue);

                // Send 'F' termination character
                float emptyBlock[1] = {0.0};
                WriteRealArray('F', emptyBlock, 1);
            }
        }
    }
}

void Acquisition_Task()
{
    rt_printf("Starting Acquisition task\r\n");

    SampleType sample;
    SensorMessage msg;
    bool periodic_set = false;
    unsigned long long acquisitionPeriodNs = 0;

    while(1) {
        // Check if experiment is running
        rt_mutex_acquire(&DataMutex, TM_INFINITE);
        bool running = experimentRunning;
        rt_mutex_release(&DataMutex);
        
        // Set task periodic with acquisitionPeriod once experiment starts
        if(running && !periodic_set) {
            // Convert milliseconds to nanoseconds (calculated once)
            acquisitionPeriodNs = (unsigned long long)ExperimentParameters.acquisitionPeriod * 1000000;
            rt_task_set_periodic(NULL, TM_NOW, acquisitionPeriodNs);
            periodic_set = true;
        }
        
        if(periodic_set && running) {
            rt_task_wait_period(NULL);
            
            // Acquire sensor data
            SampleAcquisition(&sample);
            
            // Populate message structure
            msg.motorSpeed = sample.motorSpeed;
            msg.platformSpeed = sample.platformSpeed;
            msg.platformPosition = sample.platformPosition;
            msg.motorCurrent = sample.motorCurrent;
            msg.timestamp = ExperimentElapsedMs;
            
            // Write to queue
            rt_queue_write(&SensorData_Queue, &msg, sizeof(SensorMessage), Q_NORMAL);
        } else {
            // When not in periodic mode, sleep briefly
            if(periodic_set && !running) {
                // Stop periodic mode
                periodic_set = false;
            }
            rt_task_sleep(10000000); // 10ms sleep while waiting for experiment to start
        }
    }
}

void ControlLaw_Task()
{
    rt_printf("Starting Control Law task\r\n");

    SampleType sample;
    bool periodic_set = false;
    unsigned long long lawPeriodNs = 0;

    while(1) {
        // Check if experiment is running
        rt_mutex_acquire(&DataMutex, TM_INFINITE);
        bool running = experimentRunning;
        rt_mutex_release(&DataMutex);
        
        // Set task periodic with lawPeriod once experiment starts
        if(running && !periodic_set) {
            // Convert milliseconds to nanoseconds (calculated once)
            lawPeriodNs = (unsigned long long)ExperimentParameters.lawPeriod * 1000000;
            rt_task_set_periodic(NULL, TM_NOW, lawPeriodNs);
            periodic_set = true;
        }
        
        if(periodic_set && running) {
            rt_task_wait_period(NULL);
            
            // Acquire current sensor data for control law computation
            // Note: We acquire directly rather than reading from queue because:
            // 1. Control law needs the most current sensor data for stability
            // 2. Queue data may be stale if lawPeriod != acquisitionPeriod
            // 3. The queue is primarily for data logging/HMI communication
            SampleAcquisition(&sample);
            
            // Compute control law
            float command = ComputeLaw(sample);
            
            // Apply command to motor
            ApplySetpointCurrent(command);
        } else {
            // When not in periodic mode, sleep briefly
            if(periodic_set && !running) {
                // Stop periodic mode
                periodic_set = false;
            }
            
            // Apply zero current when not running
            ApplySetpointCurrent(0.0);
            
            rt_task_sleep(10000000); // 10ms sleep while waiting for experiment to start
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
    
    // Set experiment running flag to false (use mutex for safety)
    rt_mutex_acquire(&DataMutex, TM_INFINITE);
    experimentRunning = false;
    rt_mutex_release(&DataMutex);
    
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
    
    // Get initial sensor reading for initialization
    SampleType sample;
    SampleAcquisition(&sample);
    
    // Initialize experiment before starting
    InitializeExperiment(sample);
    
    // Reset elapsed time
    ExperimentElapsedMs = 0;
    
    // Clear the sensor queue
    rt_queue_flush(&SensorData_Queue);
    
    // Set experiment running flag (use mutex for safety)
    rt_mutex_acquire(&DataMutex, TM_INFINITE);
    experimentRunning = true;
    rt_mutex_release(&DataMutex);
    
    // Signal start event
    rt_event_signal(&ExperimentControl_Event, EVENT_START);
}

//----------------------------------------------------------

void ReturnSensorsMeasurement()
{
    SensorMessage msg;
    RT_QUEUE_INFO queueInfo;
    
    float samplesBlock[50 * 4];
    char terminationChar;
    int i;
    int messagesRead = 0;
    int maxMessages = 50; // Maximum block size we can handle
    
    // Query the queue to get the actual number of messages available
    int ret = rt_queue_inquire(&SensorData_Queue, &queueInfo);
    if (ret < 0) {
        rt_printf("Error querying queue: %d\r\n", ret);
        // Send empty block with 'F' termination
        float emptyBlock[1] = {0.0};
        WriteRealArray('F', emptyBlock, 1);
        return;
    }
    
    // Calculate number of messages to read (min of available and max we can handle)
    int messagesToRead = queueInfo.nmessages;
    if (messagesToRead > maxMessages) {
        messagesToRead = maxMessages;
    }
    
    // Read available messages from the queue
    for(i = 0; i < messagesToRead; i++) {
        ret = rt_queue_read(&SensorData_Queue, &msg, sizeof(SensorMessage), TM_NONBLOCK);
        if (ret > 0) {
            samplesBlock[(messagesRead * 4)]     = msg.motorSpeed;
            samplesBlock[(messagesRead * 4) + 1] = msg.platformSpeed;
            samplesBlock[(messagesRead * 4) + 2] = msg.platformPosition;
            samplesBlock[(messagesRead * 4) + 3] = msg.motorCurrent;
            messagesRead++;
        } else {
            // No more messages available or error occurred
            break;
        }
    }
    
    // Check experiment status with mutex protection
    rt_mutex_acquire(&DataMutex, TM_INFINITE);
    bool running = experimentRunning;
    rt_mutex_release(&DataMutex);
    
    terminationChar = running ? 'S' : 'F';  
    
    // Send the data block
    WriteRealArray(terminationChar, samplesBlock, messagesRead * 4);
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
    rt_task_create(&HumanMachineInterface_TaskDescriptor, "HMITask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);
    rt_task_create(&TimeManagement_TaskDescriptor, "TimeManagementTask", DEFAULTSTACKSIZE, TIME_MANAGEMENT_PRIORITY, 0);
    rt_task_create(&Acquisition_TaskDescriptor, "AcquisitionTask", DEFAULTSTACKSIZE, ACQUISITION_PRIORITY, 0);
    rt_task_create(&ControlLaw_TaskDescriptor, "ControlLawTask", DEFAULTSTACKSIZE, CONTROL_LAW_PRIORITY, 0);



    // Tasks starting
    rt_task_start(&HumanMachineInterface_TaskDescriptor, &HumanMachineInterface_Task, NULL);
    rt_task_start(&TimeManagement_TaskDescriptor, &TimeManagement_Task, NULL);
    rt_task_start(&Acquisition_TaskDescriptor, &Acquisition_Task, NULL);
    rt_task_start(&ControlLaw_TaskDescriptor, &ControlLaw_Task, NULL);
    
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
    rt_task_delete(&Acquisition_TaskDescriptor);
    rt_task_delete(&ControlLaw_TaskDescriptor);
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
