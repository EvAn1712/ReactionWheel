// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// * Master version                                          *
// ***********************************************************

#include "ReactionWheel.h"

//--------------------------------------------
// Global  declarations                      
//--------------------------------------------

#define HUMAN_MACHINE_INTERFACE_PRIORITY     33
#define LAWS_AND_DISPLAY_PROCESSING_PRIORITY 34
#define TIME_MANAGEMENT_PRIORITY             35

//--------------------------------------------
// Tasks descriptors                          
//--------------------------------------------

RT_TASK HumanMachineInterface_TaskDescriptor;
RT_TASK TimeManagement_TaskDescriptor;
RT_TASK LawsAndDisplayProcessing_TaskDescriptor;

//-------------------------------------------
//  Semaphores identifiers                   
//-------------------------------------------
// RT_SEM mySemaphore;                        
//     **** This space must be completed  *****

RT_SEM StartExperiment_Semaphore;
RT_SEM ExitApplication_Semaphore;

//--------------------------------------------
//  Events identifiers
//--------------------------------------------
// RT_EVENT myEvents;
//     **** This space must be completed  *****
RT_EVENT Experiment_Events;

#define BIT(n) (1<<n)
#define EVENT_INITIALIZE    BIT(0)
#define EVENT_COMPUTELAW    BIT(1)
#define EVENT_READSENSORS   BIT(2)
#define EVENT_FINISH        BIT(3)
#define EVENT_ALL (EVENT_INITIALIZE + EVENT_COMPUTELAW + EVENT_READSENSORS + EVENT_FINISH)

//--------------------------------------------
// Mutual exclusion semaphores identifiers    
//--------------------------------------------
// RT_MUTEX myMutex;                          
//  ******   This space must be completed ****** 

//--------------------------------------------
// Message queues Identifiers                 
//--------------------------------------------
// RT_QUEUE  myQueue;                         
//**** This space must be completed  *****  
RT_QUEUE SensorsMeasurement_Queue;

// Global variables communication and synchronization tasks by shared memory 
//--------------------------------------------------------------------------

bool ExperimentFinished_Flag;
bool AbortExperiment_Flag;

// declared in WheelHMI.c 
extern ExperimentParametersType ExperimentParameters;

// **************************************************************************
// *
// *                  The Tasks
// *
// **************************************************************************

void TimeManagement_Task()
{
    rt_printf("Starting Time management task \r\n");
    int experimentTime;
    rt_task_set_periodic(NULL, TM_NOW, 1000000);
    while(true) { // infinite loop

        rt_sem_p(&StartExperiment_Semaphore, TM_INFINITE);  // wait for semaphore

        rt_event_signal(&Experiment_Events,EVENT_INITIALIZE);
        experimentTime = 0;
        AbortExperiment_Flag = false;
        ExperimentFinished_Flag = false;
        while((experimentTime < ExperimentParameters.duration) && (AbortExperiment_Flag == false) && (ConnectionIsActive() == true)) {
            rt_task_wait_period(NULL);
            experimentTime++;
            if((experimentTime % ExperimentParameters.lawPeriod) == 0) {
                rt_event_signal(&Experiment_Events, EVENT_COMPUTELAW);
            }
            if((experimentTime % ExperimentParameters.acquisitionPeriod) == 0) {
                rt_event_signal(&Experiment_Events, EVENT_READSENSORS);
            }
        }
        ExperimentFinished_Flag = true;
        rt_event_signal(&Experiment_Events, EVENT_FINISH);
  
        if(AbortExperiment_Flag == true)  rt_printf(" Experiment aborted\r\n");
        if(experimentTime >= ExperimentParameters.duration) rt_printf(" End of experiment\r\n");
    }
}

void LawsAndDisplayProcessing_Task()
{
    rt_printf("Starting Law & Display Processing task\r\n");
    unsigned long inputEvents;
    SampleType sample;
    float wheelCommand;

    while(true) { // infinite loop
        
        // wait for any experiment event
        rt_event_wait(&Experiment_Events, EVENT_ALL, &inputEvents, EV_ANY, TM_INFINITE);
        
        SampleAcquisition(&sample);
        if( inputEvents & EVENT_INITIALIZE ) {
            InitializeExperiment(sample);
            rt_event_clear(&Experiment_Events, EVENT_INITIALIZE, NULL);
        }
        if( inputEvents & EVENT_COMPUTELAW ) {
            wheelCommand = ComputeLaw(sample);
            ApplySetpointCurrent(wheelCommand);
            rt_event_clear(&Experiment_Events, EVENT_COMPUTELAW, NULL);
        }
        if( inputEvents & EVENT_READSENSORS ) {
            rt_queue_write(&SensorsMeasurement_Queue, &sample, sizeof(sample), Q_NORMAL);
            rt_event_clear(&Experiment_Events, EVENT_READSENSORS, NULL);
        }
        if( inputEvents & EVENT_FINISH ) {
            ApplySetpointCurrent(0);
            rt_event_clear(&Experiment_Events, EVENT_FINISH, NULL);
        }
    }
}



//-----------------------------------------------------------
// functions called from HMI_Utilities

/** This function is called when the user press on Abort during an experiment \n
 * called from function "manageRequest" in file WheelHMI.c  
 */
void AbortExperiment(void)
{
    AbortExperiment_Flag = true;
}
//-----------------------------------------------------------

void StartExperiment(void)
{
    rt_sem_v(&StartExperiment_Semaphore);   // give semaphore
}
//----------------------------------------------------------

void ReturnSensorsMeasurement()
{
    float samplesBlock[50 * 4];
    SampleType sample;
    RT_QUEUE_INFO queueInfo;
    char terminationChar;

    rt_queue_inquire(&SensorsMeasurement_Queue, &queueInfo);
    int i;
    for(i = 0; i < queueInfo.nmessages; i++) {
        rt_queue_read(&SensorsMeasurement_Queue, &sample, sizeof(sample), TM_NONBLOCK);
        samplesBlock[(i * 4)] = sample.motorSpeed;
        samplesBlock[(i * 4) + 1] = sample.platformSpeed;
        samplesBlock[(i * 4) + 2] = sample.platformPosition;
        samplesBlock[(i * 4) + 3] = sample.motorCurrent;

    }
    if(ExperimentFinished_Flag == false)
        terminationChar = 'S';
    else
        terminationChar = 'F';

    WriteRealArray(terminationChar, samplesBlock, queueInfo.nmessages * 4);

}

//--------------------------------------------------------------------

void HumanMachineInterface_Task()
{
    rt_printf("Starting Human/Machine Interface task\r\n");
    ManageRequest();
}
/** Linux Signals Handler
 */
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
    // *** This space must be completed  *****


    // Events creation               
    // ------------------------------------- 
    // *** This space must be completed  ******
    rt_event_create(&Experiment_Events,"Events",0,EV_FIFO);
    
    // Message queues creation               
    // ------------------------------------- 
    // *** This space must be completed  ******
    rt_queue_create(&SensorsMeasurement_Queue, "Measure", 50 * sizeof(SampleType), 50, Q_FIFO);

    // Semaphores creation                 
    // ------------------------------------
    rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
    // **** This space must be completed  ***** 
    rt_sem_create(&StartExperiment_Semaphore, "Start", 0, S_PULSE);

    // Mutual exclusion semaphore creation                     
    //---------------------------------------------------------
    // rt_mutex_create(&myMutex, "mx" );                       
    //    **** This space must be completed  ***** 

    // Tasks creation                                          
    //---------------------------------------------------------
    //   **** This space must be completed  ***** 
    rt_task_create(&TimeManagement_TaskDescriptor, "TimeManagementTask", DEFAULTSTACKSIZE, TIME_MANAGEMENT_PRIORITY, 0);
    rt_task_create(&LawsAndDisplayProcessing_TaskDescriptor, "LawsAndDisplayProcessingTask", DEFAULTSTACKSIZE, LAWS_AND_DISPLAY_PROCESSING_PRIORITY, 0);
    rt_task_create(&HumanMachineInterface_TaskDescriptor, "ManageRequestTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);

    // Tasks starting
    rt_task_start(&TimeManagement_TaskDescriptor, &TimeManagement_Task, NULL);
    rt_task_start(&LawsAndDisplayProcessing_TaskDescriptor, &LawsAndDisplayProcessing_Task, NULL);
    rt_task_start(&HumanMachineInterface_TaskDescriptor, &HumanMachineInterface_Task, NULL);

    //-----------------------------------------------------------
    // Main Task waits on exit semaphore                          
    //-----------------------------------------------------------
    rt_sem_p(&ExitApplication_Semaphore, TM_INFINITE);

    //-----------------------------------------------------------
    // Tasks destruction                                         
    //-----------------------------------------------------------
    // rt_task_ delete(&myTask);                                 
    rt_printf("\r\n \r\n Destruction of \r\n  - Tasks\r\n  - Queues\r\n  - Semaphores\r\n  - Events\r\n  - Mutexes\r\n");
    // **** This space must be completed *****  
    rt_task_delete(&HumanMachineInterface_TaskDescriptor);
    rt_task_delete(&TimeManagement_TaskDescriptor);
    rt_task_delete(&LawsAndDisplayProcessing_TaskDescriptor);

    //------------------------------------------------------------
    // Semaphores destruction                                     
    //-------------------------------------------------------------
    // rt_sem_ delete(&mySemaphore);                               
    //   **** This space must be completed  ***** 

    rt_sem_delete(&StartExperiment_Semaphore);
    rt_sem_delete(&ExitApplication_Semaphore);

    //------------------------------------------------------------
    // Events destruction                                 
    //------------------------------------------------------------
    // rt_event_delete(&myEvents);                                 
    // **** This space must be completed  *****
    rt_event_delete(&Experiment_Events);
    
    //------------------------------------------------------------
    // Message queues destruction                                 
    //------------------------------------------------------------
    // rt_queue_delete(&myQueue);                                 
    // **** This space must be completed  *****
    rt_queue_delete(&SensorsMeasurement_Queue);

    rt_printf(" Application ..... finished--> exit\r\n");
    // Peripherals uninitialization 
    HardwareTerminate();
    return (0);
}