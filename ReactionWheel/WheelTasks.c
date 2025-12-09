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

//#define BIT(n) (1<<n)
//#define EVENT_1    BIT(0)
//#define EVENT_2    BIT(1)
//#define EVENT_ALL (EVENT_1 | EVENT_2)

//********   This space must be completed if needed *****  

//--------------------------------------------
// Mutual exclusion semaphores descriptors    
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__mutex.html

//********   This space must be completed if needed *****  

//--------------------------------------------
// Message queues descriptors                 
//--------------------------------------------
// https://gitlab-pages.isae-supaero.fr/l.alloza/doc-xenomai3/group__alchemy__queue.html

//********   This space must be completed if needed *****  


//--------------------------------------------------------------------------
// Global variables communication and synchronization tasks by shared memory 
//--------------------------------------------------------------------------
// bool myFlag
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
    
}

void ManageLawAcquire_Task()
{
    
}

//-----------------------------------------------------------
// functions called from HMI_Utilities

// This function is called when the user press on Abort during an experiment \n
// called from function "manageRequest" in file WheelHMI.c  
void AbortExperiment(void)
{
   rt_printf("Aborted\r\n");
   rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
   float emptyBlock[1] = {0.0f};
   WriteRealArray('F', emptyBlock, 1);
}
//-----------------------------------------------------------

void StartExperiment(void)
{
   rt_printf("Started\r\n");
   rt_sem_create(&StartExperiment_Semaphore, "Start", 0, S_FIFO);
}
//----------------------------------------------------------

void TimeManagement()
{

    static volatile unsigned long ExperimentElapsedMs = 0;
    static struct timespec start = {0,0};
    struct timespec now;
    
    if(ExperimentElapsedMs){
        if(start.tv_sec == 0 && start.tv_nsec == 0){
            clock_gettime(CLOCK_MONOTONIC, &start);
            ExperimentElapsedMs= 0;
            
        }else {
            clock_gettime(CLOCK_MONOTONIC, &now);
            ExperimentElapsedMs = (unsigned long)(now.tv_sec - start.tv_sec) * 1000UL + (now.tv_nsec- start.tv_nsec)/1000000UL;
        }  
     } else {
        start.tv_sec = start.tv_nsec = 0;
        ExperimentElapsedMs = 0;
    }
    (void)ExperimentElapsedMs;
}



void ReturnSensorsMeasurement()
{
    SampleType sample;
    
    float samplesBlock[50 * 4];
    char terminationChar;
    int i;
    float AP = ExperimentParameters.acquisitionPeriod ;
    float LP = ExperimentParameters.lawPeriod ;
    int n = (LP/AP); 
    
    for(i = 0; i < n; i++) {
        SampleAcquisition(&sample);

        samplesBlock[(i * 4)]     = sample.motorSpeed;
        samplesBlock[(i * 4) + 1] = sample.platformSpeed;
        samplesBlock[(i * 4) + 2] = sample.platformPosition;
        samplesBlock[(i * 4) + 3] = sample.motorCurrent;
        
    }
   
    terminationChar = 'S';  
    
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
    // rt_event_create(...);
    // *** This space must be completed  if needed   *****
    
    // Message queues creation               
    // ------------------------------------- 
    // *** This space must be completed  if needed   ******
    // rt_queue_create(...);

    // Semaphores creation                 
    // ------------------------------------
    rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
    rt_sem_create(&StartExperiment_Semaphore, "StartExp", 0, S_FIFO);


    // **** This space must be completed  if needed   ***** 
    

    // Mutual exclusion semaphore creation                     
    //---------------------------------------------------------
    // rt_mutex_create(...);                       
    // **** This space must be completed  if needed   ***** 

    // Tasks creation                                          
    //---------------------------------------------------------
    rt_task_create(&HumanMachineInterface_TaskDescriptor, "ReturnSensorsMeasurementestTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);
    rt_task_create(&TimeManagement_TaskDescriptor, "TimeManagementTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);
    rt_task_create(&ManageLawAcquire_TaskDescriptor, "ManageLawAcquireTask", DEFAULTSTACKSIZE, HUMAN_MACHINE_INTERFACE_PRIORITY, 0);



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
    // rt_event_delete(...);                                 
    // **** This space must be completed  if needed   *****
   
    
    //------------------------------------------------------------
    // Message queues destruction                                 
    //------------------------------------------------------------
    // rt_queue_delete(...);                                 
    // **** This space must be completed  if needed   *****
    

    rt_printf(" Application ..... finished--> exit\r\n");
    // Peripherals uninitialization 
    HardwareTerminate();
    return (0);
}
