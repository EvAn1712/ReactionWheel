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
RT_MUTEX DataMutex;
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

    struct timespec start, now;

    while(1) {
        // Attendre le démarrage d'une expérience
        rt_sem_p(&StartExperiment_Semaphore, TM_INFINITE);

        clock_gettime(CLOCK_MONOTONIC, &start);
        experimentRunning = true;
        ExperimentElapsedMs = 0;

        while(experimentRunning) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            ExperimentElapsedMs = (unsigned long)(now.tv_sec - start.tv_sec) * 1000UL
                                + (now.tv_nsec - start.tv_nsec) / 1000000UL;

            // Vérifier si durée max atteinte
            if(ExperimentElapsedMs >= ExperimentParameters.experimentDuration * 1000) {
                experimentRunning = false;
                rt_sem_v(&ExitApplication_Semaphore);
            }

            rt_task_sleep(1000000); // 1ms
        }
    }
}

void ManageLawAcquire_Task()
{
    rt_printf("Starting Manage Law Acquire task\r\n");

    SampleType sample;

    while(1) {
        if(experimentRunning) {
            SampleAcquisition(&sample);

            rt_printf("[DATA] M:%.2f P:%.2f Pos:%.2f I:%.2f\r\n",
                      sample.motorSpeed,
                      sample.platformSpeed,
                      sample.platformPosition,
                      sample.motorCurrent);

            rt_task_sleep(100000000); // 100ms
        } else {
            rt_task_sleep(500000000); // 500ms en attente
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
    experimentRunning = false;
    SetMotorCommand(0.0f); // Arrêter le moteur
    float emptyBlock[1] = {0.0f};
    WriteRealArray('F', emptyBlock, 1);
}

void StartExperiment(void)
{
    rt_printf("Started\r\n");
    rt_sem_v(&StartExperiment_Semaphore); // Débloquer la tâche TimeManagement
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
    // rt_event_delete(...);                                 
    // **** This space must be completed  if needed   *****
   
    
    //------------------------------------------------------------
    // Message queues destruction                                 
    //------------------------------------------------------------
    // rt_queue_delete(...);                                 
    // **** This space must be completed  if needed   *****
    rt_mutex_delete(&DataMutex);

    rt_printf(" Application ..... finished--> exit\r\n");
    // Peripherals uninitialization 
    HardwareTerminate();
    return (0);
}
