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
RT_TASK TimeManagement_TaskDescriptor;
RT_TASK ManageLawAndCurves_TaskDescriptor;

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
#define EVENT_INITIALIZE    BIT(0)
#define EVENT_COMPUTELAW    BIT(1)
#define EVENT_READSENSORS   BIT(2)
#define EVENT_FINISH        BIT(3)
#define EVENT_ALL (EVENT_INITIALIZE | EVENT_COMPUTELAW | EVENT_READSENSORS | EVENT_FINISH)

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
volatile bool ExperimentFinished_Flag = false;
volatile bool AbortExperiment_Flag = false;
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
    int experimentTime;
    rt_task_set_periodic(NULL, TM_NOW, TIME_MANAGEMENT_PERIOD_NS);
    
    while(true) {
        rt_sem_p(&StartExperiment_Semaphore, TM_INFINITE);
        
        rt_event_signal(&ExperimentControl_Event, EVENT_INITIALIZE);
        experimentTime = 0;
        AbortExperiment_Flag = false;
        ExperimentFinished_Flag = false;
        
        while((experimentTime < ExperimentParameters.duration) && 
              (AbortExperiment_Flag == false) && 
              (ConnectionIsActive() == true)) {
            rt_task_wait_period(NULL);
            experimentTime++;
            
            if((experimentTime % ExperimentParameters.lawPeriod) == 0) {
                rt_event_signal(&ExperimentControl_Event, EVENT_COMPUTELAW);
            }
            if((experimentTime % ExperimentParameters.acquisitionPeriod) == 0) {
                rt_event_signal(&ExperimentControl_Event, EVENT_READSENSORS);
            }
        }
        
        ExperimentFinished_Flag = true;
        rt_event_signal(&ExperimentControl_Event, EVENT_FINISH);
        
        if(AbortExperiment_Flag == true) rt_printf(" Experiment aborted\r\n");
        if(experimentTime >= ExperimentParameters.duration) rt_printf(" End of experiment\r\n");
    }
}

void ManageLawAndCurves_Task()
{
    rt_printf("Starting Manage Law Acquire task\r\n");
    unsigned long inputEvents;
    SampleType sample;
    float wheelCommand;

    while(true) {
        rt_event_wait(&ExperimentControl_Event, EVENT_ALL, &inputEvents, EV_ANY, TM_INFINITE);
        
        SampleAcquisition(&sample);
        
        if(inputEvents & EVENT_INITIALIZE) {
            InitializeExperiment(sample);
            rt_event_clear(&ExperimentControl_Event, EVENT_INITIALIZE, NULL);
        }
        
        if(inputEvents & EVENT_COMPUTELAW) {
            wheelCommand = ComputeLaw(sample);
            ApplySetpointCurrent(wheelCommand);
            rt_event_clear(&ExperimentControl_Event, EVENT_COMPUTELAW, NULL);
        }
        
        if(inputEvents & EVENT_READSENSORS) {
            rt_queue_write(&SensorData_Queue, &sample, sizeof(sample), Q_NORMAL);
            rt_event_clear(&ExperimentControl_Event, EVENT_READSENSORS, NULL);
        }
        
        if(inputEvents & EVENT_FINISH) {
            ApplySetpointCurrent(0.0);
            rt_event_clear(&ExperimentControl_Event, EVENT_FINISH, NULL);
        }
    }
}


//-----------------------------------------------------------
// functions called from HMI_Utilities

// This function is called when the user press on Abort during an experiment \n
// called from function "manageRequest" in file WheelHMI.c  
void AbortExperiment(void)
{
    AbortExperiment_Flag = true;
}

void StartExperiment(void)
{
    rt_queue_flush(&SensorData_Queue);
    rt_sem_v(&StartExperiment_Semaphore);
}

//----------------------------------------------------------

void ReturnSensorsMeasurement()
{
    float samplesBlock[50 * 4];
    SampleType sample;
    RT_QUEUE_INFO queueInfo;
    char terminationChar;

    rt_queue_inquire(&SensorData_Queue, &queueInfo);
    int i;
    for(i = 0; i < queueInfo.nmessages; i++) {
        rt_queue_read(&SensorData_Queue, &sample, sizeof(sample), TM_NONBLOCK);
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
    // *** This space must be completed  if needed   *****

    // ------------------------------------- 
    // Events creation               
    rt_event_create(&ExperimentControl_Event, "ExperimentControl", 0, EV_FIFO);
    
    // Message queues creation               
    // ------------------------------------- 
    rt_queue_create(&SensorData_Queue, "SensorQueue",
                    SENSOR_QUEUE_SIZE * sizeof(SampleType), SENSOR_QUEUE_SIZE, Q_FIFO);

    // Semaphores creation                 
    // ------------------------------------
    rt_sem_create(&ExitApplication_Semaphore, "Exit", 0, S_FIFO);
    rt_sem_create(&StartExperiment_Semaphore, "StartExp", 0, S_PULSE);

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
