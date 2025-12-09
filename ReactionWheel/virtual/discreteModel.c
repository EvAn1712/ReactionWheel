// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************
#include "discreteModel.h"

#define TASKNAME     "discreteModel"
#define ONE_SECOND   1e9     // Time unit for rt_functions is nanosecond
#define PERIOD       0.001   // Time unit is second

RT_TASK  discreteModel_TaskDescriptor;
RT_MUTEX discreteModel_Mutex;

void discreteModel_Task()
{
    rt_task_set_periodic(NULL, TM_NOW, PERIOD * ONE_SECOND);
    //--< infinite loop >------------------------------------------------------
    while(true) {

        rt_mutex_acquire(&discreteModel_Mutex, TM_INFINITE);

        dModel_step(); // Simulink generated function

        rt_mutex_release(&discreteModel_Mutex);

        // wait next period
        rt_task_wait_period(NULL);
    }
}

void discreteModel_Initialize()
{
    rt_mutex_create(&discreteModel_Mutex, TASKNAME "Mutex");
    rt_task_create(&discreteModel_TaskDescriptor, TASKNAME, 0, T_HIPRIO, 0);
    dModel_initialize(); // Simulink generated function
    rt_task_start(&discreteModel_TaskDescriptor, &discreteModel_Task, NULL);
    rt_printf("Hardware-> Simulated Wheel Task started\r\n");
}

void discreteModel_Reset()
{
    rt_mutex_acquire(&discreteModel_Mutex, TM_INFINITE);
    dModel_terminate();  // Simulink generated function
    dModel_initialize(); // Simulink generated function
    rt_mutex_release(&discreteModel_Mutex);
    rt_printf(" Hardware-> Simulated Wheel reseted\r\n");
}

void discreteModel_Terminate()
{
    dModel_terminate(); // Simulink generated function
    rt_task_delete(&discreteModel_TaskDescriptor);
    rt_mutex_delete(&discreteModel_Mutex);
    rt_printf("Hardware-> Simulated Wheel Task deleted\r\n");
}

