//
// File:   Motor.h
// Author: Laurent Alloza <laurent.alloza@isae-supaero.fr>
//

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
    
void Motor_initialize(void);
void Motor_duty(double percent);
void Motor_terminate(void);

#ifdef __cplusplus
}
#endif

#endif

