//
// Created by lak19 on 2025/6/25.
//
#ifndef _PID_H_
#define _PID_H_

#include "stm32f1xx.h"
#include "encoder.h"
#include <stdio.h>

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;  //误差与上一次误差
    float integral,maxIntegral; //积分值和最大积分值
    float output,LowOutputLim,HighOutputLim;
    float SP; //用户设定的值
    uint64_t t_k_1;
    float err_k_1;
    float err_int_k_1;
}PID;

//void PID_Init(void);//PID参数初始化
//float Speed_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
//float Location_PID_Realize(PID* pid,float target,float feedback);
void PID_Init_General(PID * pid);
void PID_Set_General(PID * pid, float KP, float KI, float KD);
void PID_ChangeSP_General(PID * pid, float SP);
void PID_Reset_General(PID * pid);
float PID_Compute_General(PID * pid, float FB);
void PID_LimConfig_General(PID * pid, float LowOutputLim, float HighOutputLim);
void Motor_PID_Compute(void);
void USART_Parse_Command(char* str, uint8_t motor_n);

#endif
