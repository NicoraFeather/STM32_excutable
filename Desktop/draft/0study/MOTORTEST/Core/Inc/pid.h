//
// Created by lak19 on 2025/6/25.
//
#ifndef _PID_H_
#define _PID_H_

#include "stm32f1xx.h"
#include "encoder.h"
#include <stdio.h>

extern uint8_t DataBuff[200];//接受到的完整指令内容
extern float L_Target_Speed; //在main中设定的目标值

//PID三个参数的初始值，当前已经整定
#define KP_speed 5487.5
#define KI_speed 0.51
#define KD_speed 0

#define KP_position 0.13
#define KI_position 0
#define KD_position 0

#define ASCII_0 48   //0的ASCII码，因为发送的数据是HEX

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值和最大积分值
    float output,maxOutput;
}PID;

void PID_Init(void);//PID参数初始化
float Speed_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
void USART_PID_Adjust(uint8_t Motor_n);//PID参数赋值函数
float Get_Data(void);
float Location_PID_Realize(PID* pid,float target,float feedback);
#endif
