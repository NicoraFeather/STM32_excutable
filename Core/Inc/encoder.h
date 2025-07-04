//
// Created by lak19 on 2025/6/25.
//
#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "pid.h"
#include "stm32f1xx.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include "pid.h"

//电机1的编码器输入引脚 PB6，PB7
#define MOTO1_ENCODER1_PORT GPIOB
#define MOTO1_ENCODER1_PIN  GPIO_PIN_6
#define MOTO1_ENCODER2_PORT GPIOB
#define MOTO1_ENCODER2_PIN  GPIO_PIN_7

//电机2的编码器输入引脚 PC6，PC7
#define MOTO2_ENCODER1_PORT GPIOC
#define MOTO2_ENCODER1_PIN  GPIO_PIN_6
#define MOTO2_ENCODER2_PORT GPIOC
#define MOTO2_ENCODER2_PIN  GPIO_PIN_7

//定时器以及通道编号
#define ENCODER1_TIM htim4                      //电机1编码器定时器
#define ENCODER2_TIM htim8                      //电机2编码器定时器
#define MOTOR1_TIM     htim3                    //电机1PWM产生
#define MOTOR2_TIM     htim3                    //电机2PWM产生
#define MOTOR1_CHANNEL_FORWARD TIM_CHANNEL_1    //电机1拉高向前通道
#define MOTOR1_CHANNEL_BACKWARD TIM_CHANNEL_2   //电机1拉高向后通道
#define MOTOR2_CHANNEL_FORWARD TIM_CHANNEL_4    //电机2拉高向前通道
#define MOTOR2_CHANNEL_BACKWARD TIM_CHANNEL_3   //电机2拉高向后通道
#define PWM_TIM     htim3                       //PWM产生定时器，和MOTOR1_TIM等效
#define GAP_TIM     htim6                       //间隔定时器
#define MOTOR_SPEED_RERATIO 30u                 //电机减速比
#define PULSE_PER_ROUND 500                     //编码器线数
#define RADIUS_OF_TYRE 34                       //轮胎半径，单位mm
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14  //轮胎周长，单位mm
#define RELOADVALUE __HAL_TIM_GET_AUTORELOAD(&ENCODER1_TIM)      //获取编码器自动装载值，1&2对称
#define COUNTERNUM1 __HAL_TIM_GetCounter(&ENCODER1_TIM)          //获取编码器定时器中的计数值
#define COUNTERNUM2 __HAL_TIM_GetCounter(&ENCODER2_TIM)


#include "pid.h"
//电机状态结构体
typedef struct _Motor
{
    int32_t lastCount;   //上次计数值
    int32_t totalCount;  //总计数值
    int32_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}Motor;

void Motor_Init(void); //电机初始化
void Motor_Get_Speed(Motor* motor); //测速函数
#endif