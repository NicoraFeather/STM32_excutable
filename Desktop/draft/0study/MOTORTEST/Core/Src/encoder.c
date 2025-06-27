#include "encoder.h"

Motor motor1; //定义一个左轮的结构体
Motor motor2; //定义一个右轮的结构体

/**
 * @brief 所有电机初始化，开启PWM，开启间隔中断，开启编码器，初始为刹车状态
 */
void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER1_TIM, TIM_CHANNEL_ALL);      //开启编码器定时器
    HAL_TIM_Encoder_Start(&ENCODER2_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 7200-1);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //开启所有的PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_4);
    __HAL_TIM_SET_COUNTER(&ENCODER1_TIM, 0);                //编码器定时器初始值设定为30000
    __HAL_TIM_SET_COUNTER(&ENCODER2_TIM, 0);
    motor1.lastCount = 0;                                   //结构体内容初始化
    motor1.totalCount = 0;
    motor1.overflowNum = 0;
    motor1.speed = 0;
    motor1.direct = 0;

    motor2.lastCount = 0;                                   //结构体内容初始化
    motor2.totalCount = 0;
    motor2.overflowNum = 0;
    motor2.speed = 0;
    motor2.direct = 0;
}

/**
 * @brief 获取当前速度
 * @param htim 判断当前中断是不是间隔定时器
 */
void Motor_Get_Speed(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==GAP_TIM.Instance)
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER1_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM1 + motor1.overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值

        if(motor1.lastCount - motor1.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum++;
            motor1.totalCount = COUNTERNUM1 + motor1.overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor1.totalCount - motor1.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum--;
            motor1.totalCount = COUNTERNUM1 + motor1.overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PER_ROUND) * 100;//单位：°每秒
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
    }
}

//
// Created by lak19 on 2025/6/25.
//
