#include "encoder.h"

#include "functional.h"

Motor motor1; //定义一个左轮的结构体
Motor motor2; //定义一个右轮的结构体
extern PID pid_l_speed, pid_l_position, pid_r_speed, pid_r_position;
/**
 * @brief 所有电机初始化，开启PWM，开启编码器，初始为刹车状态
 */
void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER1_TIM, TIM_CHANNEL_ALL);      //开启编码器定时器
    HAL_TIM_Encoder_Start(&ENCODER2_TIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 7200-1);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //开启所有的PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_4);
    __HAL_TIM_SET_COUNTER(&ENCODER1_TIM, 10000);
    __HAL_TIM_SET_COUNTER(&ENCODER2_TIM, 10000);
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

    PID_Init_General(&pid_l_speed);
    PID_Init_General(&pid_r_speed);
    PID_Set_General(&pid_l_speed, 0, 0, 0);
    PID_Set_General(&pid_r_speed, 0, 0, 0);

}

/**
 * @brief 获取当前速度，单位 Rad/s
 * @param motor 电机的指针
 * @note 放在回调函数中
 */
void Motor_Get_Speed(Motor* motor)
{
    if (motor == &motor1) {
        motor->direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER1_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor->totalCount = COUNTERNUM1 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值

        if(motor->lastCount - motor->totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum++;
            motor->totalCount = COUNTERNUM1 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor->totalCount - motor->lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum--;
            motor->totalCount = COUNTERNUM1 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        motor->speed = (float)(motor->totalCount - motor->lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PER_ROUND) * 100 * 360;//单位：°每秒
        motor->speed = Deg_to_Rag(motor->speed);
        motor->lastCount = motor->totalCount; //记录这一次的计数值
    }

    else if (motor == &motor2)
    {
        motor->direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER2_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor->totalCount = COUNTERNUM2 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值

        if(motor->lastCount - motor->totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum++;
            motor->totalCount = COUNTERNUM2 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor->totalCount - motor->lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum--;
            motor->totalCount = COUNTERNUM2 + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        motor->speed = -(float)(motor->totalCount - motor->lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PER_ROUND) * 100 * 360;//单位：zhuan每秒
        motor->speed = Deg_to_Rag(motor->speed);
        motor->lastCount = motor->totalCount; //记录这一次的计数值
    }
}




//
// Created by lak19 on 2025/6/25.
//
