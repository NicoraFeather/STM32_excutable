//
// Created by lak19 on 2025/7/10.
//

#include "../Inc/motor_control.h"

#include "mpu6050.h"

extern PID pid_l_speed, pid_r_speed; //速度环PID结构体
extern PID pid_turn; //转向PID结构体
extern Motor motor1, motor2; //电机结构体
extern unsigned char Digtal;

/**
 * 将所有电机的PWM设置为停止状态
 */
void Motor_Control_Stop() {
    PID_ChangeSP_General(&pid_l_speed, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 7200-1);
}

void Motor_Control_Go(float speed) {
    PID_ChangeSP_General(&pid_l_speed, speed);
    PID_ChangeSP_General(&pid_r_speed, speed);
}

void Go_Ahead()
{
    PID_ChangeSP_General(&pid_l_speed,6);
    PID_ChangeSP_General(&pid_r_speed,6);
}
void Turn_Right()
{
    PID_ChangeSP_General(&pid_l_speed,-3);
    PID_ChangeSP_General(&pid_r_speed,6);
}
void Turn_Left()
{
    PID_ChangeSP_General(&pid_l_speed,6);
    PID_ChangeSP_General(&pid_r_speed,-3);
}

void Self_Left()
{
    PID_ChangeSP_General(&pid_l_speed,-6);
    PID_ChangeSP_General(&pid_r_speed,6);
}
void Self_Right()
{
    PID_ChangeSP_General(&pid_l_speed,6);
    PID_ChangeSP_General(&pid_r_speed,-6);
}
void Car_Stop()
{
    PID_ChangeSP_General(&pid_l_speed,0);
    PID_ChangeSP_General(&pid_l_speed,0);
}
void Car_SlowDown()
{
    PID_ChangeSP_General(&pid_l_speed,2);
    PID_ChangeSP_General(&pid_l_speed,2);
}
void Gray_control(void)
{
    switch (Digtal) {
        case 0b00000000: // 全黑
            Car_Stop();
            break;
        case 0b11111111: // 全白
            Car_SlowDown();
            break;
        case 0b11111110:
        case 0b11111100:
        case 0b11111000:
        case 0b11110001:
        case 0b11111001:
        case 0b11110011:
            Turn_Right();
            break;
        case 0b01111111: // 左边白右边黑
        case 0b00111111:
        case 0b00011111:
        case 0b10011111:
        case 0b10001111:
        case 0b11001111:
            Turn_Left();
            break;
        default:
            Go_Ahead(); // 默认前进
            break;
    }
}