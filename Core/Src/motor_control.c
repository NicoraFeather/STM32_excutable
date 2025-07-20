//
// Created by lak19 on 2025/7/10.
//

#include "../Inc/motor_control.h"

#include <stdbool.h>

#include "mpu6050.h"

extern PID pid_l_speed, pid_r_speed; //速度环PID结构体
extern PID pid_turn; //转向PID结构体
extern Motor motor1, motor2; //电机结构体
extern unsigned char Digtal;

/**
 * 将所有电机的PWM设置为停止状态
 */
void Motor_Control_Stop()
{
    PID_ChangeSP_General(&pid_l_speed, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 7200-1);
    __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 7200-1);
}

void Motor_Control_Go(float speed)
{
    PID_ChangeSP_General(&pid_l_speed, speed);
    PID_ChangeSP_General(&pid_r_speed, speed);
}

void Go_Ahead()
{
    PID_ChangeSP_General(&pid_l_speed, 12);
    PID_ChangeSP_General(&pid_r_speed, 12);
}

void Turn_Right()
{
    PID_ChangeSP_General(&pid_l_speed, 12);
    PID_ChangeSP_General(&pid_r_speed, 6);
}

void Turn_Left()
{
    PID_ChangeSP_General(&pid_l_speed, 6);
    PID_ChangeSP_General(&pid_r_speed, 12);
}

void Turn_Right_Adj()
{
    PID_ChangeSP_General(&pid_l_speed, 12);
    PID_ChangeSP_General(&pid_r_speed, 10);
}

void Turn_Left_Adj()
{
    PID_ChangeSP_General(&pid_l_speed, 10);
    PID_ChangeSP_General(&pid_r_speed, 12);
}


void Self_Left()
{
    PID_ChangeSP_General(&pid_l_speed, -6);
    PID_ChangeSP_General(&pid_r_speed, 6);
}

void Self_Right()
{
    PID_ChangeSP_General(&pid_l_speed, 6);
    PID_ChangeSP_General(&pid_r_speed, -6);
}

void Car_Stop()
{
    PID_ChangeSP_General(&pid_l_speed, 0);
    PID_ChangeSP_General(&pid_r_speed, 0);
}

void Gray_control(void)
{
    switch (Digtal)
    {
        // 全黑：停止
        case 0b00000000:
            Car_Stop();
            break;

        // 理想前进：中间两路都检测到黑线
        case 0b11100111: // 0b00011000 取反：黑=0，白=1
        case 0b11101111: // 容忍轻微偏移
        case 0b11110111:
            Go_Ahead();
            break;

        // 轻微偏左：中间偏左黑
        case 0b11110011:
        case 0b11111011:
            Turn_Left_Adj(); // 微调右转
            break;

        // 轻微偏右：中间偏右黑
        case 0b11011111:
        case 0b11001111:
            Turn_Right_Adj(); // 微调左转
            break;

        // 明显偏左：左侧多路黑
        case 0b11111001:
        case 0b11111101:
        case 0b11111100:
        case 0b11111110:
            Turn_Left(); // 明显右转
            break;

        // 明显偏右：右侧多路黑
        case 0b10111111:
        case 0b10011111:
        case 0b01111111:
        case 0b00111111:
            Turn_Right(); // 明显左转
            break;

        // 全白：无黑线（脱线）
        case 0b11111111:
            Go_Ahead(); // 或执行原地旋转找线
            break;

        default:
            Go_Ahead();
            break;
    }


    // uint8_t masked = Digtal | 0x18;  // 0x18 = 0b00011000
    //
    // // 全黑检测（包括中间两路）
    // if (Digtal == 0x00) {  // 0b00000000
    //     Car_Stop();
    //     return;
    // }
    //
    // // 所有有效传感器（中间两路除外）检测到白色
    // if (masked == 0xFF) {  // 0b11111111
    //     Go_Ahead();
    //     return;
    // }
    //
    // // 提取左右两侧传感器状态
    // uint8_t left_sensors = (masked >> 5) & 0x07;  // 左侧3个传感器（第5-7位）
    // uint8_t right_sensors = masked & 0x07;         // 右侧3个传感器（第0-2位）
    //
    // // 判断逻辑
    // if (right_sensors != 0x07 && left_sensors == 0x07) {
    //     // 右侧有黑线且左侧全白 - 右转
    //     Turn_Right();
    // } else if (left_sensors != 0x07 && right_sensors == 0x07) {
    //     // 左侧有黑线且右侧全白 - 左转
    //     Turn_Left();
    // } else if (right_sensors == 0x00 || left_sensors == 0x00) {
    //     // 单侧完全检测到黑线（急转弯情况）
    //     if (right_sensors == 0x00) Turn_Right();
    //     else Turn_Left();
    // } else {
    //     // 其他情况（如十字路口）保持直行
    //     Go_Ahead();
    // }
}
