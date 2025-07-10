//
// Created by lak19 on 2025/7/10.
//

#include "motor_control.h"

#include "mpu6050.h"

extern PID pid_l_speed, pid_r_speed; //速度环PID结构体
extern PID pid_turn; //转向PID结构体
extern Motor motor1, motor2; //电机结构体

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

void Motor_Control_Turn(float theta) {
    PID_ChangeSP_General(&pid_turn, theta); // 更新转向PID目标值
    float omega_diff = PID_Compute_General(&pid_turn,Deg_to_Rag(Mpu6050_Data.Gyro_Z));// 计算转向PID输出

    PID_ChangeSP_General(&pid_l_speed, motor1.speed+omega_diff); // 更新左轮目标速度
    PID_ChangeSP_General(&pid_r_speed, motor2.speed-omega_diff); // 更新右轮目标速度
}
