//
// Created by lak19 on 2025/7/4.
//

#include "control.h"

extern PID pid_l_speed, pid_r_speed; // 电机速度环PID结构体
PID pid_theta, pid_theta_dot;
float const g = 9.8f; // 重力加速度
float const lp = 0.05f; //重心到转轴距离，单位m
float const rw = 0.034f; //轮胎半径，单位m

uint32_t LastTime = 0; // 上次计算时间
float omega_ref = 0;
void Control_Init(void)
{
    PID_Init_General(&pid_theta);
    PID_Init_General(&pid_theta_dot);

    PID_Set_General(&pid_theta, 4, 0, 0); // 初始化PID参数
    PID_Set_General(&pid_theta_dot, 10, 10, 0);

    PID_LimConfig_General(&pid_theta, -12.57f, 12.57f); // 设置输出限幅
    PID_LimConfig_General(&pid_theta_dot, -12.7f, 12.7f);
}

void Control_Compute(void)
{
    float now = HAL_GetTick();
    float deltaT = (now - LastTime) * 1.0e-3f; // 计算时间增量，单位秒
    PID_ChangeSP_General(&pid_theta, 0); // 设置目标角度为0

    float theta = Deg_to_Rag (Mpu6050_Data.KalmanPitch); // 获取当前角度
    float theta_dot = Deg_to_Rag(Mpu6050_Data.Gyro_X); // 获取当前角速度

    float theta_dot_ref = PID_Compute_General(&pid_theta, theta);//计算反馈值
    PID_ChangeSP_General(&pid_theta_dot, theta_dot_ref);//改变反馈值
    float theta_dot_dot_ref = PID_Compute_General(&pid_theta_dot, theta_dot); // 计算角加速度反馈值
    float x_dot_dot_ref = (g * sinf(theta) - theta_dot_dot_ref * 0.05) / cosf(theta); // 计算小车的加速度

    omega_ref += 1.0f / rw * x_dot_dot_ref * 0.01; // 更新参考角速度
    PID_ChangeSP_General(&pid_l_speed, -omega_ref); // 更新左轮目标速度
    PID_ChangeSP_General(&pid_r_speed, -omega_ref);
    LastTime = now;
}