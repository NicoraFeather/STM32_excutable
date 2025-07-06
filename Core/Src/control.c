//
// Created by lak19 on 2025/7/4.
//

#include "control.h"

extern PID pid_l_speed, pid_r_speed; // 电机速度环PID结构体
extern Motor motor1, motor2; // 电机结构体

PID pid_theta, pid_theta_dot, pid_velocity, pid_turn; // 角度环PID结构体
float const g = 9.8f; // 重力加速度
float const lp = 0.05f; //重心到转轴距离，单位m
float const rw = 0.034f; //轮胎半径，单位m
uint32_t LastTime = 0; // 上次计算时间
float omega_ref = 0; //角速度设定值
/**
 * 控制初始化，设定各环PID初始化，设定初速度为0
 */
void Control_Init(void)
{
    PID_Init_General(&pid_velocity);
    PID_Init_General(&pid_theta);
    PID_Init_General(&pid_theta_dot);
    PID_Init_General(&pid_turn);

    PID_Set_General(&pid_velocity,10,1,0); // 初始化线速度PID参数
    PID_Set_General(&pid_theta, 6, 0, 0); // 初始化俯仰角PID参数
    PID_Set_General(&pid_theta_dot, 15, 10, 0);// 初始化俯仰角速度PID参数
    PID_Set_General(&pid_turn, 0, 0, 0); // 初始化//转向PID参数

    PID_LimConfig_General(&pid_theta, -12.57f, 12.57f); // 设置输出限幅
    PID_LimConfig_General(&pid_theta_dot, -120.7f, 120.7f);

    PID_ChangeSP_General(&pid_velocity, 0); // 设置目标速度为0
}

/**
 * 平衡控制计算函数，proc类型
 */
void Control_Compute(void)
{
    float now = Get_us64(); //获取一个时间
    float deltaT = (now - LastTime) * 1.0e-6f; // 计算时间增量，单位秒

    float theta = Deg_to_Rag (Mpu6050_Data.KalmanPitch); // 获取当前角度
    float theta_dot = Deg_to_Rag(Mpu6050_Data.Gyro_X); // 获取当前角速度

    float omega=0.5*(motor1.speed + motor2.speed); // 计算当前角速度
    float omega2=-theta_dot*(lp+rw)/rw; //计算速度悖论补偿速度
    float omega1=-omega2+omega;//计算速度悖论速度
    float x_dot = omega1*rw;//计算反馈线速度

    PID_Compute_General(&pid_velocity, x_dot); //计算速度PID
    float theta_ref = atan(x_dot/g);//非线性成分补偿
    PID_ChangeSP_General(&pid_theta, theta_ref); // 设置目标角度为0

    float theta_dot_ref = PID_Compute_General(&pid_theta, theta);//计算反馈值
    PID_ChangeSP_General(&pid_theta_dot, theta_dot_ref);//改变反馈值
    float theta_dot_dot_ref = PID_Compute_General(&pid_theta_dot, theta_dot); // 计算角加速度反馈值
    float x_dot_dot_ref = (g * sinf(theta) - theta_dot_dot_ref * 0.05) / cosf(theta); // 计算小车的加速度

    omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT; // 更新参考角速度

    float omega_diff = PID_Compute_General(&pid_turn,Deg_to_Rag(Mpu6050_Data.Gyro_Z));// 计算转向PID输出

    PID_ChangeSP_General(&pid_l_speed, (-omega_ref+omega_diff)); // 更新左轮目标速度
    PID_ChangeSP_General(&pid_r_speed, (-omega_ref)-omega_diff); // 更新右轮目标速度
    LastTime = now;// 更新上次计算时间
}

/**
 * 设定前进速度
 * @param Speed 设定速度，单位m/s
 */
void Control_Speed(float Speed)
{
    PID_ChangeSP_General(&pid_velocity, Speed);
}

/**
 * 调整转向角度
 * @param Turn 转向角度，单位弧度
 */
void Control_Turn(float Turn)
{
    PID_ChangeSP_General(&pid_turn, Turn);
}