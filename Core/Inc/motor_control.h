//
// Created by lak19 on 2025/7/10.
//

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "encoder.h"
#include "pid.h"
void Motor_Control_Stop();
void Motor_Control_Go(float speed); // 设置电机前进速度
void Go_Ahead();
void Gray_control(void); // 灰度传感器控制函数
#endif //MOTOR_CONTROL_H
