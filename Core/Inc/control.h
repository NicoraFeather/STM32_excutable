//
// Created by lak19 on 2025/7/4.
//

#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"
#include "mpu6050.h"

void Control_Init(void); // 控制器初始化
void Control_Compute(void); // 控制器计算
#endif //CONTROL_H
