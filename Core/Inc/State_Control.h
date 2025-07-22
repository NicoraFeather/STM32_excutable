//
// Created by lak19 on 2025/7/19.
//

#ifndef STATE_CONTROL_H
#define STATE_CONTROL_H

#include "motor_control.h"
#include "Grayscale_Sensor.h"

typedef enum
{
    MID,
    FAR,
} FLAG_FAR_OR_MID; //用于标记在节点2时的距离状态

typedef enum
{
    LEFT,
    RIGHT,
} MID_LEFT_OR_RIGHT; //用于标记在中端时左右转向

typedef enum
{
    STATE_CHECK_MEDICINE,
    STATE_CHECK_NUM,
    STATE_GOTO_12,
    STATE_GOTO_345678,
    STATE_WAIT_MEDICINE,
    STATE_BACK,
} State_Task;

typedef enum
{
    MOVE_TO_1 = 1,
    MOVE_TO_2,
    MOVE_TO_3,
    MOVE_TO_4,
    MOVE_TO_5,
    MOVE_TO_6,
    MOVE_TO_7,
    MOVE_TO_8,
} _Move_Flag;

// 定义运动控制状态枚举
typedef enum
{
    STATE_START,
    STATE_FORWARD_1,
    STATE_FORWARD_2,
    STATE_FORWARD_3,
    STATE_TURN_LEFT_A,
    STATE_TURN_RIGHT_A,
    STATE_TURN_LEFT_B,
    STATE_TURN_RIGHT_B,
    STATE_TURNOVER,
    STATE_END,
    STATE_STOP,
    STATE_WAIT,
    STATE_IF_FAR,

    STATE_FORWARD_1_BACK,
    STATE_FORWARD_2_BACK,
    STATE_FORWARD_3_BACK,
    STATE_TURN_LEFT_A_BACK,
    STATE_TURN_RIGHT_A_BACK,
    STATE_TURN_LEFT_B_BACK,
    STATE_TURN_RIGHT_B_BACK,
    STATE_END_BACK,
} State_Motor;



void Control_goto_1(void);
void Control_goto_2(void);
void Control_goto_345678(void);
void Control_goto_34(void);



#endif //STATE_CONTROL_H
