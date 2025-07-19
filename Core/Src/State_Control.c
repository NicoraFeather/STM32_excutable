//
// Created by lak19 on 2025/7/19.
//

#include "../Inc/State_Control.h"

typedef enum
{
    STATE_CHECK_MEDICINE,
    STATE_CHECK_NUM,
    STATE_GOTO_12,
    STATE_GOTO_34,
    STATE_GOTO_5678,
    STATE_WAIT_MEDICINE,
    STATE_BACK,
} State_Task;

typedef enum
{
    MOVE_TO_3,
    MOVE_TO_4,
    MOVE_TO_5,
    MOVE_TO_6,
    MOVE_TO_7,
    MOVE_TO_8,
} Move_Flag;

// 定义运动控制状态枚举
typedef enum
{
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

    STATE_FORWARD_1_BACK,
    STATE_FORWARD_2_BACK,
    STATE_FORWARD_3_BACK,
    STATE_TURN_LEFT_A_BACK,
    STATE_TURN_RIGHT_A_BACK,
    STATE_TURN_LEFT_B_BACK,
    STATE_TURN_RIGHT_B_BACK,
    STATE_END_BACK,
} State_Motor;


uint32_t counter_10ms = 0; // 10ms计时器

const uint32_t TURN_TIME = 150; //转向的时间，12/6速
const uint32_t END_TIME = 60; //从节点到终点的时间,12速
const uint32_t TURNOVER_TIME = 150; //翻转时间，6速
const uint32_t FORWARD_1_TIME = 260; //走到一号节点的时间，12速
const uint32_t FORWARD_2_TIME = 510; //走到二号节点的时间，12速

State_Motor current_motor_state = STATE_FORWARD_1;

// 每10ms调用此函数
void Control_goto_2(void)
{
    switch (current_motor_state)
    {
        case STATE_FORWARD_1: //前进到节点1
            Go_Ahead();
            if (++counter_10ms >= FORWARD_1_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURN_RIGHT_A;
            }
            break;

        case STATE_TURN_RIGHT_A: //第一次右转90度
            Turn_Right();
            if (++counter_10ms >= TURN_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_END;
            }
            break;

        case STATE_END: //走向末端
            Go_Ahead();
            if (++counter_10ms >= END_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURNOVER;
            }
            break;

        case STATE_TURNOVER:
            Self_Right();
            if (++counter_10ms >= TURNOVER_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_END_BACK;
            }
            break;

        case STATE_END_BACK: //回到末端
            Go_Ahead();
            if (++counter_10ms >= END_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURN_LEFT_A_BACK;
            }
            break;

        case STATE_TURN_LEFT_A_BACK:
            Turn_Left();
            if (++counter_10ms >= TURN_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_FORWARD_1_BACK;
            }

        case STATE_FORWARD_1_BACK: //回到节点1
            Go_Ahead();
            if (++counter_10ms >= FORWARD_1_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_STOP;
            }
            break;

        case STATE_STOP: // 停止状态
            Car_Stop();
            break;

        default:
            break;
    }
}


