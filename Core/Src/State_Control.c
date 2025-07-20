//
// Created by lak19 on 2025/7/19.
//

#include "../Inc/State_Control.h"

extern unsigned char Digtal;

typedef enum
{
    FAR,
    MID,
} FLAG_FAR_OR_MID; //用于标记距离状态

typedef enum
{
    LEFT,
    RIGHT,
} MID_LEFT_OR_RIGHT;

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


uint32_t counter_10ms = 0; // 10ms计时器

const uint32_t TURN_TIME = 150; //转向的时间，12/6速
const uint32_t END_TIME = 55; //从节点到终点的时间,12速
const uint32_t END_BACK_TIME = 60; //从终点回到节点的时间，12速
const uint32_t TURNOVER_TIME = 160; //翻转时间，6速
const uint32_t FORWARD_1_TIME = 265; //走到一号节点的时间，12速
const uint32_t FORWARD_1_BACK_TIME = 150; //从一号节点返回的时间，12速
const uint32_t FORWARD_2_TIME = 572; //走到二号节点的时间，12速
const uint32_t FORWARD_2_BACK_TIME = 472; //从二号节点返回的时间，12速


State_Task current_task_state = STATE_CHECK_MEDICINE; //当前任务状态
FLAG_FAR_OR_MID Flag_FAR_OR_MID = MID; //距离状态标志
MID_LEFT_OR_RIGHT Flag_MID_LEFT_OR_RIGHT = RIGHT; //中间状态标志
// 每10ms调用此函数
void Control_goto_2(void)
{
    static State_Motor current_motor_state = STATE_START;
    switch (current_motor_state)
    {
        case STATE_START: //初始状态
            if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_SET) //检测到OFF后开始
            {
                current_motor_state = STATE_FORWARD_1;
            }
            break;

        case STATE_FORWARD_1: //前进到节点1
            Gray_control();
            if (++counter_10ms >= FORWARD_1_TIME || Digtal == 0b00000000)
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
            Gray_control();
            if (++counter_10ms >= END_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_WAIT;
            }
            break;

        case STATE_WAIT:
            Car_Stop();
            if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_RESET) //检测到药品被取下（即ON）
            {
                current_motor_state = STATE_TURNOVER;
            }
            break;

        case STATE_TURNOVER:
            Self_Right();
            if (++counter_10ms >= TURNOVER_TIME - 10)
            {
                counter_10ms = 0;
                current_motor_state = STATE_END_BACK;
            }
            break;

        case STATE_END_BACK: //回到末端
            Gray_control();
            if (++counter_10ms >= END_BACK_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURN_LEFT_A_BACK;
            }
            break;

        case STATE_TURN_LEFT_A_BACK:
            Turn_Left();
            if (++counter_10ms >= TURN_TIME + 10)
            {
                counter_10ms = 0;
                current_motor_state = STATE_FORWARD_1_BACK;
            }
            break;

        case STATE_FORWARD_1_BACK: //回到节点1
            Gray_control();
            if (++counter_10ms >= FORWARD_1_BACK_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_STOP;
            }
            break;

        case STATE_STOP: // 停止状态
            Car_Stop();
        default:
            break;
    }
}

void Control_goto_1(void)
{
    static State_Motor current_motor_state = STATE_START;
    switch (current_motor_state)
    {
        case STATE_START: //初始状态
            if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_SET) //检测到OFF后开始
            {
                current_motor_state = STATE_FORWARD_1;
            }
            break;

        case STATE_FORWARD_1: //前进到节点1
            Gray_control();
            if (++counter_10ms >= FORWARD_1_TIME || Digtal == 0b00000000)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURN_LEFT_A;
            }
            break;

        case STATE_TURN_LEFT_A: //第一次左转90度
            Turn_Left();
            if (++counter_10ms >= TURN_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_END;
            }
            break;

        case STATE_END: //走向末端
            Gray_control();
            if (++counter_10ms >= END_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_WAIT;
            }
            break;

        case STATE_WAIT:
            Car_Stop();
            if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_RESET) //检测到药品被取下（即ON）
            {
                current_motor_state = STATE_TURNOVER;
            }
            break;

        case STATE_TURNOVER:
            Self_Left();
            if (++counter_10ms >= TURNOVER_TIME - 10)
            {
                counter_10ms = 0;
                current_motor_state = STATE_END_BACK;
            }
            break;

        case STATE_END_BACK: //回到末端
            Gray_control();
            if (++counter_10ms >= END_BACK_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_TURN_RIGHT_A_BACK;
            }
            break;

        case STATE_TURN_RIGHT_A_BACK:
            Turn_Right();
            if (++counter_10ms >= TURN_TIME + 10)
            {
                counter_10ms = 0;
                current_motor_state = STATE_FORWARD_1_BACK;
            }
            break;

        case STATE_FORWARD_1_BACK: //回到节点1
            Gray_control();
            if (++counter_10ms >= FORWARD_1_BACK_TIME)
            {
                counter_10ms = 0;
                current_motor_state = STATE_STOP;
            }
            break;

        case STATE_STOP: // 停止状态
            Car_Stop();
        default:
            break;
    }
}

void Control_goto_345678(void)
{
    static State_Motor current_motor_state = STATE_START;
    switch (current_motor_state)
    {
        case STATE_START: //初始状态
            if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_SET) //检测到OFF后开始
            {
                current_motor_state = STATE_FORWARD_2;
            }
            break;
        case STATE_FORWARD_2:
            Gray_control();
            if (++counter_10ms >= FORWARD_2_TIME || Digtal == 0b00000000)
            {
                counter_10ms = 0;
                current_motor_state = STATE_IF_FAR;
            }
            break;
        case STATE_IF_FAR:
            if (Flag_FAR_OR_MID == MID)
            {
                Control_goto_34();
            } else
            {
                //Control_goto_3456();
            }
    }
}

void Control_goto_34(void)
{
    if (Flag_MID_LEFT_OR_RIGHT == LEFT) //左侧
    {
        static State_Motor current_motor_state = STATE_TURN_LEFT_A;
        switch (current_motor_state)
        {
            case STATE_TURN_LEFT_A: //第一次左转90度
                Turn_Left();
                if (++counter_10ms >= TURN_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_END;
                }
                break;

            case STATE_END: //走向末端
                Gray_control();
                if (++counter_10ms >= END_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_WAIT;
                }
                break;

            case STATE_WAIT:
                Car_Stop();
                if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_RESET) //检测到药品被取下（即ON）
                {
                    current_motor_state = STATE_TURNOVER;
                }
                break;

            case STATE_TURNOVER:
                Self_Left();
                if (++counter_10ms >= TURNOVER_TIME - 10)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_END_BACK;
                }
                break;

            case STATE_END_BACK: //回到末端
                Gray_control();
                if (++counter_10ms >= END_BACK_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_TURN_RIGHT_A_BACK;
                }
                break;

            case STATE_TURN_RIGHT_A_BACK:
                Turn_Right();
                if (++counter_10ms >= TURN_TIME + 10)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_FORWARD_2_BACK;
                }
                break;

            case STATE_FORWARD_2_BACK: //回到节点3
                Gray_control();
                if (++counter_10ms >= FORWARD_2_BACK_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_STOP;
                }
                break;
            case STATE_STOP:
                Car_Stop();
            default:
                break;
        }
    }
    else
    {
        static State_Motor current_motor_state = STATE_TURN_RIGHT_A;
        switch (current_motor_state)
        {
            case STATE_TURN_RIGHT_A: //第一次左转90度
                Turn_Right();
                if (++counter_10ms >= TURN_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_END;
                }
                break;

            case STATE_END: //走向末端
                Gray_control();
                if (++counter_10ms >= END_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_WAIT;
                }
                break;

            case STATE_WAIT:
                Car_Stop();
                if (HAL_GPIO_ReadPin(EN_KEY_GPIO_Port, EN_KEY_Pin) == GPIO_PIN_RESET) //检测到药品被取下（即ON）
                {
                    current_motor_state = STATE_TURNOVER;
                }
                break;

            case STATE_TURNOVER:
                Self_Right();
                if (++counter_10ms >= TURNOVER_TIME - 10)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_END_BACK;
                }
                break;

            case STATE_END_BACK: //回到末端
                Gray_control();
                if (++counter_10ms >= END_BACK_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_TURN_LEFT_A_BACK;
                }
                break;

            case STATE_TURN_LEFT_A_BACK:
                Turn_Left();
                if (++counter_10ms >= TURN_TIME + 10)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_FORWARD_2_BACK;
                }
                break;

            case STATE_FORWARD_2_BACK: //回到节点3
                Gray_control();
                if (++counter_10ms >= FORWARD_2_BACK_TIME)
                {
                    counter_10ms = 0;
                    current_motor_state = STATE_STOP;
                }
                break;
            case STATE_STOP:
                Car_Stop();
            default:
                break;
        }
    }
}
