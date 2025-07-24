//
// Created by lak19 on 2025/6/25.
//

#include "callback.h"

#include <stdbool.h>

#include "pid.h"
#include "balance_control.h"
#include "motor_control.h"
#include "Grayscale_Sensor.h"
#include "State_Control.h"
#include "uart_command.h"
#include "uart_com_cycle.h"
#include "vbat.h"
#include "wireless.h"

extern uint8_t DAP_Com_Buff[128]; //接收串口1的数据缓存数组
float motor_Out1 = 0;
float motor_Out2 = 0;
char message[100] = ""; //与vofa通信用数组
extern DMA_HandleTypeDef hdma_usart1_rx;
extern PID pid_l_speed, pid_l_position, pid_r_speed, pid_r_position;
extern Motor motor1;
extern Motor motor2;
extern uint8_t enable_flag; // 用于标记是否开启计算
extern float theta_ref;
extern float x_dot; //test
extern uint8_t BLE_Com[128];
extern uint32_t counter_10ms;
extern bool system_initialized;

/********************灰度传感器配置********************/
extern No_MCU_Sensor sensor; //无时基传感器结构体
extern unsigned char Digtal;
extern char Gray_rx_buff[256];
/********************串口配置********************/
extern uint8_t DAP_Com_Buff[128]; //串口接收数据缓存
extern uint8_t DAP_Com[128]; //提取的数据
extern uint8_t DAP_Com_Length; // 提取的数据长度
extern uint8_t DAP_Com_Decode[128]; // 解码之后的数据

 /*************在uart_command.c中定义*************/
extern FLAG_FAR_OR_MID Flag_FAR_OR_MID; //距离状态标志
extern MID_LEFT_OR_RIGHT Flag_MID_LEFT_OR_RIGHT; //中间状态标志
extern _Move_Flag Move_Flag;
extern _Move_Flag TargetNum[8]; //目标位置数组，存储接受到的目标位置命令
extern uint8_t CmdIndex; //命令索引位，表示当前接受到的命令条数

int i = 0;
float L_Target_Position = 20000;
float Now_Position = 0;
#define SPEED_RECORD_NUM 20 // 经测试，50Hz个采样值进行滤波的效果比较好
float speed_Record[SPEED_RECORD_NUM] = {0};

/**
 * @brief 平均滤波
 * @param new_Spe 新输入的数据
 * @param speed_Record 速度数列
 * @return 速度平均值
 * @note 因为本例未使用，精度已经有0.01，参数未整定
 */
float Speed_Low_Filter(float new_Spe, float *speed_Record)
{
    float sum = 0.0f;
    float test_Speed = new_Spe;
    for (uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) //将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i - 1];
        sum += speed_Record[i - 1];
    }
    speed_Record[0] = new_Spe; //第一位是新的数据
    sum += new_Spe;
    test_Speed = sum / SPEED_RECORD_NUM;
    return sum / SPEED_RECORD_NUM; //返回均值
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //定时器回调函数，用于计算速度
{
    if (htim->Instance == GAP_TIM.Instance) //10ms间隔定时器中断，计算速度、调整速度、发送参数
    {
        // /************位置环*************/
        //  Now_Position = (float)(motor1.totalCount-10000);// 得到当前位置 10000编码器脉冲计数的初始值
        //  Now_Position = MappingProp(Now_Position,12000,-12000,72,-72);
        //  L_Target_Speed = Location_PID_Realize(&pid_l_position,L_Target_Position,Now_Position);//位置环 Target_Position是目标位置，自行定义即可
        // /***************************PID速度环**********************************/
        // motor_Out1 = Speed_PID_Realize(&pid_l_speed,L_Target_Speed,motor1.speed);
        // //L_Target_Speed是目标速度，自行定义就好
        //  if(motor_Out1 >= 0)
        //  {
        //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM)-motor_Out1);
        //  }
        //  else
        //  {
        //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM)+motor_Out1);
        //  }
        // /*******************************姿态读取***************************/
        // MPU6050_Kalman_Euler_Angels();
        /*******************新一版PID速度环*********************/
        DAP_Com_Length = Command_GetCommand(DAP_Com);//DAP_Com_Length事实上是包含包头和校验和的字节数
        if (DAP_Com_Length != 0) //如果有命令
        {
            Decode_Command(DAP_Com_Length-3);
            HAL_UART_Transmit(&huart3, DAP_Com_Decode, strlen(DAP_Com_Decode), HAL_MAX_DELAY); //回显解码之后的命令
            USART_Parse_Command(DAP_Com_Decode, 1); //解析命令，以及变更索引位
        }
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);
        Motor_Get_Speed(&motor1);
        Motor_Get_Speed(&motor2);
        //MPU6050_Kalman_Euler_Angels();
        // if (TargetNum[0] != 0)
        // {
        //     if (TargetNum[1] != 0 )
        //     {
        //         if (TargetNum[1] == TargetNum[0])
        //         {
        //             Flag_FAR_OR_MID = MID;
        //             Flag_MID_LEFT_OR_RIGHT = LEFT;
        //         }
        //         else if (TargetNum[2] == TargetNum[0])
        //         {
        //             Flag_FAR_OR_MID = MID;
        //             Flag_MID_LEFT_OR_RIGHT = RIGHT;
        //         }
        //         else
        //             Flag_FAR_OR_MID = FAR;
        //     }
            if (Move_Flag == MOVE_TO_1)
            {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                Control_goto_1();
            }
            else if (Move_Flag == MOVE_TO_2)
            {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                Control_goto_2();
            }
            else
            {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                Control_goto_345678();
            }
        //}
        //Control_goto_345678();
        Motor_PID_Compute();
        /*******************************串口发送数据*********************************/
        i++;
        if (i >= 10)
        {
            i = 0;
            //HAL_UART_Transmit(&huart1,Gray_rx_buff,strlen(Gray_rx_buff),HAL_MAX_DELAY);
            // sprintf(message, "speed:%.2f,%.2f,%.2f,%.2f\r\n", motor1.speed, motor2.speed, pid_l_speed.SP,
            //          pid_r_speed.SP);
            // HAL_UART_Transmit_IT(&huart3, message, strlen(message));

           //sprintf(Gray_rx_buff, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n", (Digtal >> 0) & 0x01, (Digtal >> 1) & 0x01,
           //        (Digtal >> 2) & 0x01, (Digtal >> 3) & 0x01, (Digtal >> 4) & 0x01, (Digtal >> 5) & 0x01,
           //        (Digtal >> 6) & 0x01, (Digtal >> 7) & 0x01);
           //HAL_UART_Transmit_IT(&huart1, Gray_rx_buff, strlen(Gray_rx_buff));
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        Command_Write(DAP_Com_Buff, Size);
        //HAL_UART_Transmit(huart, DAP_Com_Buff, Size, HAL_MAX_DELAY); //回显接收到的命令
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, DAP_Com_Buff, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }

    // if (huart->Instance == USART3) // 如果是蓝牙串口
    // {
    //     HAL_UART_Transmit_DMA(&huart1, BLE_Com, Size);
    //
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, BLE_Com, 128);
    //     __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    // }
}
