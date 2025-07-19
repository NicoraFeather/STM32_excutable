//
// Created by lak19 on 2025/7/14.
//

#include "uart_command.h"
#include "Grayscale_Sensor.h"
#include "pid.h"

extern PID pid_l_speed, pid_l_position, pid_r_speed, pid_r_position;

uint8_t Uart1_DataBuff[128] = {0};
uint8_t dataArr[128] = {0}; // 假设最大数据长度为128
uint8_t target_flag = 0; // 标记目的地
/*************************帧格式************************
 * 帧头  | 数据长度 | 数据内容 | 校验和 |
 * 0xAA |   N      |   Data   |  Sum   |
 * N = 帧的所有内容的长度
 * (帧头和校验和)
 * 校验和为除去校验和的所有字节之和
 * 注意：数据内容的长度不能超过125字节
 ****************************************************/

/********************命令格式********************
 * P2=1.23!  // 设置电机速度环PID参数
 * I2=0.01!
 * D2=0.001!
 * P1=0.5!   // 设置电机位置环PID参数
 * I1=0.01!
 * D1=0.001!
 * Pos=10000! // 设置电机目标位置
 * Spe=5!     // 设左电机目标速度
 * F=6!      // 设置前进速度，可以为负数
 * L=3!      // 设置左转速度
 * R=3!      // 设置右转速度
 * RES_W     // 设定灰度传感器白色标定值，返回八位数组
 * RES_B     // 设定灰度传感器黑色标定值，返回八位数组
 ****************************************************/


/**
 * 简单的命令解析函数，放在DMA空闲中断回调函数
 * @param huart
 */
void Rx_Command(uint16_t size)
{
    if (Uart1_DataBuff[0] == 0xAA)
    {
        if (Uart1_DataBuff[1] == size) {
            uint8_t sum = 0;
            for (int i = 0; i < size - 1; i++)
                sum += Uart1_DataBuff[i];
            if (sum == Uart1_DataBuff[size - 1])
                {
                // 校验通过，提取数据
                uint8_t dataLen = size - 3; // 数据从DataBuff[2]开始，最后一位是校验和
                for (uint8_t i = 0; i < dataLen; i++) {
                    dataArr[i] = Uart1_DataBuff[2 + i];
                }
                // 此处可根据需要处理dataArr
            }
        }
    }
}


void USART_Parse_Command(char* str, uint8_t motor_n)
{
    char* cmd = strtok(str, "=");
    char* val = strtok(NULL, "!");
    if(cmd == NULL) return;

    // 处理无参数命令
    if(strcmp(cmd, "RES_W") == 0)
    {
        // 这里调用灰度传感器白色标定函数
        //Grayscale_White_Calibrate();
        return;
    }
    if(strcmp(cmd, "RES_B") == 0)
    {
        // 这里调用灰度传感器黑色标定函数
        //Grayscale_Black_Calibrate();
        return;
    }

    // 处理有参数命令
    if(val == NULL) return;
    float value = atof(val);

    if (motor_n == 1) // 左电机
    {
        if(strcmp(cmd, "P2") == 0)
            pid_l_speed.kp = value;
        else if(strcmp(cmd, "I2") == 0)
            pid_l_speed.ki = value;
        else if (strcmp(cmd, "D2") == 0)
            pid_l_speed.kd = value;
        else if (strcmp(cmd, "P1") == 0)
            pid_l_position.kp = value;
        else if (strcmp(cmd, "I1") == 0)
            pid_l_position.ki = value;
        else if (strcmp(cmd, "D1") == 0)
            pid_l_position.kd = value;
        else if (strcmp(cmd, "Pos") == 0)
            pid_l_position.SP = value;
        else if (strcmp(cmd, "Spe") == 0)
            pid_l_speed.SP = value;
    }
    else if (motor_n == 2) // 右电机
    {
        if(strcmp(cmd, "P2") == 0)
            pid_r_speed.kp = value;
        else if(strcmp(cmd, "I2") == 0)
            pid_r_speed.ki = value;
        else if (strcmp(cmd, "D2") == 0)
            pid_r_speed.kd = value;
        else if (strcmp(cmd, "P1") == 0)
            pid_r_position.kp = value;
        else if (strcmp(cmd, "I1") == 0)
            pid_r_position.ki = value;
        else if (strcmp(cmd, "D1") == 0)
            pid_r_position.kd = value;
        else if (strcmp(cmd, "Pos") == 0)
            pid_r_position.SP = value;
        else if (strcmp(cmd, "Spe") == 0)
            pid_r_speed.SP = value;
    }

    // 公共控制命令
    if(strcmp(cmd, "F") == 0) {
        PID_ChangeSP_General(&pid_l_speed, value);
        PID_ChangeSP_General(&pid_r_speed, value);
    }

    else if(strcmp(cmd, "R") == 0) {
        PID_ChangeSP_General(&pid_l_speed, value);
        PID_ChangeSP_General(&pid_r_speed, -value);
    }

    else if(strcmp(cmd, "L") == 0) {
        PID_ChangeSP_General(&pid_l_speed, -value);
        PID_ChangeSP_General(&pid_r_speed, value);
    }

}
