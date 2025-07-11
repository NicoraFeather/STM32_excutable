//
// Created by lak19 on 2025/6/25.
//

#include "callback.h"
#include "pid.h"
#include "control.h"
#include "vbat.h"
#include "wireless.h"
const uint8_t UART_RX_BUF_SIZE = 128; //串口接收缓冲区大小

uint8_t DataBuff[128] = {1,2,3}; //接收数据缓存数组
float motor_Out1=0;
float motor_Out2=0;
char message[100]="";
extern DMA_HandleTypeDef hdma_usart1_rx;
extern PID pid_l_speed,pid_l_position,pid_r_speed,pid_r_position;
extern Motor motor1;
extern Motor motor2;
extern uint8_t enable_flag; // 用于标记是否开启计算
extern float theta_ref; extern float x_dot;//test
extern uint8_t command_received[128];
int i=0;
float L_Target_Position=20000;
float Now_Position=0;
#define SPEED_RECORD_NUM 20 // 经测试，50Hz个采样值进行滤波的效果比较好
float speed_Record[SPEED_RECORD_NUM]={0};

/**
 * @brief 平均滤波
 * @param new_Spe 新输入的数据
 * @param speed_Record 速度数列
 * @return 速度平均值
 * @note 因为本例未使用，精度已经有0.01，参数未整定
 */
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    float test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;
    return sum/SPEED_RECORD_NUM;//返回均值
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{

    // if(htim->Instance==GAP_TIM.Instance)//10ms间隔定时器中断，计算速度、调整速度、发送参数
    // {
    //     // /************位置环*************/
    //     //  Now_Position = (float)(motor1.totalCount-10000);// 得到当前位置 10000编码器脉冲计数的初始值
    //     //  Now_Position = MappingProp(Now_Position,12000,-12000,72,-72);
    //     //  L_Target_Speed = Location_PID_Realize(&pid_l_position,L_Target_Position,Now_Position);//位置环 Target_Position是目标位置，自行定义即可
    //     // /***************************PID速度环**********************************/
    //     // motor_Out1 = Speed_PID_Realize(&pid_l_speed,L_Target_Speed,motor1.speed);
    //     // //L_Target_Speed是目标速度，自行定义就好
    //     //  if(motor_Out1 >= 0)
    //     //  {
    //     //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
    //     //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM)-motor_Out1);
    //     //  }
    //     //  else
    //     //  {
    //     //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
    //     //      __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM)+motor_Out1);
    //     //  }
    //     // /*******************************姿态读取***************************/
    //     // MPU6050_Kalman_Euler_Angels();
    //     /*******************新一版PID速度环*********************/
    //     Motor_Get_Speed(&motor1);
    //     Motor_Get_Speed(&motor2);
    //     MPU6050_Kalman_Euler_Angels();
    //     if (enable_flag)
    //     {
    //         Control_Compute();
    //         Motor_PID_Compute();
    //     }
    //     /*******************************串口发送数据*********************************/
    //     i++;
    //     if (i>=10) {
    //         i=0;
    //         // MPU6050_Update();
    //         // uint8_t reg;
    //         // IIC_Simula_Read(MPU6050_ADDR_AD0_LOW, WHO_AM_I, 1, &reg);
    //         float t = Get_us64();
    //         sprintf(message,"speed:%.2f,%.2f,%.2f,%.2f,%.2f\r\n",motor1.speed,theta_ref,x_dot,Mpu6050_Data.Gyro_X,pid_l_speed.SP);
    //         HAL_UART_Transmit_DMA(&huart1,message,strlen(message));

    //     }
    // }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) //如果是串口1（上位机）
    {
        DataBuff[Size] = '\0'; // 末尾补0，方便字符串处理\

        // 处理数据
        USART_Parse_Command(DataBuff, 1);// 1表示左电机

       // HAL_UART_Transmit_DMA(huart, DataBuff, Size); // 回传接收到的数据

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, DataBuff, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
    }

    if (huart->Instance == USART3)// 如果是蓝牙串口
    {
        if (command_received[0]=='A')
            HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        HAL_UART_Transmit_DMA(&huart3, command_received, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,command_received, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
    }
}