#include "pid.h"

PID pid_l_speed,pid_l_position;
extern float L_Target_Position;
/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
void PID_Init(void)//PID参数初始化
{
    pid_l_speed.err = 0;
    pid_l_speed.integral = 0;
    pid_l_speed.maxIntegral = 5000;
    pid_l_speed.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);  //获取自动重装载值
    pid_l_speed.lastErr = 0;
    pid_l_speed.output = 0;
    pid_l_speed.kp = KP_speed; //初始值
    pid_l_speed.ki = KI_speed;
    pid_l_speed.kd = KD_speed;

    pid_l_position.err = 0;
    pid_l_position.integral = 0;
    pid_l_position.maxIntegral = 80;
    pid_l_position.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_l_position.lastErr = 0;
    pid_l_position.output = 0;
    pid_l_position.kp = KP_position; //初始值
    pid_l_position.ki = KI_position;
    pid_l_position.kd = KD_position;

}

/**
 * @brief 编码器速度环
 * @param pid 目标PID结构体
 * @param target 目标值，目标的速度
 * @param feedback 反馈值，测得的速度
 * @return 调整量，上限为7200的PWM比较值
 */
float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0)
            pid->output = 0;       //当target大于0时，正常是不会有负值的
        else if(pid->output > pid->maxOutput)
            pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput)
            pid->output = -pid->maxOutput;
        else if(pid->output > 0)
            pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
    return pid->output;
}

/**
 * @brief 位置环
 * @param pid 目标PID结构体
 * @param target 目标值
 * @param feedback 反馈值，测得的距离
 * @return 速度值，单位n/s
 */
float Location_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    if(pid->err < 0.5 && pid->err > -0.5) pid->err = 0;//pid死区
    pid->err = target - feedback;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}


/**
 * @brief 解析出DataBuff中的数据
 * @return 解析得到的数据
 * @note 待改进：算法很笨使用字符数组遍历，可以用更好的字符处理函数
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数，数据有可能是负数，需要特别处理
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置，200是指最大允许接收数据
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;                      // 定位到感叹号就结束查询
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;    //得到的是数据的长度
    if(data_Num == 4) // 数据共4位，4.02
    {
        data_return = (DataBuff[data_Start_Num]-ASCII_0)  + (DataBuff[data_Start_Num+2]-ASCII_0)*0.1f +
                (DataBuff[data_Start_Num+3]-ASCII_0)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位,40.22
    {
        data_return = (DataBuff[data_Start_Num]-ASCII_0)*10 + (DataBuff[data_Start_Num+1]-ASCII_0) + (DataBuff[data_Start_Num+3]-ASCII_0)*0.1f +
                (DataBuff[data_Start_Num+4]-ASCII_0)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位,450.22
    {
        data_return = (DataBuff[data_Start_Num]-ASCII_0)*100 + (DataBuff[data_Start_Num+1]-ASCII_0)*10 + (DataBuff[data_Start_Num+2]-ASCII_0) +
                (DataBuff[data_Start_Num+4]-ASCII_0)*0.1f + (DataBuff[data_Start_Num+5]-ASCII_0)*0.01f;
    }
    else if(data_Num == 7) // 数据共7位,4500.22
    {
        data_return = (DataBuff[data_Start_Num]-ASCII_0)*1000 + (DataBuff[data_Start_Num+1]-ASCII_0)*100 + (DataBuff[data_Start_Num+2]-ASCII_0)*10 +
                (DataBuff[data_Start_Num+3]-ASCII_0) + (DataBuff[data_Start_Num+5]-ASCII_0)*0.1f+ (DataBuff[data_Start_Num+6]-ASCII_0)*0.01f;
    }
    else if(data_Num == 8) // 数据共8位,45000.22
    {
        data_return = (DataBuff[data_Start_Num]-ASCII_0)*10000 + (DataBuff[data_Start_Num+1]-ASCII_0)*1000 + (DataBuff[data_Start_Num+2]-ASCII_0)*100 +
                (DataBuff[data_Start_Num+3]-ASCII_0)*10 + (DataBuff[data_Start_Num+4]-ASCII_0)+ (DataBuff[data_Start_Num+6]-ASCII_0)*0.1f + (DataBuff[data_Start_Num+7]-ASCII_0)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    printf("data=%.2f\r\n",data_return);
    return data_return;
}

/**
 * @brief 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
    //    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 2)//右边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            pid_l_position.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            pid_l_position.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            pid_l_position.kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            pid_l_speed.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            pid_l_speed.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            pid_l_speed.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
            L_Target_Speed = data_Get;
         else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
             L_Target_Position = data_Get;
    }
    else if(Motor_n == 1) // 左边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            pid_l_position.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            pid_l_position.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            pid_l_position.kd = data_Get;
        if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            pid_l_speed.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            pid_l_speed.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            pid_l_speed.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
            L_Target_Speed = data_Get;
        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
            L_Target_Position = data_Get;
    }
}

//
// Created by lak19 on 2025/6/25.
//
