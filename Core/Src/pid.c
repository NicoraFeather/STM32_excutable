#include "pid.h"

#include "../../Lib/Inc/delay.h"
#include "../../Lib/Inc/functional.h"
#include "vbat.h"

PID pid_l_speed, pid_l_position, pid_r_speed, pid_r_position;
extern float L_Target_Position;
extern Motor motor1;
extern Motor motor2;
/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
// void PID_Init(void)//PID参数初始化
// {
//     pid_l_speed.err = 0;
//     pid_l_speed.integral = 0;
//     pid_l_speed.maxIntegral = 5000;
//     pid_l_speed.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);  //获取自动重装载值
//     pid_l_speed.lastErr = 0;
//     pid_l_speed.output = 0;
//     pid_l_speed.kp = KP_speed; //初始值
//     pid_l_speed.ki = KI_speed;
//     pid_l_speed.kd = KD_speed;
//
//     pid_l_position.err = 0;
//     pid_l_position.integral = 0;
//     pid_l_position.maxIntegral = 80;
//     pid_l_position.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
//     pid_l_position.lastErr = 0;
//     pid_l_position.output = 0;
//     pid_l_position.kp = KP_position; //初始值
//     pid_l_position.ki = KI_position;
//     pid_l_position.kd = KD_position;
// }

// void PID_Init(PID * pid,float KP, float KI, float KD, float maxintegral, float maxoutput)//PID参数初始化
// {
//     pid -> err = 0;
//     pid -> integral = 0;
//     pid -> maxIntegral = maxintegral;
//     pid -> maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);  //获取自动重装载值
//     pid -> lastErr = 0;
//     pid -> output = 0;
//     pid -> kp = KP; //初始值
//     pid -> ki = KI;
//     pid -> kd = KD;
//     pid -> SP =0.0f;
// }
/**
 * 初始化PID结构体，所有量归零
 * @param pid
 */
void PID_Init_General(PID * pid)
{
    pid -> err = 0;
    pid -> integral = 0;
    pid -> lastErr = 0;
    pid -> output = 0;

    pid -> kp = 0; //保证一开始为0
    pid -> ki = 0;
    pid -> kd = 0;
    pid -> SP =0.0f;

    pid -> t_k_1 = 0;
    pid->err_k_1 = 0;
    pid->err_int_k_1 = 0;

    pid -> LowOutputLim = -3.4e38f; //表示初始不限制输出的幅度
    pid -> HighOutputLim = 3.4e38f;
}

/**
 * 设置PID超参数
 * @param pid
 * @param KP
 * @param KI
 * @param KD
 */
void PID_Set_General(PID * pid, float KP, float KI, float KD)
{
    pid->kp = KP;
    pid->ki = KI;
    pid->kd = KD;
}

/**
 * 改变PID目标值
 * @param pid
 * @param SP PID目标值
 */
void PID_ChangeSP_General(PID * pid, float SP)
{
    pid -> SP = SP;
}

/**
 * PID计算环节
 * @param pid
 * @param FB 反馈值
 * @return PID环节后的输出量
 */
float PID_Compute_General(PID * pid, float FB)
{
    float err = pid->SP - FB;

    uint32_t t_k = Get_us64();

    float deltaT = (pid->t_k_1 == 0) ? 0.01f : (t_k - pid->t_k_1) * 1.0e-6f; // 第一次假定周期为10ms
    //float deltaT = (t_k - pid->t_k_1) * 1.0e-6f;
    float err_dev = 0.0f;
    float err_int = pid->err_int_k_1;

    if (pid->t_k_1 != 0)
    {
        err_dev = (err - pid->err_k_1) / deltaT;
        err_int = pid->err_int_k_1 + (err + pid->err_k_1) * deltaT * 0.5f;
    }

    float COP = pid->kp * err;
    float COI = pid->ki * err_int;
    float COD = pid->kd * err_dev;
    float CO =COD + COI + COP;
    //更新数据
    pid->t_k_1 = t_k;
    pid->err_k_1 = err;
    pid->err_int_k_1 = err_int;
    //输出限幅
    if (CO >= pid->HighOutputLim)
        CO = pid->HighOutputLim;
    else if (CO <= pid->LowOutputLim)
        CO = pid->LowOutputLim;
    //积分限幅
    if (pid->err_int_k_1 >= pid->HighOutputLim)
        pid->err_int_k_1 = pid->HighOutputLim;
    else if (pid->err_int_k_1 < pid->LowOutputLim)
        pid->err_int_k_1 = pid->LowOutputLim;

    return CO;
}

/**
 * PID设定限制幅度函数
 * @param pid
 * @param LowOutputLim 设定下限
 * @param HighOutputLim 设定上限
 */
void PID_LimConfig_General(PID * pid, float LowOutputLim, float HighOutputLim)
{
    pid -> LowOutputLim = LowOutputLim;
    pid -> HighOutputLim = HighOutputLim;
}

/**
 * PID重置函数
 * @param pid
 */
void PID_Reset_General(PID * pid)
{
    pid->err_int_k_1 = 0;
    pid->err_k_1 = 0;
    pid->t_k_1 = 0;
}

/**
 * 电机PID速度环计算函数
 */
void Motor_PID_Compute(void)
{
    float vbat = Get_Vbat();
    // float vbat = 12.0f; // 假设电池电压为12V，实际应用中应从VBAT获取

    PID_LimConfig_General(&pid_l_speed,-vbat, vbat);
    PID_LimConfig_General(&pid_r_speed,-vbat, vbat);

    float u_l = PID_Compute_General(&pid_l_speed, (motor1.speed));
    float u_r = PID_Compute_General(&pid_r_speed, (motor2.speed));

    if(u_l >= 0)
    {
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f - u_l/vbat)));
    }
    else
    {
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f + u_l/vbat)));
    }

    // if(u_l >= 0) {
    //     // 正转：正向通道输出有效PWM（占空比 = 1 - u_l/vbat），反向通道停止
    //     __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD,
    //         (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f - u_l/vbat)));
    //     __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD,
    //         __HAL_TIM_GetAutoreload(&PWM_TIM)); // 反向通道100% -> 停止
    // } else {
    //     // 反转：反向通道输出有效PWM，正向通道停止
    //     __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD,
    //         __HAL_TIM_GetAutoreload(&PWM_TIM)); // 正向通道100% -> 停止
    //     __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD,
    //         (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f + u_l/vbat))); // 注意u_l为负
    // }

    if(u_r >= 0)
    {
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f - u_r/vbat)));
    }
    else
    {
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, __HAL_TIM_GetAutoreload(&PWM_TIM));
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, (uint32_t)(__HAL_TIM_GetAutoreload(&PWM_TIM) * (1.0f + u_r/vbat)));
    }
}






// /**
//  * @brief 编码器速度环
//  * @param pid 目标PID结构体
//  * @param target 目标值，目标的速度
//  * @param feedback 反馈值，测得的速度
//  * @return 调整量，上限为7200的PWM比较值
//  */
// float Speed_PID_Realize(PID * pid,float target,float feedback)//一次PID计算
// {
//     pid->err = target - feedback;
//     if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
//     pid->integral += pid->err;
//
//     if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
//     else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;
//
//     if(target == 0) pid->integral = 0; // 刹车时清空i
//
//     pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID
//
//     //输出限幅
//     if(target >= 0)//正转时
//     {
//         if(pid->output < 0)
//             pid->output = 0;       //当target大于0时，正常是不会有负值的
//         else if(pid->output > pid->maxOutput)
//             pid->output = pid->maxOutput;
//     }
//     else if(target < 0)//反转时
//     {
//         if(pid->output < -pid->maxOutput)
//             pid->output = -pid->maxOutput;
//         else if(pid->output > 0)
//             pid->output = 0;
//     }
//
//     pid->lastErr = pid->err;
//     if(target == 0) pid->output = 0; // 刹车时直接输出0
//     return pid->output;
// }
//
// /**
//  * @brief 位置环
//  * @param pid 目标PID结构体
//  * @param target 目标值
//  * @param feedback 反馈值，测得的距离
//  * @return 速度值，单位n/s
//  */
// float Location_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
// {
//     if(pid->err < 0.5 && pid->err > -0.5) pid->err = 0;//pid死区
//     pid->err = target - feedback;
//     pid->integral += pid->err;
//
//     if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
//     else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;
//
//     pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID
//
//     //输出限幅
//     if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
//     if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
//
//     pid->lastErr = pid->err;
//
//     return pid->output;
// }




//
// Created by lak19 on 2025/6/25.
//
