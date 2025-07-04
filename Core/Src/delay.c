//
// Created by lak19 on 2025/6/28.
//
#include "delay.h"

/**
 * @brief 微秒级别延时，仅适用于F1系列72M主频
 * @param us 微秒数
 */
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
    {
        ;
    }
}


//使用一个定时器进行延时，频率1M
// #define DLY_TIM_Handle (&htim4)  //延时专用定时器
// void delay_us(uint16_t nus)
// {
//     __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
//     __HAL_TIM_ENABLE(DLY_TIM_Handle);
//     while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
//     {
//     }
//     __HAL_TIM_DISABLE(DLY_TIM_Handle);
// }




