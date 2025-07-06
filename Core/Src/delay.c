//
// Created by lak19 on 2025/6/28.
//
#include "delay.h"
#include "stm32f1xx.h"

// DWT寄存器定义


// 全局64位微秒计数器
volatile uint64_t micros_counter = 0;

// 最后记录的周期值（用于检测溢出）
volatile uint32_t last_cycle = 0;


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

// 初始化DWT并启动微秒计时
void DWT_InitMicros(void) {
    // 使能DWT跟踪
    DEM_CR |= (1 << 24);

    // 重置计数器
    DWT_CYCCNT = 0;

    // 启用周期计数器
    DWT_CR |= 1;

    // 初始化变量
    last_cycle = 0;
    micros_counter = 0;

    // 启用SysTick中断用于周期更新
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = SystemCoreClock / 1000 - 1;  // 1ms中断
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}



// 获取从初始化开始的微秒数
uint64_t Get_us64(void) {
    // 需要临界区保护，防止在SysTick中断更新时读取
    __disable_irq();

    // 获取当前周期计数
    uint32_t current_cycle = DWT_CYCCNT;

    // 计算自上次中断以来的增量
    uint32_t partial_cycles = current_cycle - last_cycle;

    // 计算部分微秒数
    uint64_t partial_micros = (uint64_t)partial_cycles / 72;

    // 组合完整值
    uint64_t result = micros_counter + partial_micros;

    __enable_irq();

    return result;
}

