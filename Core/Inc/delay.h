//
// Created by lak19 on 2025/6/28.
//

#ifndef DELAY_H
#define DELAY_H

#include "tim.h"

#define DWT_CR       (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT   (*(volatile uint32_t*)0xE0001004)
#define DEM_CR       (*(volatile uint32_t*)0xE000EDFC)

/**
 * 裸机多任务执行函数
 * @param T 毫秒数
 */
#define PERIODIC(T) \
    static uint32_t nxt = 0; \
    if(HAL_GetTick() < nxt) return; \
    nxt += (T);

void delay_us(uint32_t us);
void DWT_InitMicros(void);
uint64_t Get_us64(void) ;

#endif //DELAY_H
