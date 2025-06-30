//
// Created by lak19 on 2025/6/28.
//

#ifndef DELAY_H
#define DELAY_H

#include "tim.h"
/**
 * 裸机多任务执行函数
 * @param T 毫秒数
 */
#define PERIODIC(T) \
    static uint32_t nxt = 0; \
    if(HAL_GetTick() < nxt) return; \
    nxt += (T);

void delay_us(uint32_t us);

#endif //DELAY_H
