//
// Created by lak19 on 2025/7/7.
//

#include "../Inc/wireless.h"
uint8_t BLE_Com[128] = {0};
void BlueTooth_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,BLE_Com, 128);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}