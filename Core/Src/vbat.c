//
// Created by lak19 on 2025/7/5.
//

#include "vbat.h"

#include "adc.h"
#include "stm32f1xx_hal_adc.h"

void Vbat_Init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
}
float Get_Vbat() {
    return HAL_ADC_GetValue(&hadc1)/4095.0f*11.0f*3.3f; // 3.3V参考电压，11.0是分压电阻的比例
}