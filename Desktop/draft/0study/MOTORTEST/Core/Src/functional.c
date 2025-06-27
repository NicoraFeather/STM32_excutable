//
// Created by lak19 on 2025/6/27.
//
#include "functional.h"
/**
 * @brief 用于实现浮点数值的比例映射，错误时返回NAN
 * @param data_in 进入的数值
 * @param in_up 原始范围上限
 * @param in_down 原始范围下限
 * @param out_up 映射目标上限
 * @param out_down 映射目标下限
 * @return 映射变换后的浮点值
 */
float MappingProp(float data_in, float in_up,float in_down,float out_up,float out_down) {
    if (in_down>=in_up || out_down>=out_up) {
        return NAN;
    }
    else {
        data_in=data_in*out_up-data_in*out_down-in_down*out_up+in_up*out_down;
        data_in=data_in/(in_up-in_down);
        return data_in;
    }
}


/**
 * @brief 用于限制幅度，错误时返回NAN
 * @param data_in 进入的数值
 * @param up 幅度上限
 * @param down 幅度下限
 * @return 限幅后的浮点数
 */
float LimAmplitude(float data_in, float up,float down) {
    if (down >= up)
        return NAN;
    else {
        if (data_in >= up)
            return up;
        else if (data_in <= down)
            return down;
        else
            return data_in;
    }
}