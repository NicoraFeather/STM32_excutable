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

/**
 * 弧度向角度换算
 * @param reg 弧度
 * @return 角度
 */
float Rag_to_Deg(float rag) {
    return rag*180.0f/M_PI;
}

/**
 * 角度向弧度换算
 * @param deg 角度
 * @return 弧度
 */
float Deg_to_Rag(float deg) {
    return deg/180.0f*M_PI;
}

/**
 * 计算互补滤波器系数α
 * @param Cutoff_f 转折频率，高频和低频的分界线
 * @param Sampling_f 采样频率
 * @return 互补滤波器的系数
 */
float CompleFilt_alpha(float Cutoff_f, float Sampling_f) {
    return 1.0f/(1.0f+Cutoff_f/Sampling_f);
}

/**
 * 互补滤波
 * @param alpha 互补滤波器系数
 * @param lowfreq_data 低频数据
 * @param highfreq_data 高频数据
 * @return 滤波器数据
 */
float CompleFilter(float alpha, float lowfreq_data, float highfreq_data) {
    return alpha * lowfreq_data + (1 - alpha) * highfreq_data;
}