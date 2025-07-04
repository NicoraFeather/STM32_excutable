//
// Created by lak19 on 2025/6/27.
//

#ifndef FUNCTIONAL_H
#define FUNCTIONAL_H

#include <math.h>
#include <stdlib.h>

float MappingProp(float data_in, float in_up,float in_down,float out_up,float out_down);
float LimAmplitude(float data_in, float up,float down);
float Deg_to_Rag(float deg);
float Rag_to_Deg(float reg);
float CompleFilt_alpha(float Cutoff_f, float Sampling_f);
float CompleFilter(float alpha, float lowfreq_data, float highfreq_data);


#endif //FUNCTIONAL_H
