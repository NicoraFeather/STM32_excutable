//
// Created by lak19 on 2025/6/27.
//

#ifndef FUNCTIONAL_H
#define FUNCTIONAL_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


struct Node //链表节点结构体
{
    float data;
    struct Node* next;
};

struct queue //队列结构体
{
    float avg;     //平均值
    int win_size;  //窗口大小
    float sum;     //数据的总和
    int curr_num;  //当前队列的元素个数
    struct Node* front;// 头指针
    struct Node* rear; //尾指针
};


float MappingProp(float data_in, float in_up,float in_down,float out_up,float out_down);
float LimAmplitude(float data_in, float up,float down);
float Deg_to_Rag(float deg);
float Rag_to_Deg(float reg);
float CompleFilt_alpha(float Cutoff_f, float Sampling_f);
float CompleFilter(float alpha, float lowfreq_data, float highfreq_data);
void Sliding_Window(struct queue* q, float new_data);
struct queue* create_queue(int win_size);


#endif //FUNCTIONAL_H
