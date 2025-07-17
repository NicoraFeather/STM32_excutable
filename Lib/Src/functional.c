//
// Created by lak19 on 2025/6/27.
//
#include "../Inc/functional.h"
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
    return rag*57.2957795130823f; // 180/M_PI
}

/**
 * 角度向弧度换算
 * @param deg 角度
 * @return 弧度
 */
float Deg_to_Rag(float deg) {
    return deg*0.0174532925199433f; // M_PI/180
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

/**************滑动窗口滤波***************/

/**
 * @brief 初始化队列
 * @param win_size 窗口大小
 * @return 创建队列的指针
 */
struct queue* create_queue(int win_size) {
    struct queue* q = malloc(sizeof(struct queue));
    q->front = q->rear = NULL;
    q->win_size = win_size;
    q->curr_num = 0;
    q->sum = 0.0f;
    q->avg = 0.0f;
    return q;
}

/**
 * @brief 入队函数
 * @param q 目标队列的指针
 * @param new_data 新的数据
 */
void enqueue(struct queue* q, float new_data)
 {
    if(!q || q->curr_num > q->win_size) //如果数据异常，不做操作
        return;
    struct Node* newNode = malloc(sizeof(struct Node)); // 为新的节点申请内存空间
    newNode->data = new_data; //设置新节点的数据
    newNode->next = NULL; //这时候新的节点是最后一个节点，后面没有了，改成空指针

    if (q->rear == NULL)   //如果队列是空的（这里rear和front类似的）
    {
        q->front = q->rear = newNode; //新节点是唯一的节点
    }
    else
    {
        q->rear->next = newNode;   //将当前的尾部节点的next指向新节点，将新的节点接入链表
        q->rear = newNode;  //这时候新的节点成为了最后一个充当新的rear
    }
    q->curr_num++;  //队列元素数量加一
}

/**
 * @brief 出队函数
 * @param q 目标队列指针
 * @return 出队数据的值
 */
float dequeue(struct queue* q) {   //形参是队列结构体指针
    if (!q || q->front == NULL) return -1;//如果数列是空或异常,返回错误

    struct Node* temp = q->front;  //备份头部节点，front是要反复使用的
    float old_data = temp->data;   //备份数据

    q->front = q->front->next;  //将front移动到下一个节点，解除链表的联系
    if (q->front == NULL)  //如果队列空，也就是你把最后一个数据出队了，那么链表退化成了初始状态，front和rear都是NULL
        q->rear = NULL;  //要手动设置，否则rear和原来的front是一样的

    free(temp);    //释放内存，解除联系后彻底删除
    q->curr_num--;
    return old_data;
}

/**
 * @brief 滑动窗口滤波函数
 * @param q 目标队列指针
 * @param new_data 新输入数据
 * @return 当前滤波后的平均值
 */
void Sliding_Window(struct queue* q, float new_data)
{
    if (!q) return;   //如果是队列异常

    if (q->curr_num == q->win_size)  //如果数值等于窗口，然后还要在加上的话，先出队
    {
        float removed = dequeue(q);
        q->sum -= removed;
    }

    enqueue(q, new_data);
    q->sum += new_data;
    q->avg = q->sum / q->curr_num;
}
