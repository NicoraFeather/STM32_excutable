#ifndef MPU6050_H
#define MPU6050_H

#include "iic_simulation.h"
#include "functional.h"

#define SMPLRT_DIV   0x19  // 采样率分频，典型值：0x07(125Hz) */
#define CONFIG       0x1A  // 低通滤波频率，典型值：0x06(5Hz) */
#define GYRO_CONFIG  0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define ACCEL_CONFIG 0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

#define ACCEL_XOUT_H 0x3B  // 存储最近的X轴、Y轴、Z轴加速度感应器的测量值 */
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H   0x41  // 存储的最近温度传感器的测量值 */
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x43  // 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define PWR_MGMT_1   0x6B   // 电源管理，典型值：0x00(正常启用) */
#define WHO_AM_I     0x75 	// IIC地址寄存器(默认数值0x68，只读) */

// HAL库的读写只需要使用7位地址
#define MPU6050_ADDR_AD0_LOW 0x68	// AD0低电平时右对齐地址为0X68 iic写发送0XD0，默认数值
#define MPU6050_ADDR_AD0_HIGH 0x69

typedef struct{
    // 角速度
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    // 角度
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    // 温度
    float Temp;
    //欧拉角
    float yaw; //航向角
    float pitch; //俯仰角
    float roll; //翻滚角

    double KalmanPitch;   // X 轴的卡尔曼滤波计算角度
    double KalmanRoll;   // Y 轴的卡尔曼滤波计算角度
}MPU6050DATATYPE;

typedef struct
{
    double Q_angle;    // 角度过程噪声协方差
    double Q_bias;     // 偏差过程噪声协方差
    double R_measure;  // 测量噪声协方差
    double angle;      // 当前估计角度
    double bias;       // 当前估计偏差
    double P[2][2];    // 误差协方差矩阵
} Kalman_t;


extern MPU6050DATATYPE Mpu6050_Data;

void MPU6050_Init(void);
void MPU6050_Update(void);
static void reg_write(uint8_t reg, uint8_t value);
static uint8_t reg_read(uint8_t reg);
void MPU6050_LowFilter_Euler_Angels(void);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void MPU6050_Kalman_Euler_Angels(void);
#endif

