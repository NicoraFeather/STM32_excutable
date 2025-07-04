// #include "MPU6050.h"
// #include "iic_simulation.h"
// #include "usart.h"
// #define PRINT_ACCEL     (0x01)
// #define PRINT_GYRO      (0x02)
// #define PRINT_QUAT      (0x04)
// #define ACCEL_ON        (0x01)
// #define GYRO_ON         (0x02)
// #define MOTION          (0)
// #define NO_MOTION       (1)
// #define DEFAULT_MPU_HZ  (200)
// #define FLASH_SIZE      (512)
// #define FLASH_MEM_START ((void*)0x1800)
// #define q30  1073741824.0f
// short gyro[3], accel[3], sensors;
// float Roll,Pitch,Yaw;
// float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
// static signed char gyro_orientation[9] = {-1, 0, 0,
//                                            0,-1, 0,
//                                            0, 0, 1};
//
// static  unsigned short inv_row_2_scale(const signed char *row)
// {
//     unsigned short b;
//
//     if (row[0] > 0)
//         b = 0;
//     else if (row[0] < 0)
//         b = 4;
//     else if (row[1] > 0)
//         b = 1;
//     else if (row[1] < 0)
//         b = 5;
//     else if (row[2] > 0)
//         b = 2;
//     else if (row[2] < 0)
//         b = 6;
//     else
//         b = 7;      // error
//     return b;
// }
//
//
// static  unsigned short inv_orientation_matrix_to_scalar(
//     const signed char *mtx)
// {
//     unsigned short scalar;
//     scalar = inv_row_2_scale(mtx);
//     scalar |= inv_row_2_scale(mtx + 3) << 3;
//     scalar |= inv_row_2_scale(mtx + 6) << 6;
//
//
//     return scalar;
// }
//
// static void run_self_test(void)
// {
//     int result;
//     long gyro[3], accel[3];
//
//     result = mpu_run_self_test(gyro, accel);
//     if (result == 0x7) {
//         /* Test passed. We can trust the gyro data here, so let's push it down
//          * to the DMP.
//          */
//         float sens;
//         unsigned short accel_sens;
//         mpu_get_gyro_sens(&sens);
//         gyro[0] = (long)(gyro[0] * sens);
//         gyro[1] = (long)(gyro[1] * sens);
//         gyro[2] = (long)(gyro[2] * sens);
//         dmp_set_gyro_bias(gyro);
//         mpu_get_accel_sens(&accel_sens);
//         accel[0] *= accel_sens;
//         accel[1] *= accel_sens;
//         accel[2] *= accel_sens;
//         dmp_set_accel_bias(accel);
// 		//printf("setting bias succesfully ......\r\n");
//     }
// }
//
//
//
// uint8_t buffer[14];
//
// int16_t  MPU6050_FIFO[6][11];
// int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
//
//
//
// /**************************************************************************
// Function: The new ADC data is updated to FIFO array for filtering
// Input   : ax，ay，az：x，y, z-axis acceleration data；gx，gy，gz：x. Y, z-axis angular acceleration data
// Output  : none
// 函数功能：将新的ADC数据更新到 FIFO数组，进行滤波处理
// 入口参数：ax，ay，az：x，y，z轴加速度数据；gx，gy，gz：x，y，z轴角加速度数据
// 返回  值：无
// **************************************************************************/
// void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
// {
// unsigned char i ;
// int32_t sum=0;
// for(i=1;i<10;i++){	//FIFO 操作
// MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
// MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
// MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
// MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
// MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
// MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
// }
// MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
// MPU6050_FIFO[1][9]=ay;
// MPU6050_FIFO[2][9]=az;
// MPU6050_FIFO[3][9]=gx;
// MPU6050_FIFO[4][9]=gy;
// MPU6050_FIFO[5][9]=gz;
//
// sum=0;
// for(i=0;i<10;i++){	//求当前数组的合，再取平均值
//    sum+=MPU6050_FIFO[0][i];
// }
// MPU6050_FIFO[0][10]=sum/10;
//
// sum=0;
// for(i=0;i<10;i++){
//    sum+=MPU6050_FIFO[1][i];
// }
// MPU6050_FIFO[1][10]=sum/10;
//
// sum=0;
// for(i=0;i<10;i++){
//    sum+=MPU6050_FIFO[2][i];
// }
// MPU6050_FIFO[2][10]=sum/10;
//
// sum=0;
// for(i=0;i<10;i++){
//    sum+=MPU6050_FIFO[3][i];
// }
// MPU6050_FIFO[3][10]=sum/10;
//
// sum=0;
// for(i=0;i<10;i++){
//    sum+=MPU6050_FIFO[4][i];
// }
// MPU6050_FIFO[4][10]=sum/10;
//
// sum=0;
// for(i=0;i<10;i++){
//    sum+=MPU6050_FIFO[5][i];
// }
// MPU6050_FIFO[5][10]=sum/10;
// }
//
// /**************************************************************************
// Function: Setting the clock source of mpu6050
// Input   : source：Clock source number
// Output  : none
// 函数功能：设置  MPU6050 的时钟源
// 入口参数：source：时钟源编号
// 返回  值：无
//  * CLK_SEL | Clock Source
//  * --------+--------------------------------------
//  * 0       | Internal oscillator
//  * 1       | PLL with X Gyro reference
//  * 2       | PLL with Y Gyro reference
//  * 3       | PLL with Z Gyro reference
//  * 4       | PLL with external 32.768kHz reference
//  * 5       | PLL with external 19.2MHz reference
//  * 6       | Reserved
//  * 7       | Stops the clock and keeps the timing generator in reset
// **************************************************************************/
// void MPU6050_setClockSource(uint8_t source){
//     IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
//
// }
//
// /** Set full-scale gyroscope range.
//  * @param range New full-scale gyroscope range value
//  * @see getFullScaleRange()
//  * @see MPU6050_GYRO_FS_250
//  * @see MPU6050_RA_GYRO_CONFIG
//  * @see MPU6050_GCONFIG_FS_SEL_BIT
//  * @see MPU6050_GCONFIG_FS_SEL_LENGTH
//  */
// void MPU6050_setFullScaleGyroRange(uint8_t range) {
//     IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
// }
//
// /**************************************************************************
// Function: Setting the maximum range of mpu6050 accelerometer
// Input   : range：Acceleration maximum range number
// Output  : none
// 函数功能：设置 MPU6050 加速度计的最大量程
// 入口参数：range：加速度最大量程编号
// 返回  值：无
// **************************************************************************/
// //#define MPU6050_ACCEL_FS_2          0x00  		//===最大量程+-2G
// //#define MPU6050_ACCEL_FS_4          0x01			//===最大量程+-4G
// //#define MPU6050_ACCEL_FS_8          0x02			//===最大量程+-8G
// //#define MPU6050_ACCEL_FS_16         0x03			//===最大量程+-16G
// void MPU6050_setFullScaleAccelRange(uint8_t range) {
//     IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
// }
//
// /**************************************************************************
// Function: Set mpu6050 to sleep mode or not
// Input   : enable：1，sleep；0，work；
// Output  : none
// 函数功能：设置 MPU6050 是否进入睡眠模式
// 入口参数：enable：1，睡觉；0，工作；
// 返回  值：无
// **************************************************************************/
// void MPU6050_setSleepEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
// }
//
// /**************************************************************************
// Function: Read identity
// Input   : none
// Output  : 0x68
// 函数功能：读取  MPU6050 WHO_AM_I 标识
// 入口参数：无
// 返回  值：0x68
// **************************************************************************/
// uint8_t MPU6050_getDeviceID(void) {
//
//     IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
//     return buffer[0];
// }
//
// /**************************************************************************
// Function: Check whether mpu6050 is connected
// Input   : none
// Output  : 1：Connected；0：Not connected
// 函数功能：检测MPU6050 是否已经连接
// 入口参数：无
// 返回  值：1：已连接；0：未连接
// **************************************************************************/
// uint8_t MPU6050_testConnection(void) {
//    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
//    return 1;
//    	else return 0;
// }
//
// /**************************************************************************
// Function: Setting whether mpu6050 is the host of aux I2C cable
// Input   : enable：1，yes；0;not
// Output  : none
// 函数功能：设置 MPU6050 是否为AUX I2C线的主机
// 入口参数：enable：1，是；0：否
// 返回  值：无
// **************************************************************************/
// void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
// }
//
// /**************************************************************************
// Function: Setting whether mpu6050 is the host of aux I2C cable
// Input   : enable：1，yes；0;not
// Output  : none
// 函数功能：设置 MPU6050 是否为AUX I2C线的主机
// 入口参数：enable：1，是；0：否
// 返回  值：无
// **************************************************************************/
// void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
// }
//
// /**************************************************************************
// Function: initialization Mpu6050 to enter the available state
// Input   : none
// Output  : none
// 函数功能：初始化	MPU6050 以进入可用状态
// 入口参数：无
// 返回  值：无
// **************************************************************************/
// void MPU6050_initialize(void) {
//     MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
//     MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪量程设置
//     MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
//     MPU6050_setSleepEnabled(0); //进入工作状态
// 	  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
// 	  MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通关闭
// }
//
// /**************************************************************************
// Function: Initialization of DMP in mpu6050
// Input   : none
// Output  : none
// 函数功能：MPU6050内置DMP的初始化
// 入口参数：无
// 返回  值：无
// **************************************************************************/
// void DMP_Init(void)
// {
//    u8 temp[1]={0};
// 	 Flag_Show=1;
//    IIC_Simula_Read(0x68,0x75,1,temp);
// 	 printf("mpu_set_sensor complete ......\r\n");
// 	if(temp[0]!=0x68)NVIC_SystemReset();
// 	if(!mpu_init())
//   {
// 	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
// 	  	 printf("mpu_set_sensor complete ......\r\n");
// 	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
// 	  	 printf("mpu_configure_fifo complete ......\r\n");
// 	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
// 	  	 printf("mpu_set_sample_rate complete ......\r\n");
// 	  if(!dmp_load_motion_driver_firmware())
// 	  	printf("dmp_load_motion_driver_firmware complete ......\r\n");
// 	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
// 	  	 printf("dmp_set_orientation complete ......\r\n");
// 	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
// 	      DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
// 	      DMP_FEATURE_GYRO_CAL))
// 	  	 printf("dmp_enable_feature complete ......\r\n");
// 	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
// 	  	 printf("dmp_set_fifo_rate complete ......\r\n");
// 	  run_self_test();
// 		if(!mpu_set_dmp_state(1))
// 		{
// 			printf("mpu_set_dmp_state complete ......\r\n");
// 		}
//
//   }
// 	Flag_Show=0;
//
// }
// /**************************************************************************
// Function: Read the attitude information of DMP in mpu6050
// Input   : none
// Output  : none
// 函数功能：读取MPU6050内置DMP的姿态信息
// 入口参数：无
// 返回  值：无
// **************************************************************************/
// void Read_DMP(void)
// {
// 	  unsigned long sensor_timestamp;
// 		unsigned char more;
// 		long quat[4];
//
// 				dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		//读取DMP数据
// 				if (sensors & INV_WXYZ_QUAT )
// 				{
// 					 q0=quat[0] / q30;
// 					 q1=quat[1] / q30;
// 					 q2=quat[2] / q30;
// 					 q3=quat[3] / q30; 		//四元数
// 					 Roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//计算出横滚角
// 					 Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // 计算出俯仰角
// 					 Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	 //计算出偏航角
// 				}
//
// }
// /**************************************************************************
// Function: Read mpu6050 built-in temperature sensor data
// Input   : none
// Output  : Centigrade temperature
// 函数功能：读取MPU6050内置温度传感器数据
// 入口参数：无
// 返回  值：摄氏温度
// **************************************************************************/
// int Read_Temperature(void)
// {
// 	  float Temp;
// 	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
// 		if(Temp>32768) Temp-=65536;	//数据类型转换
// 		Temp=(36.53+Temp/340)*10;	  //温度放大十倍存放
// 	  return (int)Temp;
// }
// //------------------End of File----------------------------



//
// Created by lak19 on 2025/6/27.
//
/*md！md！￥…*…&%#￥%&*&&，服了！！！IIC用的是普通IO口，还要模拟时序！！！*/

#include "mpu6050.h"

static uint8_t Mpu6050Addr = MPU6050_ADDR_AD0_LOW;  //默认的为低
MPU6050DATATYPE Mpu6050_Data; //创建数据存储全局结构体
uint32_t timer = 0;       //创建时间存储变量，溢出需50天

Kalman_t KalmanPitch = {      //创建pitch角的超参数结构体
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanRoll = {      //创建roll角的超参数结构体
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

/**
 * 向MPU6050写寄存器
 * @param reg 寄存器地址
 * @param value 值
 */
static void reg_write(uint8_t reg, uint8_t value)
{
    IIC_Simula_Write(Mpu6050Addr, reg, 1, &value);
}

/**
 * 从MPU6050读取数据
 * @param reg 寄存器的值
 * @return 值
 */
static uint8_t reg_read(uint8_t reg)
{
    uint8_t regValue;
    IIC_Simula_Read(Mpu6050Addr, reg, 1,&regValue);
    return regValue;
}

/**
 * 花100ms时间初始化MPU6050，已经包含模拟IIC初始化
 */
void MPU6050_Init(void)
{
    IIC_Simulation_Init(); //模拟IIC接口的初始化

    reg_write(PWR_MGMT_1, 0x80); //MPU6050复位
    HAL_Delay(100);
    reg_write(PWR_MGMT_1, 0x00);  //退出睡眠模式
    reg_write(SMPLRT_DIV, 0x07);	    // 1Khz的速率
    reg_write(GYRO_CONFIG, 0x18); //设置量程为±2000°/s
    reg_write(ACCEL_CONFIG, 0x00); //设置加速度计的量程为±2g
}

/**
 * 读取MPU6050当前存储的未滤波数据，更新到全局结构体
 */
void MPU6050_Update(void)
{
    int16_t Accel_X_Raw = (int16_t)((reg_read(ACCEL_XOUT_H) << 8) + reg_read(ACCEL_XOUT_L)); //加速度初始数据
    int16_t Accel_Y_Raw = (int16_t)((reg_read(ACCEL_YOUT_H) << 8) + reg_read(ACCEL_YOUT_L));
    int16_t Accel_Z_Raw = (int16_t)((reg_read(ACCEL_ZOUT_H) << 8) + reg_read(ACCEL_ZOUT_L));

    int16_t Temperature_Raw = (int16_t)((reg_read(TEMP_OUT_H) << 8) + reg_read(TEMP_OUT_L)); //温度初始数据

    int16_t Gyro_X_Raw = (int16_t)((reg_read(GYRO_XOUT_H) << 8) + reg_read(GYRO_XOUT_L));  //角速度初始数据
    int16_t Gyro_Y_Raw = (int16_t)((reg_read(GYRO_YOUT_H) << 8) + reg_read(GYRO_YOUT_L));
    int16_t Gyro_Z_Raw = (int16_t)((reg_read(GYRO_ZOUT_H) << 8) + reg_read(GYRO_ZOUT_L));

    Mpu6050_Data.Accel_X = Accel_X_Raw * 6.1035e-5f; //加速度转换
    Mpu6050_Data.Accel_Y = Accel_Y_Raw * 6.1035e-5f;
    Mpu6050_Data.Accel_Z = Accel_Z_Raw * 6.1035e-5f;

    Mpu6050_Data.Temp = Temperature_Raw / 340.0f +36.53f; //温度转换

    Mpu6050_Data.Gyro_X = Gyro_X_Raw * 6.1035e-2f; //角速度转换
    Mpu6050_Data.Gyro_Y = Gyro_Y_Raw * 6.1035e-2f;
    Mpu6050_Data.Gyro_Z = Gyro_Z_Raw * 6.1035e-2f;

}

/**
 * MPU6050低通滤波函数，读取欧拉角
 */
void MPU6050_LowFilter_Euler_Angels(void)
{

    double dt = (double)(HAL_GetTick() - timer) / 1000;  // 计算时间增量，单位秒
    timer = HAL_GetTick();  // 更新计时器

    float alpha = 0.9090909; //计算得到的低通滤波系数

    MPU6050_Update();  //更新数据
    /*******角速度计和加速度计的分别姿态解算********/
    float yaw_g = Mpu6050_Data.yaw + Mpu6050_Data.Gyro_Z * dt;     //角速度计的姿态解算
    float pitch_g = Mpu6050_Data.pitch + Mpu6050_Data.Gyro_X * dt;
    float roll_g = Mpu6050_Data.roll + Mpu6050_Data.Gyro_Y * dt;
    float pitch_a = Rag_to_Deg(atan2(Mpu6050_Data.Accel_Y, Mpu6050_Data.Accel_Z)); //加速度计的姿态解算
    float roll_a = Rag_to_Deg(atan2(Mpu6050_Data.Accel_X, Mpu6050_Data.Accel_Z));
    /******************对两个姿态结算进行低通滤波***********************/
    Mpu6050_Data.yaw = yaw_g;
    Mpu6050_Data.pitch = CompleFilter(alpha, pitch_g,pitch_a);
    Mpu6050_Data.roll = CompleFilter(alpha, roll_g,roll_a);
}

/**
 * MPU6050的卡尔曼滤波姿态解算
 */
void MPU6050_Kalman_Euler_Angels(void)
{
    double dt = (double)(HAL_GetTick() - timer) / 1000;  // 获取时间差（毫秒），转换为秒
    timer = HAL_GetTick();  // 更新计时器

    MPU6050_Update();   //更新数据

    //计算偏航角 yaw
    float yaw_g = Mpu6050_Data.yaw + Mpu6050_Data.Gyro_Z * dt;
    Mpu6050_Data.yaw = yaw_g;

    // 计算滚转角 roll
    double roll;  // 用于存储计算得到的滚转角（X 轴）
    double roll_sqrt = sqrt(
        Mpu6050_Data.Accel_X * Mpu6050_Data.Accel_X + Mpu6050_Data.Accel_Z * Mpu6050_Data.Accel_Z);
    if (roll_sqrt != 0.0)
    {
        roll = Rag_to_Deg(atan(Mpu6050_Data.Accel_Y/ roll_sqrt));  // 先计算出弧度制 roll ，再弧度转换为角度值
    }
    else
    {
        roll = 0.0;
    }

    // 计算俯仰角 pitch
    double pitch = Rag_to_Deg(atan2(-Mpu6050_Data.Accel_X, Mpu6050_Data.Accel_Z));

    // 如果俯仰角度变化过快(超过90度)，防止角度跳变
    if ((pitch < -90 && Mpu6050_Data.KalmanRoll > 90) || (pitch > 90 && Mpu6050_Data.KalmanRoll < -90))
    {
        KalmanRoll.angle = pitch;
        Mpu6050_Data.KalmanRoll = pitch;
    }
    else
    {
        // 卡尔曼滤波器更新俯仰角度 Y
        Mpu6050_Data.KalmanRoll = Kalman_getAngle(&KalmanRoll, pitch, Mpu6050_Data.Gyro_Y, dt);
    }

    // 如果俯仰角绝对值超过 90 度，则反转 X 轴的陀螺仪角速度，防止符号错误
    if (fabs(Mpu6050_Data.KalmanRoll) > 90)
        Mpu6050_Data.Gyro_X = -Mpu6050_Data.Gyro_X;

    // 卡尔曼滤波器更新滚转角度 X
    Mpu6050_Data.KalmanPitch = Kalman_getAngle(&KalmanPitch, roll, Mpu6050_Data.Gyro_X, dt);
}

/**
 * 卡尔曼计算函数，套用五个公式
 * @param Kalman 卡尔曼相关系数结构体
 * @param newAngle 预测角度
 * @param newRate 角速度
 * @param dt 时间间隔
 * @return 最优估计
 */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    /*---------------------预测阶段--------------------------*/

    // 1. 预测角度
    // 角速度 = 陀螺仪角速度 - 陀螺仪偏置值 (得到无偏角速度)
    double rate = newRate - Kalman->bias;
    // 预测角度 = 前一时刻角速 + 时间间隔*角速度
    Kalman->angle += dt * rate;

    // 2. 预测协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // 预测角度协方差
    Kalman->P[0][1] -= dt * Kalman->P[1][1];  // 预测角度和偏置的协方差
    Kalman->P[1][0] -= dt * Kalman->P[1][1];  // 预测偏置和角度的协方差
    Kalman->P[1][1] += Kalman->Q_bias * dt;   // 预测偏置协方差


    /*---------------------更新阶段--------------------------*/

    // 3. 更新卡尔曼增益
    // 总误差协方差 = 预测协方差 + 测量噪声协方差
    double S = Kalman->P[0][0] + Kalman->R_measure;
    // 卡尔曼增益 K
    double K[2];
    K[0] = Kalman->P[0][0] / S;  // 角度的卡尔曼增益
    K[1] = Kalman->P[1][0] / S;  // 偏置的卡尔曼增益

    // 4. 更新角度和偏置
    // 测量残差 = 测量值 - 预测值
    double y = newAngle - Kalman->angle;
    // 根据卡尔曼增益，更新角度和偏置的估计值，修正预测阶段的误差
    Kalman->angle += K[0] * y;  // 更新角度估计。
    Kalman->bias += K[1] * y;   // 更新偏置估计

    // 5. 更新协方差矩阵 P
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;  // 更新角度协方差
    Kalman->P[0][1] -= K[0] * P01_temp;  // 更新角度和偏置的协方差
    Kalman->P[1][0] -= K[1] * P00_temp;  // 更新偏置和角度的协方差
    Kalman->P[1][1] -= K[1] * P01_temp;  // 更新偏置协方差

    // 6. 返回滤波后的最优角度值
    return Kalman->angle;
}