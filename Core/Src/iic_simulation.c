//
// Created by lak19 on 2025/6/27.
//

#include "iic_simulation.h"
#include "delay.h"

/**
 * 初始化模拟IIC，IIC默认使用Bx引脚
 */
void IIC_Simulation_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOB_CLK_ENABLE(); //使能 GPIOB 时钟

	GPIO_Initure.Pin=IIC_SCL_PIN|IIC_SDA_PIN;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD; //开漏模式
	GPIO_Initure.Pull=GPIO_PULLUP; //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH; //高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	IIC_SDA=1;
	IIC_SCL=1;
}

/**
 * @brief IIC的开始式
 * @return 正常返回1
 */
int IIC_Start(void)
{
	SDA_OUT();     //SDA转为输出模式
	IIC_SDA=1;
	if(!READ_SDA)return 0;
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;     //当CLK为高，SDA变化
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;     //钳住IIC总线，准备发送或接受数据
	return 1;
}

/**
 * @brief IIC的结束式
 */
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(1);
	IIC_SCL=1;
	IIC_SDA=1;
	delay_us(1);
}

/**
 * IIC等待应答信号
 * @return
 */
int IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;
	return 1;
}

/**
 * IIC应答
 */
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**
 * IIC不应答
 */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**
 * IIC发送一个字节
 * @param txd 发送的字节
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	  SDA_OUT();
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1;
			delay_us(1);
			IIC_SCL=1;
			delay_us(1);
			IIC_SCL=0;
			delay_us(1);
    }
}

/**
 * IIC读取一个位
 * @param ack 1为发送应答信号，0为不发送
 * @return 读取到的数据
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
	for(i=0;i<8;i++ )
	{
		IIC_SCL=0;
		delay_us(2);
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;
		delay_us(2);
	}
	if (ack)
		IIC_Ack();
	else
		IIC_NAck();
	return receive;
}

/**
 * IIC向设备连续写入数据
 * @param addr 设备地址（一般的，是7位二进制数）
 * @param reg 设备的寄存器地址
 * @param len 字节数
 * @param data 数据指针
 * @return 正常为0，否者为1
 */
int IIC_Simula_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
	for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * IIC从设备连续读出数据
 * @param addr 设备地址（一般的，是7位二进制数）
 * @param reg 设备寄存器地址
 * @param len 字节长度
 * @param buf 读出数据缓存
 * @return 正常为0，否则为1
 */
int IIC_Simula_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**
 * IIC读取一个字节
 * @param I2C_Addr 设备地址
 * @param addr 寄存器地址
 * @return 读到的数据
 */
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;

	IIC_Start();
	IIC_Send_Byte(I2C_Addr);
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;
	IIC_Wait_Ack();
	//IIC_Stop();
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);
  IIC_Stop();

	return res;
}

unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
	return IIC_Simula_Write(dev, reg, 1, &data);
}
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
	*data=I2C_ReadOneByte(dev, reg);
	return 1;
}

/**
 * 写入指定的位
 * @param dev 设备地址（一般的，是7位二进制数）
 * @param reg 寄存器地址
 * @param bitStart 开始的位
 * @param length 长度
 * @param data 数据指针
 * @return
 */
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**
 *
 * @param dev 设备地址
 * @param reg 寄存器地址
 * @param bitNum 第几位
 * @param data 数据本身
 * @return 0正常
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data){
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}



