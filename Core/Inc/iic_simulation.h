//
// Created by lak19 on 2025/6/27.
//

#ifndef IIC_IO_SIMULATION_H
#define IIC_IO_SIMULATION_H

#include "delay.h"

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
//计算指定寄存器和比特位对应的位带别名地址
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
//创建可直接访问的指针，实现单比特原子操作

//IO地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)

//直接操作寄存器进行读写状态切换
#define SDA_IN() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)3<<28;}

//IO 操作，这里需要更改引脚号
#define IIC_SCL PBout(14) //SCL
#define IIC_SDA PBout(15) //SDA
#define READ_SDA PBin(15) //输入 SDA
#define IIC_SCL_PIN GPIO_PIN_14
#define IIC_SDA_PIN GPIO_PIN_15

//IIC底层操作函数
int IIC_Start(void);				          //发送IIC开始信号
void IIC_Stop(void);	  			          //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);		      //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);     //IIC读取一个字节
int IIC_Wait_Ack(void); 			          //IIC等待ACK信号
void IIC_Ack(void);						      //IIC发送ACK信号
void IIC_NAck(void);					      //IIC不发送ACK信号

//主要的三个函数
void IIC_Simulation_Init(void);
int IIC_Simula_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int IIC_Simula_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

//IIC拓展功能函数
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
#endif //IIC_IO_SIMULATION_H
