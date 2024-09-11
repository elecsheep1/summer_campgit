#ifndef __SPI_BSP_H__
#define __SPI_BSP_H__


/************SPI-ENCONDER*库说明*************************/
/*
Pos_Now表示下标号的编码器现在的累积线数位置 例如Pos_Now[2]表示2号编码器，但这个2不是编码器自身的ID，你认为这个编码器是第二个，那这个编码器就是第二号
Encoder_Init函数中可以利用SPI_Set_Speed函数设置波特率，建议1M波特率附近，也就是PCLK_32，对于F4的板子，大概1点3几M
在Encoder_Cal函数中计算累积的编码器值
也就是说，主要调用的函数就是Encoder_Init和Encoder_Cal，如果要增加编码器的个数或者改变其波特率等等，改写这两个函数基本就可以了
对外查看Pos_Now数组即可
坑点：
①SPI在cubemx里面的引脚配置和板子上画的引脚配置大概率可能不一样，以板子配置为主
②注意波特率的分频，不同SPI口的时钟源不同，分频也不同，尽量将波特率设置在1M左右
*/
/************SPI-ENCONDER*库说明**************************/

#include "gpio.h"
#include "spi.h"

#define PCLK_2 0x00      //波特率2分频
#define PCLK_4 0x08	    //波特率4分频
#define PCLK_8 0x10      //...
#define PCLK_16 0x18		//...
#define PCLK_32 0x20		//...
#define PCLK_64 0x28		//...
#define PCLK_128 0x30	//...
#define PCLK_256 0x38	//...

#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_9

#define SPI1_CS_H  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define SPI1_CS_L  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define SPI2_CS_H  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
#define SPI2_CS_L  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)

uint16_t Encoder1_Read(SPI_HandleTypeDef* hspi);//读SPI1的值
uint16_t Encoder2_Read(SPI_HandleTypeDef* hspi);//读SPI2的值
void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler);//设置SPI的波特率
void Encoder_Cal(uint8_t ID);//多圈累积计数
void Encoder_Init(void);//编码器初始化

extern int32_t Pos_Base[5];//累积圈数
extern int32_t Pos_Now[5];//现在的累积线数位置
extern int16_t Pos_Init[5];//初始的编码器位置（绝对值编码器掉电保存）
extern uint16_t tmp[5];//编码器单圈当前位置
extern float The_Angle_ReadNow[5];//编码器单圈当前位置_角度
extern uint16_t The_Pos_ReadNow[5];//编码器单圈当前位置_线数


#endif
