#ifndef __SPI_BSP_H__
#define __SPI_BSP_H__


/************SPI-ENCONDER*��˵��*************************/
/*
Pos_Now��ʾ�±�ŵı��������ڵ��ۻ�����λ�� ����Pos_Now[2]��ʾ2�ű������������2���Ǳ����������ID������Ϊ����������ǵڶ�������������������ǵڶ���
Encoder_Init�����п�������SPI_Set_Speed�������ò����ʣ�����1M�����ʸ�����Ҳ����PCLK_32������F4�İ��ӣ����1��3��M
��Encoder_Cal�����м����ۻ��ı�����ֵ
Ҳ����˵����Ҫ���õĺ�������Encoder_Init��Encoder_Cal�����Ҫ���ӱ������ĸ������߸ı��䲨���ʵȵȣ���д���������������Ϳ�����
����鿴Pos_Now���鼴��
�ӵ㣺
��SPI��cubemx������������úͰ����ϻ����������ô���ʿ��ܲ�һ�����԰�������Ϊ��
��ע�Ⲩ���ʵķ�Ƶ����ͬSPI�ڵ�ʱ��Դ��ͬ����ƵҲ��ͬ��������������������1M����
*/
/************SPI-ENCONDER*��˵��**************************/

#include "gpio.h"
#include "spi.h"

#define PCLK_2 0x00      //������2��Ƶ
#define PCLK_4 0x08	    //������4��Ƶ
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

uint16_t Encoder1_Read(SPI_HandleTypeDef* hspi);//��SPI1��ֵ
uint16_t Encoder2_Read(SPI_HandleTypeDef* hspi);//��SPI2��ֵ
void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler);//����SPI�Ĳ�����
void Encoder_Cal(uint8_t ID);//��Ȧ�ۻ�����
void Encoder_Init(void);//��������ʼ��

extern int32_t Pos_Base[5];//�ۻ�Ȧ��
extern int32_t Pos_Now[5];//���ڵ��ۻ�����λ��
extern int16_t Pos_Init[5];//��ʼ�ı�����λ�ã�����ֵ���������籣�棩
extern uint16_t tmp[5];//��������Ȧ��ǰλ��
extern float The_Angle_ReadNow[5];//��������Ȧ��ǰλ��_�Ƕ�
extern uint16_t The_Pos_ReadNow[5];//��������Ȧ��ǰλ��_����


#endif
