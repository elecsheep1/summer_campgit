#include "spi_bsp.h"

/*���Ķ�ͷ�ļ��е�˵������ø������ʹ��˵��*/

int32_t Pos_Base[5] = {0};//�ۻ�Ȧ��
int32_t Pos_Now[5] = {0};//���ڵ��ۻ�����λ��
int16_t Pos_Init[5] = {0};//��ʼ�ı�����λ�ã�����ֵ���������籣�棩
float The_Angle_ReadNow[5] = {0};//��������Ȧ��ǰλ��_�Ƕ�
uint16_t The_Pos_ReadNow[5] = {0};//��������Ȧ��ǰλ��_����
uint16_t tmp[5] = {0};//��������Ȧ��ǰλ��
uint16_t tmp_last[5] = {0};//��������Ȧ��һ��λ��
uint16_t offset_counter[5] = {0,1538,0,0,0};

void Encoder_Init(void)
{
	SPI_Set_Speed(&hspi1, PCLK_64);//ʱ�ӷ�Ƶ���Ը�
	SPI_Set_Speed(&hspi2, PCLK_32);
	for(uint8_t i=0; i<50; i++)//�˲�
	{
		Encoder1_Read(&hspi1);
		Encoder2_Read(&hspi2);
		HAL_Delay(0);
	}
	Pos_Init[1] = Encoder1_Read(&hspi1) + offset_counter[1];
	tmp_last[1] = Pos_Init[1];
	Pos_Init[2] = Encoder2_Read(&hspi2) + offset_counter[2];
	tmp_last[2] = Pos_Init[2];
}
void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));   //�ж���Ч��

    __HAL_SPI_DISABLE(hspi);  //�ر�SPI
    hspi->Instance->CR1 &= 0XFFC7;  //λ3-5���㣬�������ò�����
    hspi->Instance->CR1 |= SPI_BaudRatePrescaler;	 //����SPI�ٶ�
    __HAL_SPI_ENABLE(hspi);  //ʹ��SPI
}
uint16_t Encoder1_Read(SPI_HandleTypeDef* hspi)
{
	static uint16_t PreData = 0;
	uint16_t RawData = 0;
	uint16_t Data = 0;
	
	SPI1_CS_L;
	HAL_SPI_Receive(hspi, (uint8_t*)&RawData, 1, 10);
	SPI1_CS_H;
	
	Data = (RawData>>3)&0xfff;
	if(RawData>>15 == 1)  
		Data = PreData;
	PreData = Data;
	
	return Data;
}
uint16_t Encoder2_Read(SPI_HandleTypeDef* hspi)
{
	static uint16_t PreData = 0;
	uint16_t RawData = 0;
	uint16_t Data = 0;
	
	SPI2_CS_L;
	HAL_SPI_Receive(hspi, (uint8_t*)&RawData, 1, 10);
	SPI2_CS_H;
	
	Data = (RawData>>3)&0xfff;
	if(RawData>>15 == 1)  
		Data = PreData;
	PreData = Data;
	
	return Data;
}
void Encoder_Cal(uint8_t ID)
{
	if(ID == 1)
	{
		tmp[1] = Encoder1_Read(&hspi1) + offset_counter[1];
		if(tmp[1] - tmp_last[1] > 2048)  
		Pos_Base[1]-=4096;
		else if(tmp[1] - tmp_last[1] < -2048)  
		Pos_Base[1]+=4096;
		Pos_Now[1] = Pos_Base[1] + tmp[1];// - Pos_Init[1];
		tmp_last[1] = tmp[1];
		The_Pos_ReadNow[1] = tmp[1];
		The_Angle_ReadNow[1] = The_Pos_ReadNow[1] * 360.0f / 4096.0f;
	}
	if(ID == 2)
	{
		tmp[2] = Encoder2_Read(&hspi2) + offset_counter[2];
		if(tmp[2] - tmp_last[2] > 2048)  
		Pos_Base[2]-=4096;
		else if(tmp[2] - tmp_last[2] < -2048)  
		Pos_Base[2]+=4096;
		Pos_Now[2] = Pos_Base[2] + tmp[2];// - Pos_Init[2];
		tmp_last[2] = tmp[2];
		The_Pos_ReadNow[2] = tmp[2];
		The_Angle_ReadNow[2] = The_Pos_ReadNow[2] * 360.0f / 4096.0f;
	}
}
