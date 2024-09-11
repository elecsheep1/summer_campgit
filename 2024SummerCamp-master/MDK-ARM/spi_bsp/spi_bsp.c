#include "spi_bsp.h"

/*请阅读头文件中的说明，获得更方便的使用说明*/

int32_t Pos_Base[5] = {0};//累积圈数
int32_t Pos_Now[5] = {0};//现在的累积线数位置
int16_t Pos_Init[5] = {0};//初始的编码器位置（绝对值编码器掉电保存）
float The_Angle_ReadNow[5] = {0};//编码器单圈当前位置_角度
uint16_t The_Pos_ReadNow[5] = {0};//编码器单圈当前位置_线数
uint16_t tmp[5] = {0};//编码器单圈当前位置
uint16_t tmp_last[5] = {0};//编码器单圈上一次位置
uint16_t offset_counter[5] = {0,1538,0,0,0};

void Encoder_Init(void)
{
	SPI_Set_Speed(&hspi1, PCLK_64);//时钟分频可以改
	SPI_Set_Speed(&hspi2, PCLK_32);
	for(uint8_t i=0; i<50; i++)//滤波
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
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));   //判断有效性

    __HAL_SPI_DISABLE(hspi);  //关闭SPI
    hspi->Instance->CR1 &= 0XFFC7;  //位3-5清零，用来设置波特率
    hspi->Instance->CR1 |= SPI_BaudRatePrescaler;	 //设置SPI速度
    __HAL_SPI_ENABLE(hspi);  //使能SPI
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
