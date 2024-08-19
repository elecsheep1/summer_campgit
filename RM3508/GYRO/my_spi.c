#include "my_spi.h"

/**
 * @brief  设置SPI速度
 * @note  None
 * @param  hspi:SPI地址  SPI_BaudRatePrescaler:SPI分频系数
 * @retval None
  */
void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));   //判断有效性

    __HAL_SPI_DISABLE(hspi);  //关闭SPI
    hspi->Instance->CR1 &= 0XFFC7;  //位3-5清零，用来设置波特率
    hspi->Instance->CR1 |= SPI_BaudRatePrescaler;	 //设置SPI速度
    __HAL_SPI_ENABLE(hspi);  //使能SPI
}

/**
 * @brief  读写一个字节
 * @note  None
 * @param  hspi:SPI地址  TxData:写入的字节
 * @retval Rxdata:读到的字节
  */
uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef* hspi, uint8_t TxData)
{
	uint8_t Rxdata;
    HAL_SPI_TransmitReceive(hspi, &TxData, &Rxdata, 1, 1000);
    return Rxdata;
}


