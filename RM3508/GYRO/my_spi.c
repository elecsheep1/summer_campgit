#include "my_spi.h"

/**
 * @brief  ����SPI�ٶ�
 * @note  None
 * @param  hspi:SPI��ַ  SPI_BaudRatePrescaler:SPI��Ƶϵ��
 * @retval None
  */
void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));   //�ж���Ч��

    __HAL_SPI_DISABLE(hspi);  //�ر�SPI
    hspi->Instance->CR1 &= 0XFFC7;  //λ3-5���㣬�������ò�����
    hspi->Instance->CR1 |= SPI_BaudRatePrescaler;	 //����SPI�ٶ�
    __HAL_SPI_ENABLE(hspi);  //ʹ��SPI
}

/**
 * @brief  ��дһ���ֽ�
 * @note  None
 * @param  hspi:SPI��ַ  TxData:д����ֽ�
 * @retval Rxdata:�������ֽ�
  */
uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef* hspi, uint8_t TxData)
{
	uint8_t Rxdata;
    HAL_SPI_TransmitReceive(hspi, &TxData, &Rxdata, 1, 1000);
    return Rxdata;
}


