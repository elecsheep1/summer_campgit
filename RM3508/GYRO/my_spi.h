#ifndef MY_SPI_H
#define MY_SPI_H

#include "spi.h"
#include "main.h"

#define SPI3_CS_GPIO_Port   GPIOB
#define SPI3_CS_Pin   GPIO_PIN_9

#define SPI1_CS_H  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define SPI1_CS_L  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define SPI2_CS_H  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
#define SPI2_CS_L  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define SPI3_CS_H  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET)
#define SPI3_CS_L  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET)

void SPI_Set_Speed(SPI_HandleTypeDef* hspi, uint8_t SPI_BaudRatePrescaler);
uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef* hspi, uint8_t TxData);

#endif
