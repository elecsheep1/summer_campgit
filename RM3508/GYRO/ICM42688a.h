#ifndef ICM42688_H
#define ICM42688_H

#include "my_spi.h"
#include "filter.h"

//#define POSI_SCALE (0.99162255f) // 正转系数
//#define NEGA_SCALE (0.98966555f) // 反转系数


// #define POSI_SCALE 0.99499036f // 正转系数
// #define NEGA_SCALE 0.99559868f // 反转系数

#define POSI_SCALE (0.99767576f) // 正转系数
#define NEGA_SCALE (0.99966605f) // 反转系数

#define SAMPLE_NUM 20
#define GYRO_LSB2DPS 131.0f
#define ACCEL_LSB2G 8192.0f

//#define ICM_ANGLE_DEATH 0.045f // 均值死区


#define ICM_WRITE 0
#define ICM_READ 1

/*ALL BANK*/
#define REG_BANK_SEL 0x76 // 寄存器块选择寄存器

/*BANK0*/
#define DEVICE_CONFIG 0x00 // 设备设置寄存器
#define WHO_AM_I 0x75	   // 设备id寄存器
#define FIFO_CONFIG 0x16   // FIFO配置寄存器f
#define INT_SOURCE0 0x65   // 中断源配置
#define FIFO_CONFIG1 0x5F  // FIFO配置
#define FIFO_CONFIG2 0x60  // FIFO配置
#define FIFO_CONFIG3 0x61  // FIFO配置
#define INT_CONFIG1 0x64
#define ACCEL_CONFIG0 0x50
#define GYRO_CONFIG0 0x4F
#define PWR_MGMT0 0x4E // 电源管理
#define FIFO_COUNTH 0x2E
#define FIFO_COUNTL 0x2F
#define FIFO_DATA 0x30
#define INTF_CONFIG0 0x4C
#define INT_CONFIG0 0x63
#define SIGNAL_PATH_RESET 0x4B
#define GYRO_DATA_Z0 0x2A
#define GYRO_DATA_Z1 0x29

/*BANK1*/
#define INTF_CONFIG4 0x7A // SPI模式选择
#define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_CONFIG_STATIC8 0x11
#define GYRO_CONFIG_STATIC9 0x12
#define GYRO_CONFIG_STATIC10 0x13

/*BANK2*/

/*BANK3*/
extern float angle_all;

typedef struct
{
	float OmigaX;
	float OmigaY;
	float OmigaZ;
	
	float LastOmigaX;
	float LastOmigaY;
	float LastOmigaZ;
	
	float LastAccelX;
	
	float Pitch;
	float Roll;
	float Yaw;
	
	float LastPitch;
	float LastRoll;
	float LastYaw;
	

	float Sample_Array[SAMPLE_NUM];
	float SampleX_Array[SAMPLE_NUM];
	float SampleVX_Array[SAMPLE_NUM];
	
	uint8_t SampleIndex;
	uint8_t SampleXIndex;
	uint8_t SampleVXIndex;
	
	
	float OmigaOffset;
	
	float OmigaMAX;
	float OmigaMIN;
	
	float OmigaOffsetTemp;
	
	float YawOffset;
	
	float AccelX;
	float AccelY;
	float AccelZ;
	
	float AccelXOffset;
	float AccelXOffsetTemp;
	
	float VelocityX;
	float VelocityY;
	
	float VelocityXOffset;
	float VelocityXOffsetTemp;
	
	float LastVelocityX;
	
	float DistanceX;
	float DistanceY;

	
	uint8_t FifoHeader;
	
} ICM42688_t;

void ICM_Init(SPI_HandleTypeDef *hspi);
void ICM_Data_Init(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt);
void ICM_DataX_Init(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt);
void ICM_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr, uint8_t Data);
uint8_t ICM_Read_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr);
void ICM_Read_Buf(SPI_HandleTypeDef *hspi, uint8_t *pData, uint8_t Addr, uint16_t Len);
float ICM_Read_FIFO(SPI_HandleTypeDef *hspi, ICM42688_t *IMU);
float ICM_ReadX_FIFO(SPI_HandleTypeDef *hspi, ICM42688_t *IMU);
float ICM_Read_Z(ICM42688_t *IMU);
float ICM_Read_AccelX(ICM42688_t *IMU);


void ICM_Yaw_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt);
void ICM_AccelX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt);
void ICM_VelocityX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt);


float ICM_Read_OmigaZ(ICM42688_t *IMU);
void Angle_Limit(float *Angle);


extern ICM42688_t MyIMU1;
// extern ICM42688_t MyIMU2;
extern Kal_Filter Gyro_K_Flt1;
// extern Kal_Filter Gyro_K_Flt2;

extern ICM42688_t MUI;


void Begin(void);



#endif
