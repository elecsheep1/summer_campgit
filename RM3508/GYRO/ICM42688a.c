#include "ICM42688a.h"
#include "string.h"
#include "arm_math.h"
#include "tim.h"
#include "stdio.h"
#include "my_spi.h"

ICM42688_t MyIMU1;
ICM42688_t MUI;
// ICM42688_t MyIMU2;

float ICM_ANGLE_DEATH = 0.045f; // 均值死区
float ICM_ACCELX_DEATH = 0.05f;
float ICM_VX_DEATH = 0.05f;

float angle_all;
Kal_Filter Gyro_K_Flt1 =
	{1.0f,	   // k_flt.C_last
	 0.0f,	   // k_flt.X_last
			   //	 0.0001f, // k_flt.Q
	 0.00002f, // k_flt.Q
	 0.05f,	   // k_flt.R 4.0
	 0.0f, 0.0f, 0.0f, 0.0f};

Kal_Filter Gyro_K_Flt_Init =
	{1.0f,	  // k_flt.C_last
	 0.0f,	  // k_flt.X_last
	 0.0001f, // k_flt.Q
	 5.0f,	  // k_flt.R 4.0
	 0.0f, 0.0f, 0.0f, 0.0f};

/**
 * @brief  陀螺仪初始化
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Init(SPI_HandleTypeDef *hspi)
{
	uint8_t id = 0;
	memset(&MyIMU1, 0, sizeof(MyIMU1));
	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);  // 选择块0
	ICM_Write_Reg(hspi, DEVICE_CONFIG, 0x01); // 复位
	HAL_Delay(100);

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);  // 选择块0
	ICM_Write_Reg(hspi, DEVICE_CONFIG, 0x00); // 唤醒设备

	while (id != 0x47) // 等待能读出ID
	{
		id = ICM_Read_Reg(hspi, WHO_AM_I);
	}

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x01);
	ICM_Write_Reg(hspi, INTF_CONFIG4, 0x83); // 选择SPI4线模式

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);
	ICM_Write_Reg(hspi, FIFO_CONFIG, 0x40); // FIFO-stream模式

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0x0A); // 复位信道
	ICM_Write_Reg(hspi, FIFO_CONFIG2, 0x00);
	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00); // 设定FIFO阈值，满512字节产生中断
	ICM_Write_Reg(hspi, FIFO_CONFIG3, 0x02);
	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);	 // 使能FIFO阈值中断
	ICM_Write_Reg(hspi, FIFO_CONFIG1, 0x77); // 使能高分辨率FIFO，gyro accel temp 数据流入FIFO
	ICM_Write_Reg(hspi, INT_CONFIG1, 0x00);
	ICM_Write_Reg(hspi, ACCEL_CONFIG0, 0x01); // 加速度计量程±16g，采样率32kHz
	ICM_Write_Reg(hspi, GYRO_CONFIG0, 0x01);  // 陀螺仪量程±2000dps，采样率32kHz
	ICM_Write_Reg(hspi, INTF_CONFIG0, 0xB0);
	ICM_Write_Reg(hspi, INT_CONFIG0, 0x08);
	// 低通滤波器使用默认值，带宽为8kHz
	// FIFO计数默认为大端模式
	// 默认开启抗混叠滤波器、陷波滤波器
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC2, 0xA2);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC8, 0xBC);
	//		uint8_t RegVal = ICM_Read_Reg(hspi, GYRO_CONFIG_STATIC9);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC9, RegVal|0x20);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC10, 0x10);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC2, 0xA0);

	ICM_Write_Reg(hspi, PWR_MGMT0, 0x1F); // 打开陀螺仪和加速度计
	HAL_Delay(100);
	
//	ICM_Data_Init(hspi, &MyIMU1, &Gyro_K_Flt1);

	//	HAL_TIM_Base_Start_IT(&htim7);
	//	while(1);
}

/**
 * @brief  None
 * @note  None
 * @param  None
 * @retval None
 */
float Sample_Array[2000] = {0.0f};
void ICM_Data_Init(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	/*陀螺仪数据初始化*/
	/*先去掉前2000个数据，然后采集2000个稳定的陀螺仪数据，更新卡尔曼滤波器量测过程协方差，
		用卡尔曼滤波器进行滤波，消除滤波值的直流偏移，设置死区，进行积分，再消除积分的直流偏移*/
//	for (uint16_t i = 0; i < 2000; i++) // 去掉前500个数据
//	{
//		Kalman_Filter(&Gyro_K_Flt_Init, ICM_Read_FIFO(hspi, IMU));
//		HAL_Delay(0);
//	}

	for (uint16_t i = 0; i < 2000; i++) // 采集前2000个数据
	{
		// Sample_Array[i] = ICM_Read_FIFO(hspi, IMU);
		// printf("%f, %f\n", Sample_Array[i], Kalman_Filter(&Gyro_K_Flt_Init, Sample_Array[i]));

		Sample_Array[i] = Kalman_Filter(&Gyro_K_Flt_Init, ICM_Read_FIFO(hspi, IMU));
		HAL_Delay(0);
	}

	arm_mean_f32(Sample_Array, 2000, &IMU->OmigaOffset); // 消除角速度的直流偏移

//	for (uint16_t i = 0; i < 1000; i++) // 去掉前500个数据
//	{
//		Kalman_Filter(&Gyro_K_Flt1, ICM_Read_FIFO(hspi, IMU));
//		HAL_Delay(0);
//	}

//	arm_offset_f32(Sample_Array, -IMU->OmigaOffset, Sample_Array, 2000);

//  	for (uint16_t i = 0; i < 2000; i++)
//   {
//	 	if (fabs(Sample_Array[i]) < ICM_ANGLE_DEATH) // 设置死区
//			Sample_Array[i] = 0.0f;

//		if (i > 0) // 积分
//			Sample_Array[i] += Sample_Array[i - 1];
//   }

//   arm_mean_f32(Sample_Array, 2000, &IMU->YawOffset); // 消除Z轴角度的直流偏移
}

float SampleX_Array[2000] = {0.0f};
void ICM_DataX_Init(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
for (uint16_t i = 0; i < 2000; i++) // 采集前2000个数据
{
		
		SampleX_Array[i] = Kalman_Filter(&Gyro_K_Flt_Init, ICM_ReadX_FIFO(hspi, IMU));
		HAL_Delay(0);
}

	arm_mean_f32(SampleX_Array, 2000, &IMU->AccelXOffset); // 消除X轴加速度的直流偏移

}





/**
 * @brief  写ICM寄存器
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr, uint8_t Data)
{
	uint8_t temp = 0;

	temp = (ICM_WRITE << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // 发送地址
	SPI_ReadWriteByte(hspi, Data); // 发送数据
	SPI3_CS_H;
}

/**
 * @brief  读ICM寄存器
 * @note  None
 * @param  None
 * @retval 寄存器的值
 */
uint8_t ICM_Read_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr)
{
	uint8_t temp = 0;

	temp = (ICM_READ << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // 发送地址
	temp = SPI_ReadWriteByte(hspi, 0x00);
	SPI3_CS_H;

	return temp;
}

/**
 * @brief  连续读ICM寄存器
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Read_Buf(SPI_HandleTypeDef *hspi, uint8_t *pData, uint8_t Addr, uint16_t Len)
{
	uint8_t temp = 0;

	temp = (ICM_READ << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // 发送地址
	HAL_SPI_Receive(hspi, pData, Len, 1000);
	SPI3_CS_H;
	//		for(uint16_t i = 0; i<Len; i++)  不可使用此方法
	//		{
	//			SPI_ReadWriteByte(hspi, temp);  //发送地址
	//			HAL_SPI_Receive(hspi, &pData[i], 1, 1000);
	//		}
}

/**
 * @brief  读取FIFO
 * @note  None
 * @param  None
 * @retval None
 */

float ICM_Read_FIFO(SPI_HandleTypeDef *hspi, ICM42688_t *IMU)
{
	//	uint8_t RegVal = 0;

	//	uint8_t FifoCount_H = 0;
	//	uint16_t FifoCount = 0;
	uint8_t TempBuf[20] = {0};
	//	uint32_t AccelTemp = 0;
	uint32_t GyroTemp = 0;
	uint32_t AccelTemp = 0;

	//	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02);
	//	RegVal = ICM_Read_Reg(hspi, INT_SOURCE0);
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00);  //关闭阈值中断

	//	FifoCount_H = ICM_Read_Reg(hspi, FIFO_COUNTH);  //FIFO中数据个数
	//	FifoCount = (FifoCount_H<<8) | ICM_Read_Reg(hspi, FIFO_COUNTL);

	ICM_Read_Buf(hspi, TempBuf, FIFO_DATA, 20);

	//	if(FifoCount >= 20)  //FIFO中有数据包
	//	{
	IMU->FifoHeader = TempBuf[0]; // 0x78为正常值

	// 数据解析
	AccelTemp = ((TempBuf[17] & 0xC0) >> 6 | (TempBuf[2] << 2) | (TempBuf[1] << 10)) << 14;
	IMU->AccelX = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;
	AccelTemp = ((TempBuf[18] & 0xC0) >> 6 | (TempBuf[4] << 2) | (TempBuf[3] << 10)) << 14;
	IMU->AccelY = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;
	AccelTemp = ((TempBuf[19] & 0xC0) >> 6 | (TempBuf[6] << 2) | (TempBuf[5] << 10)) << 14;
	IMU->AccelZ = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;

//	GyroTemp = (((TempBuf[17] & 0x0F) | (TempBuf[8] << 4) | (TempBuf[7] << 12)) >> 1) << 13;
//	IMU->OmigaX = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
//	GyroTemp = (((TempBuf[18] & 0x0F) | (TempBuf[10] << 4) | (TempBuf[9] << 12)) >> 1) << 13;
//	IMU->OmigaY = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
	GyroTemp = (((TempBuf[19] & 0x0F) | (TempBuf[12] << 4) | (TempBuf[11] << 12)) >> 1) << 13;
	IMU->OmigaZ = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
	//	}

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02); // 复位FIFO
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);  //使能FIFO阈值中断

	return IMU->OmigaZ;
}
float ICM_ReadX_FIFO(SPI_HandleTypeDef *hspi, ICM42688_t *IMU)
{
	//	uint8_t RegVal = 0;

	//	uint8_t FifoCount_H = 0;
	//	uint16_t FifoCount = 0;
	uint8_t TempBuf[20] = {0};
	//	uint32_t AccelTemp = 0;
	uint32_t GyroTemp = 0;
	uint32_t AccelTemp = 0;

	//	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02);
	//	RegVal = ICM_Read_Reg(hspi, INT_SOURCE0);
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00);  //关闭阈值中断

	//	FifoCount_H = ICM_Read_Reg(hspi, FIFO_COUNTH);  //FIFO中数据个数
	//	FifoCount = (FifoCount_H<<8) | ICM_Read_Reg(hspi, FIFO_COUNTL);

	ICM_Read_Buf(hspi, TempBuf, FIFO_DATA, 20);

	//	if(FifoCount >= 20)  //FIFO中有数据包
	//	{
	IMU->FifoHeader = TempBuf[0]; // 0x78为正常值

	// 数据解析
	AccelTemp = ((TempBuf[17] & 0xC0) >> 6 | (TempBuf[2] << 2) | (TempBuf[1] << 10)) << 14;
	IMU->AccelX = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;
	AccelTemp = ((TempBuf[18] & 0xC0) >> 6 | (TempBuf[4] << 2) | (TempBuf[3] << 10)) << 14;
	IMU->AccelY = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;
	AccelTemp = ((TempBuf[19] & 0xC0) >> 6 | (TempBuf[6] << 2) | (TempBuf[5] << 10)) << 14;
	IMU->AccelZ = *(int32_t *)(&AccelTemp) / ACCEL_LSB2G * 9.8f / 16384.0f;

//	GyroTemp = (((TempBuf[17] & 0x0F) | (TempBuf[8] << 4) | (TempBuf[7] << 12)) >> 1) << 13;
//	IMU->OmigaX = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
//	GyroTemp = (((TempBuf[18] & 0x0F) | (TempBuf[10] << 4) | (TempBuf[9] << 12)) >> 1) << 13;
//	IMU->OmigaY = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
	GyroTemp = (((TempBuf[19] & 0x0F) | (TempBuf[12] << 4) | (TempBuf[11] << 12)) >> 1) << 13;
	IMU->OmigaZ = *(int32_t *)(&GyroTemp) / GYRO_LSB2DPS / 8192.0f;
	//	}

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02); // 复位FIFO
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);  //使能FIFO阈值中断

	return IMU->AccelX;
}









float ICM_Read_Z(ICM42688_t *IMU)
{
	return IMU->Yaw;
}

float ICM_Read_AccelX(ICM42688_t *IMU)
{
  return IMU->AccelX;
}

float ICM_Read_OmigaZ(ICM42688_t *IMU)
{
	return IMU->OmigaZ;
}



void Angle_Limit(float *Angle)
{
	static uint8_t RecursiveTimes = 0;

	RecursiveTimes++; // 限制递归次数

	if (RecursiveTimes < 100)
	{
		if (*Angle > 360.0f)
		{

			*Angle -= 360.0f;
			Angle_Limit(Angle);
		}
		else if (*Angle < 0.0f)
		{

			*Angle += 360.0f;
			Angle_Limit(Angle);
		}
	}

	RecursiveTimes--;
}








/**
 * @brief  None
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Yaw_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	IMU->OmigaZ = Kalman_Filter(K_Flt, ICM_Read_FIFO(hspi, IMU) - IMU->OmigaOffset); // 读出卡尔曼滤波减偏移后的数据

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->OmigaZ) < ICM_ANGLE_DEATH) // 把角速度为0时的值存入采样数组
	{
		IMU->Sample_Array[IMU->SampleIndex] = IMU->OmigaZ; // 滑动平均求零点偏移
		IMU->SampleIndex++;
		if (IMU->SampleIndex == SAMPLE_NUM)
			IMU->SampleIndex = 0;

		// 计算动态补偿
		arm_mean_f32(IMU->Sample_Array, SAMPLE_NUM, &IMU->OmigaOffsetTemp); // 滑动平均滤波，长度为10
	}
	else
		IMU->OmigaZ -= IMU->OmigaOffsetTemp;

	if (fabs(IMU->OmigaZ) < ICM_ANGLE_DEATH) // 死区
		IMU->OmigaZ = 0.0f;

	/*更新角度*/
	if (IMU->OmigaZ + IMU->LastOmigaZ > 0.0f)
		IMU->Yaw += (IMU->OmigaZ + IMU->LastOmigaZ) * 0.0001f * POSI_SCALE;
	else
		IMU->Yaw += (IMU->OmigaZ + IMU->LastOmigaZ) * 0.0001f * NEGA_SCALE;

	IMU->LastOmigaZ = IMU->OmigaZ;
}


void ICM_AccelX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	IMU->AccelX = Kalman_Filter(K_Flt, ICM_ReadX_FIFO(hspi, IMU) - IMU->AccelXOffset); // 读出卡尔曼滤波减偏移后的数据

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->AccelX) < ICM_ACCELX_DEATH) // 把角速度为0时的值存入采样数组
	{
		IMU->SampleX_Array[IMU->SampleXIndex] = IMU->AccelX; // 滑动平均求零点偏移
		IMU->SampleXIndex++;
		if (IMU->SampleXIndex == SAMPLE_NUM)
			IMU->SampleXIndex = 0;

		// 计算动态补偿
		arm_mean_f32(IMU->SampleX_Array, SAMPLE_NUM, &IMU->AccelXOffsetTemp); // 滑动平均滤波，长度为10
	}
	else
		IMU->AccelX-= IMU->AccelXOffsetTemp;

	if (fabs(IMU->AccelX) < ICM_ACCELX_DEATH) // 死区
		IMU->AccelX = 0.0f;

	/*更新角度*/
	if (IMU->AccelX + IMU->LastAccelX > 0.0f)
		IMU->VelocityX += (IMU->AccelX + IMU->LastAccelX) * 0.0001f * POSI_SCALE;
	else
		IMU->VelocityX += (IMU->AccelX + IMU->LastAccelX) * 0.0001f * NEGA_SCALE;

	IMU->LastAccelX = IMU->AccelX;

}

void ICM_VelocityX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	IMU->VelocityX = Kalman_Filter(K_Flt,IMU->VelocityX); // 读出卡尔曼滤波减偏移后的数据

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->VelocityX) < ICM_VX_DEATH) // 把角速度为0时的值存入采样数组
	{
		IMU->SampleVX_Array[IMU->SampleVXIndex] = IMU->OmigaZ; // 滑动平均求零点偏移
		IMU->SampleVXIndex++;
		if (IMU->SampleVXIndex == SAMPLE_NUM)
			IMU->SampleVXIndex = 0;

		// 计算动态补偿
		arm_mean_f32(IMU->SampleVX_Array, SAMPLE_NUM, &IMU->VelocityXOffsetTemp); // 滑动平均滤波，长度为10
	}
	else
		IMU->VelocityX-= IMU->VelocityXOffsetTemp;

	if (fabs(IMU->VelocityX) < ICM_VX_DEATH) // 死区
		IMU->VelocityX = 0.0f;

	/*更新角度*/
	if (IMU->VelocityX + IMU->LastVelocityX > 0.0f)
		IMU->DistanceX+= (IMU->VelocityX + IMU->LastVelocityX) * 0.0001f * POSI_SCALE;
	else
		IMU->DistanceX+= (IMU->VelocityX + IMU->LastVelocityX) * 0.0001f * NEGA_SCALE;

	IMU->LastVelocityX = IMU->VelocityX;

}


void Begin()
{
ICM_Init(&hspi2);
ICM_Data_Init(&hspi2,&MUI,&Gyro_K_Flt1);
ICM_DataX_Init(&hspi2,&MUI,&Gyro_K_Flt1);
}


