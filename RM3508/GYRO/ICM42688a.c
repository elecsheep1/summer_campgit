#include "ICM42688a.h"
#include "string.h"
#include "arm_math.h"
#include "tim.h"
#include "stdio.h"
#include "my_spi.h"

ICM42688_t MyIMU1;
ICM42688_t MUI;
// ICM42688_t MyIMU2;

float ICM_ANGLE_DEATH = 0.045f; // ��ֵ����
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
 * @brief  �����ǳ�ʼ��
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Init(SPI_HandleTypeDef *hspi)
{
	uint8_t id = 0;
	memset(&MyIMU1, 0, sizeof(MyIMU1));
	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);  // ѡ���0
	ICM_Write_Reg(hspi, DEVICE_CONFIG, 0x01); // ��λ
	HAL_Delay(100);

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);  // ѡ���0
	ICM_Write_Reg(hspi, DEVICE_CONFIG, 0x00); // �����豸

	while (id != 0x47) // �ȴ��ܶ���ID
	{
		id = ICM_Read_Reg(hspi, WHO_AM_I);
	}

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x01);
	ICM_Write_Reg(hspi, INTF_CONFIG4, 0x83); // ѡ��SPI4��ģʽ

	ICM_Write_Reg(hspi, REG_BANK_SEL, 0x00);
	ICM_Write_Reg(hspi, FIFO_CONFIG, 0x40); // FIFO-streamģʽ

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0x0A); // ��λ�ŵ�
	ICM_Write_Reg(hspi, FIFO_CONFIG2, 0x00);
	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00); // �趨FIFO��ֵ����512�ֽڲ����ж�
	ICM_Write_Reg(hspi, FIFO_CONFIG3, 0x02);
	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);	 // ʹ��FIFO��ֵ�ж�
	ICM_Write_Reg(hspi, FIFO_CONFIG1, 0x77); // ʹ�ܸ߷ֱ���FIFO��gyro accel temp ��������FIFO
	ICM_Write_Reg(hspi, INT_CONFIG1, 0x00);
	ICM_Write_Reg(hspi, ACCEL_CONFIG0, 0x01); // ���ٶȼ����̡�16g��������32kHz
	ICM_Write_Reg(hspi, GYRO_CONFIG0, 0x01);  // ���������̡�2000dps��������32kHz
	ICM_Write_Reg(hspi, INTF_CONFIG0, 0xB0);
	ICM_Write_Reg(hspi, INT_CONFIG0, 0x08);
	// ��ͨ�˲���ʹ��Ĭ��ֵ������Ϊ8kHz
	// FIFO����Ĭ��Ϊ���ģʽ
	// Ĭ�Ͽ���������˲������ݲ��˲���
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC2, 0xA2);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC8, 0xBC);
	//		uint8_t RegVal = ICM_Read_Reg(hspi, GYRO_CONFIG_STATIC9);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC9, RegVal|0x20);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC10, 0x10);
	//		ICM_Write_Reg(hspi, GYRO_CONFIG_STATIC2, 0xA0);

	ICM_Write_Reg(hspi, PWR_MGMT0, 0x1F); // �������Ǻͼ��ٶȼ�
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
	/*���������ݳ�ʼ��*/
	/*��ȥ��ǰ2000�����ݣ�Ȼ��ɼ�2000���ȶ������������ݣ����¿������˲����������Э���
		�ÿ������˲��������˲��������˲�ֵ��ֱ��ƫ�ƣ��������������л��֣����������ֵ�ֱ��ƫ��*/
//	for (uint16_t i = 0; i < 2000; i++) // ȥ��ǰ500������
//	{
//		Kalman_Filter(&Gyro_K_Flt_Init, ICM_Read_FIFO(hspi, IMU));
//		HAL_Delay(0);
//	}

	for (uint16_t i = 0; i < 2000; i++) // �ɼ�ǰ2000������
	{
		// Sample_Array[i] = ICM_Read_FIFO(hspi, IMU);
		// printf("%f, %f\n", Sample_Array[i], Kalman_Filter(&Gyro_K_Flt_Init, Sample_Array[i]));

		Sample_Array[i] = Kalman_Filter(&Gyro_K_Flt_Init, ICM_Read_FIFO(hspi, IMU));
		HAL_Delay(0);
	}

	arm_mean_f32(Sample_Array, 2000, &IMU->OmigaOffset); // �������ٶȵ�ֱ��ƫ��

//	for (uint16_t i = 0; i < 1000; i++) // ȥ��ǰ500������
//	{
//		Kalman_Filter(&Gyro_K_Flt1, ICM_Read_FIFO(hspi, IMU));
//		HAL_Delay(0);
//	}

//	arm_offset_f32(Sample_Array, -IMU->OmigaOffset, Sample_Array, 2000);

//  	for (uint16_t i = 0; i < 2000; i++)
//   {
//	 	if (fabs(Sample_Array[i]) < ICM_ANGLE_DEATH) // ��������
//			Sample_Array[i] = 0.0f;

//		if (i > 0) // ����
//			Sample_Array[i] += Sample_Array[i - 1];
//   }

//   arm_mean_f32(Sample_Array, 2000, &IMU->YawOffset); // ����Z��Ƕȵ�ֱ��ƫ��
}

float SampleX_Array[2000] = {0.0f};
void ICM_DataX_Init(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
for (uint16_t i = 0; i < 2000; i++) // �ɼ�ǰ2000������
{
		
		SampleX_Array[i] = Kalman_Filter(&Gyro_K_Flt_Init, ICM_ReadX_FIFO(hspi, IMU));
		HAL_Delay(0);
}

	arm_mean_f32(SampleX_Array, 2000, &IMU->AccelXOffset); // ����X����ٶȵ�ֱ��ƫ��

}





/**
 * @brief  дICM�Ĵ���
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr, uint8_t Data)
{
	uint8_t temp = 0;

	temp = (ICM_WRITE << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // ���͵�ַ
	SPI_ReadWriteByte(hspi, Data); // ��������
	SPI3_CS_H;
}

/**
 * @brief  ��ICM�Ĵ���
 * @note  None
 * @param  None
 * @retval �Ĵ�����ֵ
 */
uint8_t ICM_Read_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr)
{
	uint8_t temp = 0;

	temp = (ICM_READ << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // ���͵�ַ
	temp = SPI_ReadWriteByte(hspi, 0x00);
	SPI3_CS_H;

	return temp;
}

/**
 * @brief  ������ICM�Ĵ���
 * @note  None
 * @param  None
 * @retval None
 */
void ICM_Read_Buf(SPI_HandleTypeDef *hspi, uint8_t *pData, uint8_t Addr, uint16_t Len)
{
	uint8_t temp = 0;

	temp = (ICM_READ << 7) | Addr;

	SPI3_CS_L;
	SPI_ReadWriteByte(hspi, temp); // ���͵�ַ
	HAL_SPI_Receive(hspi, pData, Len, 1000);
	SPI3_CS_H;
	//		for(uint16_t i = 0; i<Len; i++)  ����ʹ�ô˷���
	//		{
	//			SPI_ReadWriteByte(hspi, temp);  //���͵�ַ
	//			HAL_SPI_Receive(hspi, &pData[i], 1, 1000);
	//		}
}

/**
 * @brief  ��ȡFIFO
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
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00);  //�ر���ֵ�ж�

	//	FifoCount_H = ICM_Read_Reg(hspi, FIFO_COUNTH);  //FIFO�����ݸ���
	//	FifoCount = (FifoCount_H<<8) | ICM_Read_Reg(hspi, FIFO_COUNTL);

	ICM_Read_Buf(hspi, TempBuf, FIFO_DATA, 20);

	//	if(FifoCount >= 20)  //FIFO�������ݰ�
	//	{
	IMU->FifoHeader = TempBuf[0]; // 0x78Ϊ����ֵ

	// ���ݽ���
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

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02); // ��λFIFO
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);  //ʹ��FIFO��ֵ�ж�

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
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x00);  //�ر���ֵ�ж�

	//	FifoCount_H = ICM_Read_Reg(hspi, FIFO_COUNTH);  //FIFO�����ݸ���
	//	FifoCount = (FifoCount_H<<8) | ICM_Read_Reg(hspi, FIFO_COUNTL);

	ICM_Read_Buf(hspi, TempBuf, FIFO_DATA, 20);

	//	if(FifoCount >= 20)  //FIFO�������ݰ�
	//	{
	IMU->FifoHeader = TempBuf[0]; // 0x78Ϊ����ֵ

	// ���ݽ���
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

	ICM_Write_Reg(hspi, SIGNAL_PATH_RESET, 0X02); // ��λFIFO
	//	ICM_Write_Reg(hspi, INT_SOURCE0, 0x04);  //ʹ��FIFO��ֵ�ж�

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

	RecursiveTimes++; // ���Ƶݹ����

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
	IMU->OmigaZ = Kalman_Filter(K_Flt, ICM_Read_FIFO(hspi, IMU) - IMU->OmigaOffset); // �����������˲���ƫ�ƺ������

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->OmigaZ) < ICM_ANGLE_DEATH) // �ѽ��ٶ�Ϊ0ʱ��ֵ�����������
	{
		IMU->Sample_Array[IMU->SampleIndex] = IMU->OmigaZ; // ����ƽ�������ƫ��
		IMU->SampleIndex++;
		if (IMU->SampleIndex == SAMPLE_NUM)
			IMU->SampleIndex = 0;

		// ���㶯̬����
		arm_mean_f32(IMU->Sample_Array, SAMPLE_NUM, &IMU->OmigaOffsetTemp); // ����ƽ���˲�������Ϊ10
	}
	else
		IMU->OmigaZ -= IMU->OmigaOffsetTemp;

	if (fabs(IMU->OmigaZ) < ICM_ANGLE_DEATH) // ����
		IMU->OmigaZ = 0.0f;

	/*���½Ƕ�*/
	if (IMU->OmigaZ + IMU->LastOmigaZ > 0.0f)
		IMU->Yaw += (IMU->OmigaZ + IMU->LastOmigaZ) * 0.0001f * POSI_SCALE;
	else
		IMU->Yaw += (IMU->OmigaZ + IMU->LastOmigaZ) * 0.0001f * NEGA_SCALE;

	IMU->LastOmigaZ = IMU->OmigaZ;
}


void ICM_AccelX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	IMU->AccelX = Kalman_Filter(K_Flt, ICM_ReadX_FIFO(hspi, IMU) - IMU->AccelXOffset); // �����������˲���ƫ�ƺ������

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->AccelX) < ICM_ACCELX_DEATH) // �ѽ��ٶ�Ϊ0ʱ��ֵ�����������
	{
		IMU->SampleX_Array[IMU->SampleXIndex] = IMU->AccelX; // ����ƽ�������ƫ��
		IMU->SampleXIndex++;
		if (IMU->SampleXIndex == SAMPLE_NUM)
			IMU->SampleXIndex = 0;

		// ���㶯̬����
		arm_mean_f32(IMU->SampleX_Array, SAMPLE_NUM, &IMU->AccelXOffsetTemp); // ����ƽ���˲�������Ϊ10
	}
	else
		IMU->AccelX-= IMU->AccelXOffsetTemp;

	if (fabs(IMU->AccelX) < ICM_ACCELX_DEATH) // ����
		IMU->AccelX = 0.0f;

	/*���½Ƕ�*/
	if (IMU->AccelX + IMU->LastAccelX > 0.0f)
		IMU->VelocityX += (IMU->AccelX + IMU->LastAccelX) * 0.0001f * POSI_SCALE;
	else
		IMU->VelocityX += (IMU->AccelX + IMU->LastAccelX) * 0.0001f * NEGA_SCALE;

	IMU->LastAccelX = IMU->AccelX;

}

void ICM_VelocityX_Cal(SPI_HandleTypeDef *hspi, ICM42688_t *IMU, Kal_Filter *K_Flt)
{
	IMU->VelocityX = Kalman_Filter(K_Flt,IMU->VelocityX); // �����������˲���ƫ�ƺ������

	// if (IMU->OmigaZ > IMU->OmigaMAX)
	// 	IMU->OmigaMAX = IMU->OmigaZ;
	// else if (IMU->OmigaZ < IMU->OmigaMIN)
	// 	IMU->OmigaMIN = IMU->OmigaZ;

	if (fabs(IMU->VelocityX) < ICM_VX_DEATH) // �ѽ��ٶ�Ϊ0ʱ��ֵ�����������
	{
		IMU->SampleVX_Array[IMU->SampleVXIndex] = IMU->OmigaZ; // ����ƽ�������ƫ��
		IMU->SampleVXIndex++;
		if (IMU->SampleVXIndex == SAMPLE_NUM)
			IMU->SampleVXIndex = 0;

		// ���㶯̬����
		arm_mean_f32(IMU->SampleVX_Array, SAMPLE_NUM, &IMU->VelocityXOffsetTemp); // ����ƽ���˲�������Ϊ10
	}
	else
		IMU->VelocityX-= IMU->VelocityXOffsetTemp;

	if (fabs(IMU->VelocityX) < ICM_VX_DEATH) // ����
		IMU->VelocityX = 0.0f;

	/*���½Ƕ�*/
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


