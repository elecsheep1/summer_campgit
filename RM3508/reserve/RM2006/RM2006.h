/*********************************************************************************
*@�������ܼ�飺
*@	RM2006_Set_I��		RM2006��������
*@	RM2006_Set_Speed��	RM2006�ٶȱջ���ͨ��������Ϊ���
*@	RM2006_Set_Pos��		RM2006λ�ñջ���ͨ���ٶ���Ϊ���������PID
*@
*@	RM2006_Get_Feedback��RM2006���ݷ�����ȡ��CAN�ж���ʹ��
*@	M_2006_Get_Torque��	RM2006ת����Ϣ��ȡ
*@	RM2006_Get_Speed��	�ٶ���Ϣ��ȡ
*@	RM2006_Get_Pos��		λ����Ϣ��ȡ
*@	pos_rec��			λ����Ϣ���£�CAN�ж���ʹ�ã����㹻�ߵĲ����ʽ���߽�����
*@
*@	Ang2Cnt��			�Ƕ�������ת��Ϊ��������Cntֵ
*@	Cnt2Ang��			������Cntֵת��Ϊʵ��ת���ĽǶ�
*@
*@	RM2006��2006��ͨ�ģ�����ֻ�ǵ�����ٱȲ�ͬ�Լ�2006����һ������¶ȷ���
*@	��˸��ļ��޸ĵ���ļ��ٱȺ����ֱ������2006�Ŀ���
*@  
*@ @file	:	RM2006.c
*@ @author	:	SHJ
*@ @version	:	v2.1���޸���ͬʱ���ƶ����������ݱ���д��Bug��ÿ���������һ�׶�����PID
*@				����ʱ���Զ�ÿ���������Եĵ�������λ�ñջ�����ڲ�����Ϊcntֵ
*@ @date	:	2019-1-30
*********************************************************************************/

#ifndef __RM2006_H
#define __RM2006_H

#include "can.h"
#include "main.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx_hal_can.h"
#define RM2006_CNT_PER_ROUND (8192) //�������߳�
#define RM2006_CNT_PER_ROUND_OUT(x) (RM2006_CNT_PER_ROUND * RM2006_Reduction_Ratio[(x - 1)])
#define RM2006_CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define RM2006_ABS(x) (x >= 0 ? x : -x)

#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
	float Target;
	float NowIS;
	
	float Err;
	float Err_last;
	float Err_sum;
	
	float Kp;
	float Ki;
	float Kd;
	
	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;
	
	float DeadBand;
	float IntegralLimit;
}M2006_PID;

//�ٶ�PID�洢
extern M2006_PID M2006_Speed_Pid[8];
//λ��PID�洢
extern M2006_PID M2006_Pos_Pid[8];

extern uint8_t RM2006_Feedback_Buf[8][7]; //�����������
extern int RM2006_Pos[8];
extern uint8_t RM2006_Sendbuf[8];


//������������һ����CAN�����ݣ�
uint8_t RM2006_Set_I(int target_i,uint8_t motor_id);
//�����źŵ���ͬCAN����
uint8_t RM2006_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len);

//��ȡ���ת����Ϣ
int RM2006_Get_Torque(uint8_t motor_id);
//��ȡ����ٶ���Ϣ
int RM2006_Get_Speed(uint8_t motor_id);
//��ȡ���λ����Ϣ
int RM2006_Get_Pos(uint8_t motor_id);
//��ȡ����¶���Ϣ
uint8_t RM2006_Temperature(uint8_t id);

//����CAN�е���Ϣ
void RM2006_Get_Feedback(uint32_t std_id, uint8_t *data_p);
//ͨ���ٶ�PID�����ٶ�
void	RM2006_Set_Speed(int goal_speed,int ID);
//ͨ��λ��PID����λ�ã��Ƕ�λ�ã��Ǿ�����������λ��,PID��debugʱ�洢��Target��cnt��
void RM2006_Set_Pos(float pos,int ID);

void RM2006_Set_Ang(float angle_t_goal,int ID);
//CAN�ж���ʵʱ����λ����Ϣ
void RM2006_Pos_Rec(uint8_t motor_id);
//�Ƕ�ת����
int RM2006_Ang2Cnt(float angle, int ID);
//����ת�Ƕ�
double RM2006_Cnt2Ang(int32_t cnt, int ID);
/*��2006����ĵ�ǰֵ����Ϊ����λ��*/
void RM2006_Set_NowPos(uint8_t ID,int32_t Pos_Angle);

#endif
