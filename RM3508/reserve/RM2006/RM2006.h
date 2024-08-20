/*********************************************************************************
*@函数功能简介：
*@	RM2006_Set_I：		RM2006电流控制
*@	RM2006_Set_Speed：	RM2006速度闭环，通过电流作为输出
*@	RM2006_Set_Pos：		RM2006位置闭环，通过速度作为输出，串级PID
*@
*@	RM2006_Get_Feedback：RM2006数据反馈获取，CAN中断中使用
*@	M_2006_Get_Torque：	RM2006转矩信息获取
*@	RM2006_Get_Speed：	速度信息获取
*@	RM2006_Get_Pos：		位置信息获取
*@	pos_rec：			位置信息更新，CAN中断中使用，以足够高的采样率解决边界问题
*@
*@	Ang2Cnt：			角度制数据转化为编码器的Cnt值
*@	Cnt2Ang：			编码器Cnt值转化为实际转过的角度
*@
*@	RM2006和2006互通的，区别只是电机减速比不同以及2006多了一个电机温度反馈
*@	因此该文件修改电机的减速比后可以直接用于2006的控制
*@  
*@ @file	:	RM2006.c
*@ @author	:	SHJ
*@ @version	:	v2.1，修复了同时控制多个电机，数据被覆写的Bug；每个电机给了一套独立的PID
*@				调参时可以对每个电机针对性的调整；将位置闭环的入口参数改为cnt值
*@ @date	:	2019-1-30
*********************************************************************************/

#ifndef __RM2006_H
#define __RM2006_H

#include "can.h"
#include "main.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx_hal_can.h"
#define RM2006_CNT_PER_ROUND (8192) //编码器线程
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

//速度PID存储
extern M2006_PID M2006_Speed_Pid[8];
//位置PID存储
extern M2006_PID M2006_Pos_Pid[8];

extern uint8_t RM2006_Feedback_Buf[8][7]; //电机反馈报文
extern int RM2006_Pos[8];
extern uint8_t RM2006_Sendbuf[8];


//电流环（在这一环向CAN发数据）
uint8_t RM2006_Set_I(int target_i,uint8_t motor_id);
//发送信号到不同CAN线上
uint8_t RM2006_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len);

//获取电机转矩信息
int RM2006_Get_Torque(uint8_t motor_id);
//获取电机速度信息
int RM2006_Get_Speed(uint8_t motor_id);
//获取电机位置信息
int RM2006_Get_Pos(uint8_t motor_id);
//获取电机温度信息
uint8_t RM2006_Temperature(uint8_t id);

//解析CAN中的信息
void RM2006_Get_Feedback(uint32_t std_id, uint8_t *data_p);
//通过速度PID设置速度
void	RM2006_Set_Speed(int goal_speed,int ID);
//通过位置PID设置位置（角度位置，是经过减速箱后的位置,PID中debug时存储的Target是cnt）
void RM2006_Set_Pos(float pos,int ID);

void RM2006_Set_Ang(float angle_t_goal,int ID);
//CAN中断中实时更新位置信息
void RM2006_Pos_Rec(uint8_t motor_id);
//角度转线数
int RM2006_Ang2Cnt(float angle, int ID);
//线数转角度
double RM2006_Cnt2Ang(int32_t cnt, int ID);
/*将2006电机的当前值设置为任意位置*/
void RM2006_Set_NowPos(uint8_t ID,int32_t Pos_Angle);

#endif
