#include "run.h"
#include "Mecanum_wheel.h"
#include <stdlib.h>
#include <math.h>
#include "mySerial.h"
#include "cmsis_os.h"

#include "RM3508.h"


M3508_PID Move_PID[3] =//8 0 0    8 0 0   
{
	{.Kp = 8000,.Ki = 1,.Kd = 0,.Max = 1000,.Min = -1000,.IntegralLimit = 150,.DeadBand = 0.01},	// x
	{.Kp = 8000,.Ki = 1,.Kd = 0,.Max = 1000,.Min = -1000,.IntegralLimit = 150,.DeadBand = 0.01},	// y
	{.Kp = 20,.Ki = 0.05,.Kd = 0,.Max = 500,.Min = -500,.IntegralLimit = 100,.DeadBand = 2},				// r
};

uint8_t arm_mode = FREE;
int M5_Pos = 0;
int M6_Pos = 0;
int M7_Pos = 0;


/**
 * @brief 跑三角
 * @note  无
 */
void triangle(){
	if(mode_flag == TEST)
	{
		//归零角度计数
		zero_all_pos();
		while(!move_motion(200, 100, 1000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(50);
		//归零角度计数
		zero_all_pos();
		while(!move_around(13,1000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(1000);
	}
	
	//归零角度计数
	zero_all_pos();
	if(mode_flag == TEST)
	{
		while(!move_motion(0, -100, 1000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(1000);
	}
	
	//归零角度计数
	zero_all_pos();
	if(mode_flag == TEST)
	{
		while(!move_motion(-200, 0, 1000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(1000);
	}
}


/**
 * @brief 跑起始点位
 * @note  无
 */
void start_left(){
		//后撤步
		zero_all_pos();
		while(!move_motion(-95, -50, 3000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(50);
}

/**
 * @brief 跑起始点位
 * @note  无
 */
void start_right(){
		//后撤步
		zero_all_pos();
		while(!move_motion(95, -50, 2000))
		{
			if(mode_flag == TYPICAL) break;
			osDelay(1);
		}
		Mec_Delay(50);
//	//向右矫正方向
//	zero_all_pos();
//	while(!move_around(25, 1000))
//	{
//		if(mode_flag == TYPICAL) break;
//		osDelay(1);
//	}
//	Mec_Delay(50);
//	//平移到球坑另一边
//	zero_all_pos();
//	while(!move_motion(380, -15, 1600))
//	{
//		if(mode_flag == TYPICAL) break;
//		osDelay(1);
//	}
//	Mec_Delay(50);
//	//向右矫正方向
//	zero_all_pos();
//	while(!move_around(30, 1000))
//	{
//		if(mode_flag == TYPICAL) break;
//		osDelay(1);
//	}
//	Mec_Delay(50);
}

/**
 * @brief 向左跑几个坑
 * @param 坑位数量
 * @note  无
 */
void run_left(int num)
{
	//左平移
	zero_all_pos();
	while(!move_motion(-25.5*num, -0, 2000))
	{
		if(mode_flag == TYPICAL) break;
		osDelay(1);
	}
	Mec_Delay(50);
	
//	//向右矫正方向
//	zero_all_pos();
//	while(!move_around(8, 1000))
//	{
//		if(mode_flag == TYPICAL) break;
//		osDelay(1);
//	}
//	Mec_Delay(50);
}

/**
 * @brief 向右跑几个坑
 * @param 坑位数量
 * @note  无
 */
void run_right(int num)
{
	//右平移
	zero_all_pos();
	while(!move_motion(25.5*num, -0, 2000))
	{
		if(mode_flag == TYPICAL) break;
		osDelay(1);
	}
	Mec_Delay(50);
	
//	//向右矫正方向
//	zero_all_pos();
//	while(!move_around(9, 1000))
//	{
//		if(mode_flag == TYPICAL) break;
//		osDelay(1);
//	}
//	Mec_Delay(50);
}

/**
 * @brief 计算PID
 * @param 0:x, 1:y, 2:r
 * @param 当前数值
 * @param 目标数值
 * @return PID计算数值
 * @note  无
 */
float PID_Cal(int id, double current, double goal){
	Move_PID[id].Target = goal;
	Move_PID[id].Speed_last = Move_PID[id].NowIS; // 微分优先PID
	Move_PID[id].NowIS = current;
	Move_PID[id].Err 	= Move_PID[id].Target -  Move_PID[id].NowIS;
	
	if(FabsD(Move_PID[id].Err) < Move_PID[id].DeadBand) Move_PID[id].Err = 0;
  Move_PID[id].Err_sum += Move_PID[id].Err; 
	Move_PID[id].Err_sum = CLAMP(Move_PID[id].Err_sum,-Move_PID[id].IntegralLimit,Move_PID[id].IntegralLimit);
	Move_PID[id].P_Out = Move_PID[id].Kp * Move_PID[id].Err;
	Move_PID[id].I_Out = Move_PID[id].Ki * Move_PID[id].Err_sum;
	Move_PID[id].D_Out = Move_PID[id].Kd * (Move_PID[id].Speed_last - Move_PID[id].NowIS); // 微分优先PID
	
	Move_PID[id].PID_Out = Move_PID[id].P_Out +Move_PID[id].I_Out + Move_PID[id].D_Out;
	Move_PID[id].PID_Out = CLAMP(Move_PID[id].PID_Out,Move_PID[id].Min,Move_PID[id].Max);
	return Move_PID[id].PID_Out;
}

/**
 * @brief 运用全场定位跑点
 * @param 起始点
 * @param 目标点
 * @param 每一帧移动多少pos
 * @param 储存沿线目标点的结构体
 * @note  无
 */
double current_meter[3];
double current_target_m_main[2];
double current_target_m[2];
int move_from_to(double start_m[3], double target_m[3], double speed_m, double current_target_meter[2]){
	
	// 计算速度角度
	double x = target_m[0] - start_m[0];
	double y = target_m[1] - start_m[1];
	double theta = atan2(y, x);
	
	// 计算增加量
	double dx = speed_m * cos(theta);
	double dy = speed_m * sin(theta);
	
	// 计算当前目标位置
	double err_x = target_m[0] - current_target_meter[0];
	double err_y = target_m[1] - current_target_meter[1];
	if(sqrt(pow(err_x, 2) + pow(err_y, 2)) > 0.01){	// 判断当前目标点是否抵达最终目标点，因为怕增加有小数误差所以设置为0.00001
		current_target_meter[0] += dx;
		current_target_meter[1] += dy;
	}
	
	double x_speed = PID_Cal(0, current_meter[0], current_target_meter[0]);	// x
	double y_speed = PID_Cal(1, current_meter[1], current_target_meter[1]);	// y
	double r_speed = PID_Cal(2, current_meter[2], current_target_meter[2]);	// r

	// 由绝对坐标系转化为小车坐标系
	double transformed_vx = x_speed * cos(current_meter[2]*pi/180) + y_speed * sin(current_meter[2]*pi/180);
	double transformed_vy = x_speed * sin(current_meter[2]*pi/180) + y_speed * cos(current_meter[2]*pi/180);

	// 判断是否到达目标
	if (fabs(current_meter[0]-target_m[0]) <= 0.02 && fabs(current_meter[1]-target_m[1]) <= 0.02 && fabs(current_meter[2]-target_m[2]) <= 5){
		set_motion(0, 0, 0);
		return 1;
	}
	else{
		// 输出小车速度
		set_motion(transformed_vx, transformed_vy, r_speed);
		return 0;
	}
}


/**
 * @brief 运用全场定位跑点
 * @param 目标点
 * @note  无
 */
int move_to(double target_m[3]){
	
	double x_speed = PID_Cal(0, current_meter[0], target_m[0]);	// x
	double y_speed = PID_Cal(1, current_meter[1], target_m[1]);	// y
	double r_speed = PID_Cal(2, current_meter[2], target_m[2]);	// r

	// 由绝对坐标系转化为小车坐标系
	double transformed_vx = x_speed * cos(current_meter[2]*pi/180) + y_speed * sin(current_meter[2]*pi/180);
	double transformed_vy = x_speed * sin(current_meter[2]*pi/180) + y_speed * cos(current_meter[2]*pi/180);

	// 判断是否到达目标
	if (fabs(current_meter[0]-target_m[0]) <= 0.02 && fabs(current_meter[1]-target_m[1]) <= 0.02 && fabs(current_meter[2]-target_m[2]) <= 5){
		set_motion(0, 0, 0);
		return 1;
	}
	else{
		// 输出小车速度
		set_motion(transformed_vx, transformed_vy, r_speed);
		return 0;
	}
}