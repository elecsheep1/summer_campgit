#ifndef __MECANUM_WHEEL_H__
#define __MECANUM_WHEEL_H__

#include "main.h"

/*
	在RM3508库中,RM3508_Set_Pos传入参数第三项加了一个k(速度比例),注意更改此函数
		电机序号：
			1	3
			2	4
*/

#define pi 3.1415926

/* 控制参数 */
#define CONTROL_MAX 500
#define MOTOR_MAX 	15000
#define DIS_TO_POS	8192.0/(15*pi)*19
#define DEADBAND_COEFFICIENT	1.3
#define R_WHEEL_TO_CENTER			20
#define K_SPEED								0.4

void PWM_Setup();
void input_motion(int x,int y,int r);
void set_motion(int x,int y,int r);

/**
 * @brief 归零左前电机角度
 * @note  无
 */
void zero_lf_pos(void);

/**
 * @brief 归零左后电机角度
 * @note  无
 */
void zero_lb_pos(void);

/**
 * @brief 归零右前电机角度
 * @note  无
 */
void zero_rf_pos(void);

/**
 * @brief 归零右后电机角度
 * @note  无
 */
void zero_rb_pos(void);

/**
 * @brief 设置左前电机转动线圈数
 * @param 转动角度
 * @note  变量类型long int
 */
int set_xlf(int x);

/**
 * @brief 设置左后电机转动线圈数
 * @param 转动角度
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xlb(int x);

/**
 * @brief 设置右前电机转动线圈数
 * @param 转动角度
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xrf(int x);

/**
 * @brief 设置右后电机转动线圈数
 * @param 转动角度
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xrb(int x);

/**
 * @brief 设置左前电机转速
 * @param 转动速度
 * @return	是否到达位置
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vlf(int v);

/**
 * @brief 设置左后电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vlb(int v);

/**
 * @brief 设置右前电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vrf(int v);

/**
 * @brief 设置右后电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vrb(int v);

/**
 * @brief 控制底盘移动特定距离
 * @param x方向移动距离
 * @param y方向移动距离
 * @param 移动速度
 * @return  是否到达位置
 * @note  距离单位为cm；速度范围各由-15000至15000,变量类型int
 * @note  在RM3508库中,RM3508_Set_Pos传入参数第三项加了一个k,注意更改此函数
 */
int move_motion(double x, double y, int v);

/**
 * @brief 控制底盘转动特定角度
 * @param 转动角度
 * @param 转动速度
 * @return  是否到达位置
 * @note  角度单位为rad,方向为顺时针；速度范围各由-15000至15000,变量类型int
 * @note  在RM3508库中,RM3508_Set_Pos传入参数第三项加了一个k,注意更改此函数
 */
int move_around(double rotation, int v);

#endif


