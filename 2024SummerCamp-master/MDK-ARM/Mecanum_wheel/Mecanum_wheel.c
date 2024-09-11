#include "Mecanum_wheel.h"
#include <stdlib.h>
#include <math.h>
#include "mySerial.h"
#include "cmsis_os.h"

#include "RM3508.h"



/**
 * @brief 设置左前电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vlf(int v){
	RM3508_Set_Speed(v, 1);
}

/**
 * @brief 设置左后电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vlb(int v){
	RM3508_Set_Speed(v, 2);
}

/**
 * @brief 设置右前电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */

void set_vrf(int v){
	RM3508_Set_Speed(-v, 3);
}

/**
 * @brief 设置右后电机转速
 * @param 转动速度
 * @note  速度范围由-15000至15000,变量类型int
 */
void set_vrb(int v){
	RM3508_Set_Speed(-v, 4);
}

/**
 * @brief 归零左前电机角度
 * @note  无
 */
void zero_lf_pos()
{
	RM3508_Set_NowPos(1, 0);
	M3508_Pos_Pid[1].Pos_last = 0;
}

/**
 * @brief 归零左后电机角度
 * @note  无
 */
void zero_lb_pos()
{
	RM3508_Set_NowPos(2, 0);
	M3508_Pos_Pid[2].Pos_last = 0;
}

/**
 * @brief 归零右前电机角度
 * @note  无
 */
void zero_rf_pos()
{
	RM3508_Set_NowPos(3, 0);
	M3508_Pos_Pid[3].Pos_last = 0;
}

/**
 * @brief 归零右后电机角度
 * @note  无
 */
void zero_rb_pos()
{
	RM3508_Set_NowPos(4, 0);
	M3508_Pos_Pid[4].Pos_last = 0;
}

/**
 * @brief 归零所有电机角度
 * @note  无
 */
void zero_all_pos()
{
	zero_lf_pos();zero_lb_pos();zero_rf_pos();zero_rb_pos();
}

/**
 * @brief 设置左前电机转动线圈数
 * @param 转动角度
 * @param 速度比例
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xlf(int x){
	RM3508_Set_Pos(x, 1);
	if(labs(RM3508_Get_Pos(1) - x) < M3508_Pos_Pid[0].DeadBand * DEADBAND_COEFFICIENT)
		return 1;
	else
		return 0;
}

/**
 * @brief 设置左后电机转动线圈数
 * @param 转动角度
 * @param 速度比例
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xlb(int x){
	RM3508_Set_Pos(x, 2);
	if(labs(RM3508_Get_Pos(2) - x) < M3508_Pos_Pid[1].DeadBand * DEADBAND_COEFFICIENT)
		return 1;
	else
		return 0;
}

/**
 * @brief 设置右前电机转动线圈数
 * @param 转动角度
 * @param 速度比例
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xrf(int x){
	x = -x;
	RM3508_Set_Pos(x, 3);
	if(labs(RM3508_Get_Pos(3) - x) < M3508_Pos_Pid[2].DeadBand * DEADBAND_COEFFICIENT)
		return 1;
	else
		return 0;
}

/**
 * @brief 设置右后电机转动线圈数
 * @param 转动角度
 * @param 速度比例
 * @return	是否到达位置
 * @note  变量类型long int
 */
int set_xrb(int x){
	x = -x;
	RM3508_Set_Pos(x, 4);
	if(labs(RM3508_Get_Pos(4) - x) < M3508_Pos_Pid[3].DeadBand * DEADBAND_COEFFICIENT)
		return 1;
	else
		return 0;
}

/**
 * @brief 传入摇杆方向，设置麦克纳姆轮移动方向与速度
 * @param x摇杆数值
 * @param y摇杆数值
 * @param 顺时针旋转摇杆数值
 * @note  无
 */
void input_motion(int x,int y,int r)
{
	//将遥控速度换算成电机速度
	x = x/(double)CONTROL_MAX*(double)MOTOR_MAX * K_SPEED;
	y = y/(double)CONTROL_MAX*(double)MOTOR_MAX * K_SPEED;
	r = r/(double)CONTROL_MAX*(double)MOTOR_MAX * K_SPEED;
	
	set_motion(x, y, r);
}

/**
 * @brief 设置麦克纳姆轮移动方向与速度
 * @param x方向移动速度
 * @param y方向移动速度
 * @param 顺时针旋转速度
 * @note  速度范围各由-15000至15000,变量类型int
 */
void set_motion(int x,int y,int r){
	
	//右为x正;下为y正
	r *= 0.8;
	int vlf = x-y+r;
	int vlb = -x-y+r;
	int vrf = -x-y-r;
	int vrb = x-y-r;
	//获取最大值以限制数据最大值不超出15000
	int max;
	max = labs(vlf)>labs(vlb)?labs(vlf):labs(vlb);
	max = max>labs(vrf)?max:labs(vrf);
	max = max>labs(vrb)?max:labs(vrb);
	//最大值映射
	double k = (double)MOTOR_MAX/max;
	if(k>1) k=1.0;
	vlf *= k;vlb *= k;vrf *= k;vrb *= k;
	//速度输出
	set_vlf(vlf);set_vlb(vlb);set_vrf(vrf);set_vrb(vrb);
}

/**
 * @brief 控制底盘移动特定距离
 * @param x方向移动距离
 * @param y方向移动距离
 * @param 移动速度
 * @return  是否到达位置
 * @note  距离单位为cm；速度范围各由-15000至15000,变量类型int
 * @note  在RM3508库中,RM3508_Set_Pos传入参数第三项加了一个k,注意更改此函数
 */
int move_motion(double x, double y, int v)
{
	//将距离换算为线圈数
	x = x * DIS_TO_POS;
	y = y * DIS_TO_POS;
	//矫正误差
	x = x * 1.08;
//	//归零角度计数
//	zero_lf_pos();zero_lb_pos();zero_rf_pos();zero_rb_pos();
	//计算速度角度
	double theta = asin( y / sqrt(fabs(x*y)) );
	if(x < 0) theta = -theta;
	//线性坐标变换至45°方向(根号二在总计算中被约掉，此处原本总体乘根号二)
	double motion_rf = 1.0/2.0 * x + 1.0/2.0 * y;
	double motion_lf = -1.0/2.0 * x + 1.0/2.0 * y;
	//计算轮子需旋转角度(此处原本总体除以根号二)
	double count_lf = motion_rf;
	double count_lb = motion_lf;
	double count_rf = motion_lf;
	double count_rb = motion_rf;
	//计算轮速角度
	double alpha = asin( fabs(motion_lf) / sqrt(pow(motion_lf,2) + pow(motion_rf,2)) );
	//调整位置环速度限幅
	M3508_Pos_Pid[0].Max = v * cos(alpha);
	M3508_Pos_Pid[1].Max = v * sin(alpha);
	M3508_Pos_Pid[2].Max = v * sin(alpha);
	M3508_Pos_Pid[3].Max = v * cos(alpha);
	M3508_Pos_Pid[0].Min = -v * cos(alpha);
	M3508_Pos_Pid[1].Min = -v * sin(alpha);
	M3508_Pos_Pid[2].Min = -v * sin(alpha);
	M3508_Pos_Pid[3].Min = -v * cos(alpha);
	//位置输出
	set_xlf((int)count_lf);
	set_xlb((int)count_lb);
	set_xrf((int)count_rf);
	set_xrb((int)count_rb);
	
	//判断是否到达
	if(fabs(RM3508_Get_Pos(1) - count_lf) < M3508_Pos_Pid[0].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(2) - count_lb) < M3508_Pos_Pid[1].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(3) + count_rf) < M3508_Pos_Pid[2].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(4) + count_rb) < M3508_Pos_Pid[3].DeadBand * DEADBAND_COEFFICIENT)
	{
		set_motion(0, 0, 0);
		return 1;
	}
	else
		return 0;
}


/**
 * @brief 控制底盘转动特定角度
 * @param 转动角度
 * @param 转动速度
 * @return  是否到达位置
 * @note  角度单位为°,方向为顺时针；速度范围各由-15000至15000,变量类型int
 * @note  在RM3508库中,RM3508_Set_Pos传入参数第三项加了一个k,注意更改此函数
 */
int move_around(double rotation, int v)
{
	//将角度换算为线圈数
	double r = rotation / 360.0 * pi * R_WHEEL_TO_CENTER * DIS_TO_POS;
//	//归零角度计数
//	zero_lf_pos();zero_lb_pos();zero_rf_pos();zero_rb_pos();
	//计算轮子需旋转角度
	double count_lf = r;
	double count_lb = r;
	double count_rf = -r;
	double count_rb = -r;
	//调整位置环速度限幅		(这里有一个小bug,没有设置速度范围判断；不过走固定位置一般不用满速，暂且问题不大)
	M3508_Pos_Pid[0].Max = v;
	M3508_Pos_Pid[1].Max = v;
	M3508_Pos_Pid[2].Max = v;
	M3508_Pos_Pid[3].Max = v;
	M3508_Pos_Pid[0].Min = -v;
	M3508_Pos_Pid[1].Min = -v;
	M3508_Pos_Pid[2].Min = -v;
	M3508_Pos_Pid[3].Min = -v;
	//位置输出
	set_xlf((int)count_lf);
	set_xlb((int)count_lb);
	set_xrf((int)count_rf);
	set_xrb((int)count_rb);
	
	//判断是否到达
	if(fabs(RM3508_Get_Pos(1) - count_lf) < M3508_Pos_Pid[0].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(2) - count_lb) < M3508_Pos_Pid[1].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(3) + count_rf) < M3508_Pos_Pid[2].DeadBand * DEADBAND_COEFFICIENT &&
			fabs(RM3508_Get_Pos(4) + count_rb) < M3508_Pos_Pid[3].DeadBand * DEADBAND_COEFFICIENT)
	{
		set_motion(0, 0, 0);
		return 1;
	}
	else
		return 0;
}

/**
 * @brief 底盘停止稳定
 * @param 稳定时间（ms）
 * @note  无
 */
void Mec_Delay(int time)
{
	for(int t=0;t<time;t++)
	{
		set_motion(0, 0, 0);
		osDelay(1);
	}
}