#ifndef __MECANUM_WHEEL_H__
#define __MECANUM_WHEEL_H__

#include "main.h"

/*
	��RM3508����,RM3508_Set_Pos����������������һ��k(�ٶȱ���),ע����Ĵ˺���
		�����ţ�
			1	3
			2	4
*/

#define pi 3.1415926

/* ���Ʋ��� */
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
 * @brief ������ǰ����Ƕ�
 * @note  ��
 */
void zero_lf_pos(void);

/**
 * @brief ����������Ƕ�
 * @note  ��
 */
void zero_lb_pos(void);

/**
 * @brief ������ǰ����Ƕ�
 * @note  ��
 */
void zero_rf_pos(void);

/**
 * @brief �����Һ����Ƕ�
 * @note  ��
 */
void zero_rb_pos(void);

/**
 * @brief ������ǰ���ת����Ȧ��
 * @param ת���Ƕ�
 * @note  ��������long int
 */
int set_xlf(int x);

/**
 * @brief ���������ת����Ȧ��
 * @param ת���Ƕ�
 * @return	�Ƿ񵽴�λ��
 * @note  ��������long int
 */
int set_xlb(int x);

/**
 * @brief ������ǰ���ת����Ȧ��
 * @param ת���Ƕ�
 * @return	�Ƿ񵽴�λ��
 * @note  ��������long int
 */
int set_xrf(int x);

/**
 * @brief �����Һ���ת����Ȧ��
 * @param ת���Ƕ�
 * @return	�Ƿ񵽴�λ��
 * @note  ��������long int
 */
int set_xrb(int x);

/**
 * @brief ������ǰ���ת��
 * @param ת���ٶ�
 * @return	�Ƿ񵽴�λ��
 * @note  �ٶȷ�Χ��-15000��15000,��������int
 */
void set_vlf(int v);

/**
 * @brief ���������ת��
 * @param ת���ٶ�
 * @note  �ٶȷ�Χ��-15000��15000,��������int
 */
void set_vlb(int v);

/**
 * @brief ������ǰ���ת��
 * @param ת���ٶ�
 * @note  �ٶȷ�Χ��-15000��15000,��������int
 */
void set_vrf(int v);

/**
 * @brief �����Һ���ת��
 * @param ת���ٶ�
 * @note  �ٶȷ�Χ��-15000��15000,��������int
 */
void set_vrb(int v);

/**
 * @brief ���Ƶ����ƶ��ض�����
 * @param x�����ƶ�����
 * @param y�����ƶ�����
 * @param �ƶ��ٶ�
 * @return  �Ƿ񵽴�λ��
 * @note  ���뵥λΪcm���ٶȷ�Χ����-15000��15000,��������int
 * @note  ��RM3508����,RM3508_Set_Pos����������������һ��k,ע����Ĵ˺���
 */
int move_motion(double x, double y, int v);

/**
 * @brief ���Ƶ���ת���ض��Ƕ�
 * @param ת���Ƕ�
 * @param ת���ٶ�
 * @return  �Ƿ񵽴�λ��
 * @note  �Ƕȵ�λΪrad,����Ϊ˳ʱ�룻�ٶȷ�Χ����-15000��15000,��������int
 * @note  ��RM3508����,RM3508_Set_Pos����������������һ��k,ע����Ĵ˺���
 */
int move_around(double rotation, int v);

#endif


