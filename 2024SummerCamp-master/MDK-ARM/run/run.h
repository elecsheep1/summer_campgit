#ifndef __RUN_H__
#define __RUN_H__

#include "main.h"

#define FREE 0
#define BUSY 1
#define READY 2

void triangle();
void start_left();
void start_right();
void run_left(int num);
void run_right(int num);
float PID_Cal(int id, double current, double goal);
int move_from_to(double start_m[3], double target_m[3], double speed_m, double current_target_meter[2]);
int move_to(double target_m[3]);

extern double current_meter[3]; //储存当前位置
extern uint8_t arm_mode; //机器臂状态标签
extern double current_target_m_main[2]; //主巡线当前目标位置储存
extern double current_target_m[2];
extern int M5_Pos;
extern int M6_Pos;
extern int M7_Pos;

#endif