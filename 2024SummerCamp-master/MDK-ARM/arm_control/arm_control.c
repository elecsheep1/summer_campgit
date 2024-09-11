#include "arm_control.h"
#include "RM3508.h"
#include "can_bsp.h"
//#include "PID.h"

#include "main.h"
#include "cmsis_os.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


//机械臂控制标志位
uint8_t START_FLAG = 0;
uint8_t STOP_FLAG = 0;
uint8_t PUT_BALL_1 = 0;// 二区放球
uint8_t PUT_BALL_2 = 0;
uint8_t GET_BALL_1 = 0;
uint8_t GET_BALL_2 = 0;
uint8_t adjust = 0;
//仓内球数
uint8_t BOX = 5;
//用于机械臂位置环
//float  M5_Pos = 0;
//float  M6_Pos = 0;
//float  M7_Pos = 0;
// 预备位置
float M5_READY = -90;
float M6_READY = 60;
float M7_READY = 30;

float M5_READY1 = -90;
float M6_READY1 = 80;
float M7_READY1 = 70;
//二区放球
float M6_PUT_1 = 170;
float M7_PUT_1 = 60;
//三区放球
float M5_PUT_2 = -90;
float M6_PUT_2 = 100;
float M7_PUT_2 = 80;
//二区取球
float M6_GET_1 = 178;
float M7_GET_1 = 60;
//从背上取球
//1
float M5_GET1_2 = -180;
float M6_GET1_2 = 140;
float M7_GET1_2 = 38;
//2
float M5_GET2_2 = 75;
float M6_GET2_2 = 137;
float M7_GET2_2 = 24;
//3
float M5_GET3_2 = 124;
float M6_GET3_2 = 123;
float M7_GET3_2 = 8;
//4
float M5_GET4_2 = 144;
float M6_GET4_2 = 158;
float M7_GET4_2 = 53;
//5
float M5_GET5_2 = 106;
float M6_GET5_2 = 159;
float M7_GET5_2 = 85;
