#include "arm_control.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "PID.h"

#include "main.h"
#include "cmsis_os.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


//机械臂控制标志位
uint8_t PUT_BALL_1 = 0;// 二区放球
uint8_t PUT_BALL_2 = 0;
uint8_t GET_BALL_1 = 0;
uint8_t GET_BALL_2 = 0;

uint8_t BOX = 5;