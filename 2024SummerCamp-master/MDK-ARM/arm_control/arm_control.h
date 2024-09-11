#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"


#define PI 3.14

extern uint8_t START_FLAG;
extern uint8_t STOP_FLAG;
extern uint8_t PUT_BALL_1;
extern uint8_t PUT_BALL_2;
extern uint8_t GET_BALL_1;
extern uint8_t GET_BALL_2;

extern uint8_t BOX;
extern uint8_t adjust;

//extern float  M5_Pos;
//extern float  M6_Pos;
//extern float  M7_Pos;

extern float M5_READY ;
extern float M6_READY ;
extern float M7_READY ;

extern float M5_READY1 ;
extern float M6_READY1 ;
extern float M7_READY1 ;

extern float M6_PUT_1;
extern float M7_PUT_1;

extern float M5_PUT_2;
extern float M6_PUT_2;
extern float M7_PUT_2;

extern float M6_GET_1;
extern float M7_GET_1;

extern float M5_GET1_2;
extern float M6_GET1_2;
extern float M7_GET1_2;

extern float M5_GET2_2;
extern float M6_GET2_2;
extern float M7_GET2_2;

extern float M5_GET3_2;
extern float M6_GET3_2;
extern float M7_GET3_2;

extern float M5_GET4_2;
extern float M6_GET4_2;
extern float M7_GET4_2;

extern float M5_GET5_2;
extern float M6_GET5_2;
extern float M7_GET5_2;

#endif
