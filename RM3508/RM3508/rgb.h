#ifndef RGB_H
#define RGB_H

#include "stdint.h"
#include "main.h"

#define  RGB_NUM  85
#define  RGB_HIGH  70//ԭ������104������PWMռ�ձȶ�ӦWB2812��1��   150
#define  RGB_LOW  30//ԭ������52������PWMռ�ձȶ�ӦWB2812��0��   58

//RGB��ɫ���ձ�
#define Red_Color 0xFF0000
#define Green_Color 0x00FF00
#define Blue_Color 0x0000FF
#define Yellow_Color 0xFFFF00
#define Purple_Color 0xA020F0
#define Oringe_Color 0xFFA500
#define Pink_Color 0xFFC0CB
#define White_Color 0xFFFFFF
#define Black_Color 0x000000
#define NavyBlue_Color 0x000080

//�ڲ����ú���
void RGB_Cal_Color(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* ColorBuf, uint16_t Color_RGB[][3]);
void RGB_Change(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* Color_360, float Bright);//���ݽǶȱ仯��ɫ


//�ⲿ���ú���
void Test_Self_RGB(void);
void RGB_Work(void);//����rgb��
void RGB_Color_Convert(TIM_HandleTypeDef* htim, uint8_t Channel, uint32_t RGB, float Bright);//���������RGBֱ����ɫ
void RGB_Color_Convert_Pro(TIM_HandleTypeDef* htim, uint8_t Channel, float Bright);//����More_RGB�������ɫ�Ķ��RGB���ư汾
void RGB_Color_Convert_Pro_with_Light(TIM_HandleTypeDef* htim, uint8_t Channel, float *Bright);//���Կ������ȵĶ��RGB���ư汾

//�����ⲿ�����õı���
extern uint16_t RGB_Index;
extern uint16_t Color_360[RGB_NUM];  //0-360
extern uint32_t More_RGB[RGB_NUM];
extern uint8_t Speed_RGB;
extern uint32_t States_RGB[RGB_NUM];
extern float States_RGB_Light[RGB_NUM];

#endif
