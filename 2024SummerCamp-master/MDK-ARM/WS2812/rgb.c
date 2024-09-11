#include "rgb.h"
#include "dma.h"
#include "tim.h"
#include "string.h"

/*********************************************************************************
  DMA����ʱ����Ҫ���óɴ��ڴ浽����,Priority����ΪHigh
  ��ʱ��������жϺ�DMA���ж��ƺ�����ͬʱ��
*********************************************************************************/

uint16_t RGB_ResetBuf[230] = {0}; //��λ�������飬DMA���ִ���������uint16_t
uint16_t ColorBuf_0[25] = {0}; //��������BUF0
uint16_t ColorBuf_1[25] = {0}; //��������BUF1
uint16_t Color_360[RGB_NUM] = {0};  //�Ƕ���Ϣ0-360
uint16_t Color_RGB[RGB_NUM][3] = {0};  //0-255 ��RGB��Ϣ������ 0-R 1-G 2-B
uint8_t RGB_TX_Flag = 0; //��������BUF���ָ����Ϣ
uint8_t RGB_TX_Finish_Flag = 1; //��ʼʱ���������һ�η��ͣ�Ҫͨ�����͸�λ�������RGB��״̬��
uint16_t RGB_Index = 0; //RGB�Ƽ���ָ��
uint32_t More_RGB[RGB_NUM] = {0};
uint8_t Speed_RGB = 0;
uint32_t States_RGB[RGB_NUM] = {0};
float States_RGB_Light[RGB_NUM] = {0};

/*********************************************************************************
  *@  name      : RGB_Cal_Color
  *@  function  : �����RGB��Ϣ��Ӧ�ķ�������
  *@  input     : ʱ��, ͨ��, ҪDMA���͵�����, �洢RGB��Ϣ������
  *@  output    : None
*********************************************************************************/
void RGB_Cal_Color(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* ColorBuf, uint16_t Color_RGB[][3])
{
	for(int8_t i = 7; i>=0; i--) //����R����Ϣ�ķ��ͣ���λ�Ƚ��գ����Ը�λҪ���ڷ���BUF�ĵ�λ
			ColorBuf[7-i] = ((Color_RGB[RGB_Index][1]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	
	for(int8_t i = 7; i>=0; i--) //����B����Ϣ�ķ��ͣ���λ�Ƚ��գ����Ը�λҪ���ڷ���BUF�ĵ�λ
			ColorBuf[15-i] = ((Color_RGB[RGB_Index][0]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	
	for(int8_t i = 7; i>=0; i--) //����G����Ϣ�ķ��ͣ���λ�Ƚ��գ����Ը�λҪ���ڷ���BUF�ĵ�λ
			ColorBuf[23-i] = ((Color_RGB[RGB_Index][2]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
} 
/*********************************************************************************
  *@  name      : RGB_Color_Convert
  *@  function  : ���������RGB��������Ϣ��ֱ�ӿ���RGB�Ʒ���
  *@  input     : ʱ��, ͨ��, RGB��Ϣ, ����
  *@  output    : None
*********************************************************************************/
void RGB_Color_Convert(TIM_HandleTypeDef* htim, uint8_t Channel, uint32_t RGB, float Bright)
{
	uint16_t R;
	uint16_t G;
	uint16_t B;
	if(RGB_TX_Finish_Flag == 1)
	{
		B = (uint16_t)(RGB&0x0000FF);
		G = (uint16_t)((RGB>>8)&0x00FF);
		R = (uint16_t)((RGB>>16)&0x00FF);
		for(uint16_t i=0; i<RGB_NUM; i++)
		{
			Color_RGB[i][0]=R*Bright;
			Color_RGB[i][1]=G*Bright;
			Color_RGB[i][2]=B*Bright;
		}
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //��ʼ����
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //��ʼ����BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //��ʼ����BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Color_Convert_Pro
  *@  function  : ���������RGB��������Ϣ��ֱ�ӿ���RGB�Ʒ��⣬Ϊ����Ʋ�ͬ��ɫ����Ƶİ汾
  *@  input     : ʱ��, ͨ��, RGB��Ϣ, ����
  *@  output    : None
*********************************************************************************/
void RGB_Color_Convert_Pro(TIM_HandleTypeDef* htim, uint8_t Channel, float Bright)
{
	uint16_t R;
	uint16_t G;
	uint16_t B;
	if(RGB_TX_Finish_Flag == 1)
	{
		for(uint16_t i=0; i<RGB_NUM; i++)
		{
			B = (uint16_t)(More_RGB[i]&0x0000FF);
			G = (uint16_t)((More_RGB[i]>>8)&0x00FF);
			R = (uint16_t)((More_RGB[i]>>16)&0x00FF);
			Color_RGB[i][0]=R*Bright;
			Color_RGB[i][1]=G*Bright;
			Color_RGB[i][2]=B*Bright;
		}
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //��ʼ����
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //��ʼ����BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //��ʼ����BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Color_Convert_Pro_with_Light
  *@  function  : ���������RGB��������Ϣ��ֱ�ӿ���RGB�Ʒ��⣬Ϊ����Ʋ�ͬ��ɫ�Լ����ȶ���Ƶİ汾
  *@  input     : ʱ��, ͨ��, RGB��Ϣ, ����
  *@  output    : None
*********************************************************************************/
void RGB_Color_Convert_Pro_with_Light(TIM_HandleTypeDef* htim, uint8_t Channel, float *Bright)
{
	uint16_t R;
	uint16_t G;
	uint16_t B;
	if(RGB_TX_Finish_Flag == 1)
	{
		for(uint16_t i=0; i<RGB_NUM; i++)
		{
			B = (uint16_t)(More_RGB[i]&0x0000FF);
			G = (uint16_t)((More_RGB[i]>>8)&0x00FF);
			R = (uint16_t)((More_RGB[i]>>16)&0x00FF);
			Color_RGB[i][0]=R*Bright[i];
			Color_RGB[i][1]=G*Bright[i];
			Color_RGB[i][2]=B*Bright[i];
		}
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //��ʼ����
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //��ʼ����BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //��ʼ����BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Change
  *@  function  : ��������ĽǶȺ�����������RGB�Ʒ�����ͬ��ɫ
  *@  input     : ʱ��, ͨ��, �Ƕȣ���ͬ�ǶȲ�ͬ��ɫ��Ӧ���ǽ��������Ӧ�õĺ�����, ���ȣ��пռ�����ٶȾ�������  Bright 0-1��
  *@  output    : None
*********************************************************************************/
void RGB_Change(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* Color_360, float Bright)
{
	float Scale = 0.0f;

	if(RGB_TX_Finish_Flag == 1)  //��һ��DMA������ɺ󣬲ſɿ�ʼ��һ��DMA����
	{
		/*360��ӳ�䵽RGB*/
		for(uint16_t i=0; i<RGB_NUM; i++)
		{
			if(Color_360[i]<60)//Red->Green,ƫRed
			{
				Scale = (float)Color_360[i]/60;
				Color_RGB[i][0] = 255*Bright;
				Color_RGB[i][1] = 255*Scale*Bright;
				Color_RGB[i][2] = 0;
			}
			else if(Color_360[i]>=60 && Color_360[i]<120)//Red->Green,ƫGreen
			{
				Scale = (float)(Color_360[i]-60)/60;
				Color_RGB[i][0] = 255*(1-Scale)*Bright;
				Color_RGB[i][1] = 255*Bright;
				Color_RGB[i][2] = 0;
			}
			else if(Color_360[i]>=120 && Color_360[i]<180)//Green->Blue,ƫGreen
			{
				Scale = (float)(Color_360[i]-120)/60;
				Color_RGB[i][0] = 0;
				Color_RGB[i][1] = 255*Bright;
				Color_RGB[i][2] = 255*Scale*Bright;
			}
			else if(Color_360[i]>=180 && Color_360[i]<240)//Green->Blue,ƫBlue
			{
				Scale = (float)(Color_360[i]-180)/60;
				Color_RGB[i][0] = 0;
				Color_RGB[i][1] = 255*(1-Scale)*Bright;
				Color_RGB[i][2] = 255*Bright;
			}
			else if(Color_360[i]>=240 && Color_360[i]<300)//Blue->Red,ƫBlue
			{
				Scale = (float)(Color_360[i]-240)/60;
				Color_RGB[i][0] = 255*Scale*Bright;
				Color_RGB[i][1] = 0;
				Color_RGB[i][2] = 255*Bright;
			}
			else if(Color_360[i]>=300 && Color_360[i]<360)//Blue->Red,ƫRed
			{
				Scale = (float)(Color_360[i]-300)/60;
				Color_RGB[i][0] = 255*Bright;
				Color_RGB[i][1] = 0;
				Color_RGB[i][2] = 255*(1-Scale)*Bright;
			}
		}
		
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //���͸�λ�źŴӶ�ʹRGB��״̬����
		RGB_TX_Finish_Flag = 0; //��һ�η���
		RGB_Index = 0; //��һ�μ���
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //��ʼ����BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //��ʼ����BUF1
	}
}
/*������������жϻص�����*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3); //�ر�ͨ��
//	if(RGB_TX_Finish_Flag==0)
//	{
//		RGB_TX_Finish_Flag=1;
//		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_0, 25); //ֱ��ֻ��һ��BUF���鷢�ͣ�����Ҳ���������
//	}
	if(RGB_TX_Flag==0 && RGB_TX_Finish_Flag==0) //�������δ��ɣ���Ҫ���͵���BUF0
	{
		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_0, 25); //ÿ��RGB������24bits,��25bitΪ���ֹ��������
		//�ı�CCRx��ֵ�Ϳ��Ըı�PWM�������ռ�ձȡ��ı�ARR��ֵ���Ϳ��Ըı�PWM�������Ƶ�ʣ������PWM�����ԭ��
		//HAL_TIM_PWM_Start_DMA����������ÿ����PWM��������ͻ��Զ����Ƚ�ֵ���ó�DMA������������
		//Ҫ��CubeMx��������CH Polarity������ԣ�������ֵС��CCRxʱ���0/1��
		if(RGB_Index == RGB_NUM-1)  RGB_TX_Finish_Flag = 1; //�����һ��RGB����Ϣ�������ˣ�������ɱ�־����1��ע��Index�Ǵ�0��ʼ������
		RGB_TX_Flag = 1; //��һ��Ҫ����BUF1
		if(RGB_TX_Finish_Flag==0) //���û�з��ͽ���
		{
			RGB_Index++; //������һ��RGB�Ƶ���Ϣ
			RGB_Cal_Color(htim, TIM_CHANNEL_3, ColorBuf_1, Color_RGB); //����BUF1��BUF0��BUF1Ҫ�������ͣ���Ȼ�ڷ�BUF0�Ĺ������ָ���BUF0��ֵ������������
		}
	}
	else if(RGB_TX_Flag==1 && RGB_TX_Finish_Flag==0) //�������δ��ɣ���Ҫ���͵���BUF1
	{
		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_1, 25);
		if(RGB_Index == RGB_NUM-1)  RGB_TX_Finish_Flag = 1; //�����һ��RGB����Ϣ���������ˣ�������ɱ�־����1
		RGB_TX_Flag = 0; //��һ��Ҫ����BUF0
		if(RGB_TX_Finish_Flag==0)  //���û�з�����
		{
			RGB_Index++; //������һ��RGB�Ƶ���Ϣ
			RGB_Cal_Color(htim, TIM_CHANNEL_3, ColorBuf_0, Color_RGB); //����BUF0��BUF0��BUF1Ҫ�������ͣ���Ȼ�ڷ�BUF1�Ĺ������ָ���BUF1��ֵ������������
		}
	}
	
}

///** 
//  *@time          :8.6
//  *@name          :RGB_Work
//  *@brief         :����RGB�� //�����Լ��������дRGB�ƵĿ���
//  *@param[in]     :null
//  *@param[out]    :null
//  *@return        :null
//**/
//__weak void RGB_Work(void)
//{
//	  /*uint16_t rgb_init=0;
//      for(rgb_init = 0; rgb_init <=360 ; rgb_init ++ )
//	  {
//		RGB_Change(&htim8, TIM_CHANNEL_3, &rgb_init, 0.02f);
//		  HAL_Delay(10);
//	  }*/
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Green_Color, 0.02);
////	  HAL_Delay(300);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Blue_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Purple_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Oringe_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Pink_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, White_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Black_Color, 0.02);
////	  HAL_Delay(300);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, NavyBlue_Color, 0.02);
////	  HAL_Delay(100);
////	  RGB_Color_Convert(&htim8, TIM_CHANNEL_3, Yellow_Color, 0.02);
////	  HAL_Delay(100);
////	  More_RGB[0] = Green_Color;
////	  More_RGB[1] = Blue_Color;
////	  More_RGB[2] = Purple_Color;
////	  More_RGB[3] = Oringe_Color;
////	  More_RGB[4] = Pink_Color;
////	  More_RGB[5] = White_Color;
////	  More_RGB[6] = NavyBlue_Color;
////	  More_RGB[7] = Yellow_Color;
////	  RGB_Color_Convert_Pro(&htim8, TIM_CHANNEL_3, 0.05);
////	  HAL_Delay(300);
////	  static uint8_t Show_Light = 0;
//	  static uint8_t UI = 0;
//	  static uint32_t Counter_RGB = 0;
//	  if(button[25] == 1)
//	  {
//		for(int i=0;i<85;i++)
//		  More_RGB[i] = Black_Color;
//	    for(int i=1;i<=Speed_RGB;i++) 
//		  More_RGB[i] = Green_Color; 
//		  for(int i=65 - (Speed_RGB*25/50);i<65 + (Speed_RGB*25/50);i++) 
//		  More_RGB[i] = Green_Color;
//		RGB_Color_Convert_Pro(&htim4, TIM_CHANNEL_3, 0.5);
//	  }
//	  else if(button[25] == 0)
//	  {
//		 Counter_RGB = (Counter_RGB + 1)%1700;
//		 UI = Counter_RGB / 20;
// 		 for(int i=0;i<85;i++)
//			More_RGB[i] = Black_Color;
//		 More_RGB[UI] = Green_Color;
//		 More_RGB[(UI+5)%85] = Red_Color;
//		 More_RGB[(UI+10)%85] = Green_Color;
//		 More_RGB[(UI+15)%85] = Red_Color;
//		 More_RGB[(UI+20)%85] = Green_Color;
//		 More_RGB[(UI+25)%85] = Red_Color;
//		 More_RGB[(UI+30)%85] = Green_Color;
//		 More_RGB[(UI+35)%85] = Red_Color;
//		 More_RGB[(UI+40)%85] = Green_Color;
//		 More_RGB[(UI+45)%85] = Red_Color;
//		 More_RGB[(UI+50)%85] = Green_Color;
//		 More_RGB[(UI+55)%85] = Red_Color;
//		 More_RGB[(UI+60)%85] = Green_Color;
//		 More_RGB[(UI+65)%85] = Red_Color;
//		 More_RGB[(UI+70)%85] = Green_Color;
//		 More_RGB[(UI+75)%85] = Red_Color;
//		 RGB_Color_Convert_Pro(&htim4, TIM_CHANNEL_3, 0.5);
//	  }
////	  HAL_Delay(300);
//}
/** 
  *@time          :4.1
  *@name          :RGB_Work
  *@brief         :����RGB�� //�����Լ��������дRGB�ƵĿ���
  *@param[in]     :null
  *@param[out]    :null
  *@return        :null
**/
void Test_Self_RGB(void)
{
//	RGB_Color_Convert(&htim4, TIM_CHANNEL_3, Green_Color, 0.05);
//	HAL_Delay(500);
	RGB_Color_Convert(&htim4, TIM_CHANNEL_3, Purple_Color, 0.05);
//	HAL_Delay(500);
//	RGB_Color_Convert(&htim4, TIM_CHANNEL_3, Pink_Color, 0.05);
//	HAL_Delay(500);
}

