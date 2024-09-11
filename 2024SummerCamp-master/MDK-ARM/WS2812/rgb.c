#include "rgb.h"
#include "dma.h"
#include "tim.h"
#include "string.h"

/*********************************************************************************
  DMA配置时流向要配置成从内存到外设,Priority可置为High
  定时器本身的中断和DMA的中断似乎不能同时开
*********************************************************************************/

uint16_t RGB_ResetBuf[230] = {0}; //复位发送数组，DMA半字传输所以是uint16_t
uint16_t ColorBuf_0[25] = {0}; //发送数组BUF0
uint16_t ColorBuf_1[25] = {0}; //发送数组BUF1
uint16_t Color_360[RGB_NUM] = {0};  //角度信息0-360
uint16_t Color_RGB[RGB_NUM][3] = {0};  //0-255 存RGB信息的数组 0-R 1-G 2-B
uint8_t RGB_TX_Flag = 0; //发送数组BUF标号指向信息
uint8_t RGB_TX_Finish_Flag = 1; //开始时算是完成了一次发送，要通过发送复位数组更新RGB灯状态了
uint16_t RGB_Index = 0; //RGB灯计数指针
uint32_t More_RGB[RGB_NUM] = {0};
uint8_t Speed_RGB = 0;
uint32_t States_RGB[RGB_NUM] = {0};
float States_RGB_Light[RGB_NUM] = {0};

/*********************************************************************************
  *@  name      : RGB_Cal_Color
  *@  function  : 计算除RGB信息对应的发送数组
  *@  input     : 时钟, 通道, 要DMA发送的数组, 存储RGB信息的数组
  *@  output    : None
*********************************************************************************/
void RGB_Cal_Color(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* ColorBuf, uint16_t Color_RGB[][3])
{
	for(int8_t i = 7; i>=0; i--) //关于R的信息的发送，高位先接收，所以高位要存在发送BUF的低位
			ColorBuf[7-i] = ((Color_RGB[RGB_Index][1]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	
	for(int8_t i = 7; i>=0; i--) //关于B的信息的发送，高位先接收，所以高位要存在发送BUF的低位
			ColorBuf[15-i] = ((Color_RGB[RGB_Index][0]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	
	for(int8_t i = 7; i>=0; i--) //关于G的信息的发送，高位先接收，所以高位要存在发送BUF的低位
			ColorBuf[23-i] = ((Color_RGB[RGB_Index][2]>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
} 
/*********************************************************************************
  *@  name      : RGB_Color_Convert
  *@  function  : 根据输入的RGB和亮度信息来直接控制RGB灯发光
  *@  input     : 时钟, 通道, RGB信息, 亮度
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
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //开始传输
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //开始计算BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //开始计算BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Color_Convert_Pro
  *@  function  : 根据输入的RGB和亮度信息来直接控制RGB灯发光，为多个灯不同颜色而设计的版本
  *@  input     : 时钟, 通道, RGB信息, 亮度
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
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //开始传输
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //开始计算BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //开始计算BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Color_Convert_Pro_with_Light
  *@  function  : 根据输入的RGB和亮度信息来直接控制RGB灯发光，为多个灯不同颜色以及亮度而设计的版本
  *@  input     : 时钟, 通道, RGB信息, 亮度
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
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //开始传输
		RGB_TX_Finish_Flag = 0;
		RGB_Index = 0;
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //开始计算BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //开始计算BUF1
	}
	
}
/*********************************************************************************
  *@  name      : RGB_Change
  *@  function  : 根据输入的角度和亮度来控制RGB灯发出不同颜色
  *@  input     : 时钟, 通道, 角度（不同角度不同颜色，应该是结合陀螺仪应用的函数）, 亮度（有空加入角速度决定亮度  Bright 0-1）
  *@  output    : None
*********************************************************************************/
void RGB_Change(TIM_HandleTypeDef* htim, uint8_t Channel, uint16_t* Color_360, float Bright)
{
	float Scale = 0.0f;

	if(RGB_TX_Finish_Flag == 1)  //上一次DMA发送完成后，才可开始下一次DMA传输
	{
		/*360度映射到RGB*/
		for(uint16_t i=0; i<RGB_NUM; i++)
		{
			if(Color_360[i]<60)//Red->Green,偏Red
			{
				Scale = (float)Color_360[i]/60;
				Color_RGB[i][0] = 255*Bright;
				Color_RGB[i][1] = 255*Scale*Bright;
				Color_RGB[i][2] = 0;
			}
			else if(Color_360[i]>=60 && Color_360[i]<120)//Red->Green,偏Green
			{
				Scale = (float)(Color_360[i]-60)/60;
				Color_RGB[i][0] = 255*(1-Scale)*Bright;
				Color_RGB[i][1] = 255*Bright;
				Color_RGB[i][2] = 0;
			}
			else if(Color_360[i]>=120 && Color_360[i]<180)//Green->Blue,偏Green
			{
				Scale = (float)(Color_360[i]-120)/60;
				Color_RGB[i][0] = 0;
				Color_RGB[i][1] = 255*Bright;
				Color_RGB[i][2] = 255*Scale*Bright;
			}
			else if(Color_360[i]>=180 && Color_360[i]<240)//Green->Blue,偏Blue
			{
				Scale = (float)(Color_360[i]-180)/60;
				Color_RGB[i][0] = 0;
				Color_RGB[i][1] = 255*(1-Scale)*Bright;
				Color_RGB[i][2] = 255*Bright;
			}
			else if(Color_360[i]>=240 && Color_360[i]<300)//Blue->Red,偏Blue
			{
				Scale = (float)(Color_360[i]-240)/60;
				Color_RGB[i][0] = 255*Scale*Bright;
				Color_RGB[i][1] = 0;
				Color_RGB[i][2] = 255*Bright;
			}
			else if(Color_360[i]>=300 && Color_360[i]<360)//Blue->Red,偏Red
			{
				Scale = (float)(Color_360[i]-300)/60;
				Color_RGB[i][0] = 255*Bright;
				Color_RGB[i][1] = 0;
				Color_RGB[i][2] = 255*(1-Scale)*Bright;
			}
		}
		
		HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)RGB_ResetBuf, 230);  //发送复位信号从而使RGB灯状态更新
		RGB_TX_Finish_Flag = 0; //下一次发送
		RGB_Index = 0; //下一次计数
		if(RGB_TX_Flag == 0)
		RGB_Cal_Color(htim, Channel, ColorBuf_0, Color_RGB); //开始计算BUF0
		if(RGB_TX_Flag == 1)
		RGB_Cal_Color(htim, Channel, ColorBuf_1, Color_RGB); //开始计算BUF1
	}
}
/*脉冲计数结束中断回调函数*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3); //关闭通道
//	if(RGB_TX_Finish_Flag==0)
//	{
//		RGB_TX_Finish_Flag=1;
//		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_0, 25); //直接只用一个BUF数组发送，好像也不会出问题
//	}
	if(RGB_TX_Flag==0 && RGB_TX_Finish_Flag==0) //如果发送未完成，且要发送的是BUF0
	{
		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_0, 25); //每个RGB灯锁存24bits,第25bit为零防止出现问题
		//改变CCRx的值就可以改变PWM的输出的占空比。改变ARR的值，就可以改变PWM的输出的频率，这就是PWM的输出原理。
		//HAL_TIM_PWM_Start_DMA函数可以在每单个PWM波结束后就会自动将比较值设置成DMA传输来的数据
		//要在CubeMx里面配置CH Polarity输出极性（当计数值小于CCRx时输出0/1）
		if(RGB_Index == RGB_NUM-1)  RGB_TX_Finish_Flag = 1; //如果这一轮RGB的信息发送完了，发送完成标志符置1，注意Index是从0开始计数的
		RGB_TX_Flag = 1; //下一次要发送BUF1
		if(RGB_TX_Finish_Flag==0) //如果没有发送结束
		{
			RGB_Index++; //发送下一个RGB灯的信息
			RGB_Cal_Color(htim, TIM_CHANNEL_3, ColorBuf_1, Color_RGB); //计算BUF1，BUF0和BUF1要轮流发送，不然在发BUF0的过程中又改了BUF0的值，程序会出问题
		}
	}
	else if(RGB_TX_Flag==1 && RGB_TX_Finish_Flag==0) //如果发送未完成，且要发送的是BUF1
	{
		HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_3, (uint32_t*)ColorBuf_1, 25);
		if(RGB_Index == RGB_NUM-1)  RGB_TX_Finish_Flag = 1; //如果这一轮RGB的信息都发送完了，发送完成标志符置1
		RGB_TX_Flag = 0; //下一次要发送BUF0
		if(RGB_TX_Finish_Flag==0)  //如果没有发送完
		{
			RGB_Index++; //发送下一个RGB灯的信息
			RGB_Cal_Color(htim, TIM_CHANNEL_3, ColorBuf_0, Color_RGB); //计算BUF0，BUF0和BUF1要轮流发送，不然在发BUF1的过程中又改了BUF1的值，程序会出问题
		}
	}
	
}

///** 
//  *@time          :8.6
//  *@name          :RGB_Work
//  *@brief         :控制RGB灯 //根据自己的需求编写RGB灯的控制
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
  *@brief         :控制RGB灯 //根据自己的需求编写RGB灯的控制
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

