#include "filt.h"
#include "DR_rm3508.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "ICM42688.h"


#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


#include "main.h"
#include "can.h"
//#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
//卡尔曼参数定义
Kal_Filter k_filt = {.p=1.0,
    .prevData=0.0,
    .q=0.000001f,
    .r=0.05f,
	  .kGain=0.0,
    .inData=0.0,};

Kal_Filter *K_filt = &k_filt;

Kal_Filter k_filt1 = 	{1.0f,	  // k_flt.C_last
	 0.0f,	  // k_flt.X_last
	 0.0001f, // k_flt.Q
	 5.0f,	  // k_flt.R 4.0
	 0.0f, 0.0f, };

Kal_Filter *K_filt1 = &k_filt1;
////算术均值滤波
//int averageFilter(int N)
//{
//   int sum = 0;
//   short i;
//   for(i = 0; i < N; ++i)
//   {
//        sum += HAL_ADC_GetValue(&hadc1);	
//   }
//   return sum/N;
//}

////一阶互补滤波
//int firstOrderFilter(int newValue, int oldValue, float a)
//{
//	return a * newValue + (1-a) * oldValue;
//}





//卡尔曼滤波
float KalmanFilter(Kal_Filter* K_Filt,float inData)
{
	//数据传递
	 K_Filt->inData = inData;
	//预测协方差矩阵
   K_Filt->p = K_Filt->p+K_Filt->q;
	//计算卡尔曼增益
   K_Filt->kGain = K_Filt->p/(K_Filt->p+K_Filt->r);
	//计算当前最优估计值
   K_Filt->inData = K_Filt->prevData + K_Filt->kGain*(K_Filt->inData - K_Filt->prevData);
	//更新协方差矩阵
   K_Filt->p = (1-K_Filt->kGain)*K_Filt->p;
	//更新数据
   K_Filt->prevData = K_Filt->inData;
   return K_Filt->inData; 
}






///*//////////////////////////////////////////////////////////////////////////
//方法一：限幅滤波法
//方法：根据经验判断，确定两次采样允许的最大偏差值（设为A），每次检测到新值时判断：
//      如果本次值与上次值之差<=A，则本次值有效，
//      如果本次值与上次值之差>A，则本次值无效，放弃本次值，用上次值代替本次值。
//优点：能克服偶然因素引起的脉冲干扰
//缺点：无法抑制周期性的干扰，平滑度差
////////////////////////////////////////////////////////////////////////////*/
//#define  A 51
//u16 Value1;

//u16 filter1() 
//{
//  u16 NewValue;
//	Value1 = ftable[b-1];
//  NewValue = ftable[b];
//	b++;
//	a++;
//	if(a==255) a=0;//应该是采样总数
//	if(b==255) b=1;
//  if(((NewValue - Value1) > A) || ((Value1 - NewValue) > A))//与A 进行比较
//	{
//                  //如果误差超过A则返回上一次测量值
//    return Value1;
//	}
//  else
//	{
//     return NewValue;
//	}
//}


///*//////////////////////////////////////////////////////////////////////////
//方法二：中位值滤波法
//方法： 连续采样N次（N取奇数），把N次采样值按大小排列，取中间值为本次有效值。
//优点：克服偶然因素（对温度、液位的变化缓慢的被测参数有良好的滤波效果）
//缺点：对流量、速度等快速变化的参数不宜
////////////////////////////////////////////////////////////////////////////*/
//#define N 3

//u16 value_buf[N]; 
//u16 filter2()
//{  
//  u16 count,i,j,temp;
//  for(count=0;count<N;count++)
//  {
//    value_buf[count] =  ftable[a];
//	  a++;
//	  if(a==255) a=0;
//  }
//	for (j=0;j<N-1;j++)   //从小到大排序
//	{
//		 for (i=0;i<N-j;i++)
//		 {
//			if ( value_buf[i] >  value_buf[i+1] )
//			{
//			 temp = value_buf[i];
//			 value_buf [i]= value_buf[i+1]; 
//			 value_buf[i+1] = temp;
//			}
//		 }
//	}
//	return value_buf[(N-1)/2];
//}



///*//////////////////////////////////////////////////////////////////////////
//方法三：算术平均滤波法
//方法：连续取N个采样值进行算术平均运算：（ N值的选取：一般流量，N=12；压力：N=4。）
//      N值较大时：信号平滑度较高，但灵敏度较低；
//      N值较小时：信号平滑度较低，但灵敏度较高；     
//优点：适用于对一般具有随机干扰的信号进行滤波；这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动
//缺点：对于测量速度较慢或要求数据计算速度较快的实时控制不适用，比较浪费RAM。
////////////////////////////////////////////////////////////////////////////*/

//#define N 5
//u16 filter3()
//{
//	u16 sum = 0,count;
//	for ( count=0;count<N;count++)
//	{
//		sum = sum+ ftable[a];
//		a++;
//		if(a==255) a=0;
//	}
//	return (sum/N);
//}


///*//////////////////////////////////////////////////////////////////////////
//方法四：递推平均滤波法（又称滑动平均滤波法）
//方法： 把连续取得的N个采样值看成一个队列，队列的长度固定为N，
//       每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先进先出原则），
//       把队列中的N个数据进行算术平均运算，获得新的滤波结果。
//       N值的选取：流量，N=12；压力，N=4；液面，N=4-12；温度，N=1-4。
//优点：对周期性干扰有良好的抑制作用，平滑度高；
//      适用于高频振荡的系统。
//缺点：灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差；
//      不易消除由于脉冲干扰所引起的采样值偏差；
//      不适用于脉冲干扰比较严重的场合；
//      比较浪费RAM。
////////////////////////////////////////////////////////////////////////////*/

//#define FILTER4_N 3
//uint16_t filter_buf[FILTER4_N + 1];
//uint16_t filter4() 
//{
//  int i;
//  int filter_sum = 0;
//  filter_buf[FILTER4_N] = ftable[a];		
//	a++;
//	if(a==255) a=0;
//  for(i = 0; i < FILTER4_N; i++) 
//	{
//    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
//    filter_sum += filter_buf[i];       //算平均，但是这一次新移进来的数据不算
//  }
//  return (int)(filter_sum / FILTER4_N);
//}


///*//////////////////////////////////////////////////////////////////////////
//方法五：中位值平均滤波法（又称防脉冲干扰平均滤波法）
//方法： 采一组队列去掉最大值和最小值后取平均值，     （N值的选取：3-14）。 
//      相当于“中位值滤波法”+“算术平均滤波法”。
//      连续采样N个数据，去掉一个最大值和一个最小值，
//      然后计算N-2个数据的算术平均值。    
//优点： 融合了“中位值滤波法”+“算术平均滤波法”两种滤波法的优点。
//       对于偶然出现的脉冲性干扰，可消除由其所引起的采样值偏差。
//       对周期干扰有良好的抑制作用。
//       平滑度高，适于高频振荡的系统。
//缺点：对于测量速度较慢或要求数据计算速度较快的实时控制不适用，比较浪费RAM。
////////////////////////////////////////////////////////////////////////////*/

//#define N 3
//int filter5() 
//{
//  int i, j;
//  int filter_temp, filter_sum = 0;
//  int filter_buf[N];
//  for(i = 0; i < N; i++) 
//	{
//    filter_buf[i] = ftable[a];
//		a++;
//		if(a==255)   a=0;
//    delay_us(10);
//  }
//  // 采样值从小到大排列（冒泡法）
//  for(j = 0; j < N - 1; j++) 
//	{
//    for(i = 0; i < N - 1 - j; i++) 
//		{
//      if(filter_buf[i] > filter_buf[i + 1]) 
//			{
//       filter_temp = filter_buf[i];
//        filter_buf[i] = filter_buf[i + 1];
//        filter_buf[i + 1] = filter_temp;
//      }
//    }
//  }
//  // 去除最大最小极值后求平均
//  for(i = 1; i < N - 1; i++) filter_sum += filter_buf[i];
//  return filter_sum / (N - 2);
//}



///*//////////////////////////////////////////////////////////////////////////
//方法六：限幅平均滤波法
//方法： 相当于“限幅滤波法”+“递推平均滤波法”；
//       每次采样到的新数据先进行限幅处理，
//       再送入队列进行递推平均滤波处理。
//优点： 融合了两种滤波法的优点；
//      对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差。
//缺点：比较浪费RAM。
////////////////////////////////////////////////////////////////////////////*/

//#define FILTER6_N 3
//#define FILTER6_A 51
//int filter_buf[FILTER6_N];

//int filter6() 
//{
//  int i;
//  int filter_sum = 0;
//  filter_buf[FILTER6_N - 1] = ftable[a];		
//	a++;
//	if(a==255)   a=0;
//  if(((filter_buf[FILTER6_N - 1] - filter_buf[FILTER6_N - 2]) > FILTER6_A) || ((filter_buf[FILTER6_N - 2] - filter_buf[FILTER6_N - 1]) > FILTER6_A))//限幅滤波
//    filter_buf[FILTER6_N - 1] = filter_buf[FILTER6_N - 2];

//  for(i = 0; i < FILTER6_N - 1; i++) //递推均值滤波
//	{
//    filter_buf[i] = filter_buf[i + 1];
//    filter_sum += filter_buf[i];
//  }
//  return filter_sum / (FILTER6_N - 1);
//}



///*//////////////////////////////////////////////////////////////////////////
//方法七：一阶滞后滤波法
//方法： 取a=0-1，本次滤波结果=(1-a)*本次采样值+a*上次滤波结果。
//优点：  对周期性干扰具有良好的抑制作用；
//        适用于波动频率较高的场合。
//       平滑度高，适于高频振荡的系统。
//缺点： 相位滞后，灵敏度低；
//      滞后程度取决于a值大小；
//      不能消除滤波频率高于采样频率1/2的干扰信号。
////////////////////////////////////////////////////////////////////////////*/

//#define FILTER7_A 0.01
//u16 Value;
//u16 filter7() 
//{
//  int NewValue;
//	Value = ftable[b-1];		
//  NewValue = ftable[b];		
//	b++;
//	if(b==255)   b=1;
//  Value = (int)((float)NewValue * FILTER7_A + (1.0 - FILTER7_A) * (float)Value);

//  return Value;
//}



///*//////////////////////////////////////////////////////////////////////////
//方法八：加权递推平均滤波法
//方法： 是对递推平均滤波法的改进，即不同时刻的数据加以不同的权；
//       通常是，越接近现时刻的数据，权取得越大。
//       给予新采样值的权系数越大，则灵敏度越高，但信号平滑度越低。
//优点： 适用于有较大纯滞后时间常数的对象，和采样周期较短的系统。
//缺点：  对于纯滞后时间常数较小、采样周期较长、变化缓慢的信号；
//       不能迅速反应系统当前所受干扰的严重程度，滤波效果差。
////////////////////////////////////////////////////////////////////////////*/

//#define FILTER8_N 12
//int coe[FILTER8_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加权系数表
//int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加权系数和
//int filter_buf[FILTER8_N + 1];
//int filter8() 
//{
//  int i;
//  int filter_sum = 0;
//  filter_buf[FILTER8_N] = ftable[a];		
//	a++;
//	if(a==255)   a=0;
//  for(i = 0; i < FILTER8_N; i++) 
// {
//    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
//    filter_sum += filter_buf[i] * coe[i];
//  }
//  filter_sum /= sum_coe;
//  return filter_sum;
//}



