#include "RM2006.h"
#include "string.h"
#include "RM3508.H"


/*********************************************************************************
  �ٶȻ�PID���ã�Kp Ki Kd
  ��̨������     8  0  0
  ��̨��ת��     8  0  0
  ��������     8  0  0
*********************************************************************************/
M2006_PID M2006_Speed_Pid[8] =//8 0 0    8 0 0   
{
	{.Kp = 11,.Ki = 1,.Kd = 0.010,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 1
	{.Kp = 21,.Ki = 0,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 2
	{.Kp = 25.5,.Ki = 0.9,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	//ID = 3
	{.Kp = 19.45,.Ki = 0.1,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 4
	/**/{.Kp = 8,.Ki = 0.f,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 5
	{.Kp = 8,.Ki = 0,.Kd = 12,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 6
	{.Kp = 8,.Ki = 0,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 7
	{.Kp = 8,.Ki = 0,.Kd = 0,.Max = 16000,.Min = -16000,.IntegralLimit = 1000,.DeadBand = 10},	 	//ID = 8
};
/*********************************************************************************
λ��PID���ã� Kp    Ki     Kd
��̨������   0.2  0.009   0.01
��̨��ת��   0.2    0       0
��������
*********************************************************************************/

M2006_PID M2006_Pos_Pid[8] = //��̨0.2, 0.009 0.01  ����0.2 0 0
{
	{.Kp = 0.1,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 1
	{.Kp = 0.01,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 2
	{.Kp = 0.01,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 3
	{.Kp = 3.2,.Ki = 0.005,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 4
	/**/{.Kp = 0.1,.Ki = 0,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 100},	//ID = 5// 
	//{.Kp = 0.7,.Ki = 0.05,.Kd = 5,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 5
	//{.Kp = 0.7,.Ki = 0,.Kd = 10,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 5
	{.Kp = 0.16,.Ki = 0.001,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 0},	//ID = 6
	{.Kp = 0.5,.Ki = 0.005,.Kd = 10,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 200},	//ID = 7
	{.Kp = 3.2,.Ki = 0.005,.Kd = 0,.Max = 14000,.Min = -14000,.IntegralLimit = 20000,.DeadBand = 2000}	//ID = 8   8 0.001 20
};

//����ʱ����5~10ms
const uint8_t RM2006_Reduction_Ratio[8] = {36,36,36,36,36,36,36,36};//������ٱ�����



//���ڴ洢���������ȫ�ֱ���
uint8_t RM2006_Feedback_Buf[8][7];		//�������ֵ(ȫ�ֱ���)
int RM2006_Pos[8];					//ÿһ��Ԫ�ض�Ӧһ��ID�ĵ������Ϣ
uint8_t RM2006_Sendbuf1[8] = {0};  //CAN��������
uint8_t RM2006_Sendbuf2[8] = {0};  //CAN��������



/*********************************************************************************
  *@  name      : RM2006_Set_I
  *@  function  : RM2006�����������
  *@  input     : Ŀ����������id
  *@  output    : �ɹ�����0��ʧ�ܷ���1
*********************************************************************************/
uint8_t RM2006_Set_I(int target_i,uint8_t motor_id)
{
	if( motor_id>=1 && motor_id<=8 ) 
	{
		int send_id = 0;
		send_id = send_id;
		
		if( target_i<=-10000 )
			target_i=-10000;
		else if( target_i>=10000 )
			target_i=10000;

        if(motor_id <=4 )   //ǰ�ĸ�ID��Ӧ��sendID��ʶ����0x200
        {
            send_id=0x200;  

            RM2006_Sendbuf2[2*motor_id-2]=target_i>>8;                  //����ֵ��8λ
            RM2006_Sendbuf2[2*motor_id-1]=target_i & 0x00ff;            //����ֵ��8λ

        }
        else 							//���ĸ�ID��Ӧ��sendID��ʶ����0x1ff
        {
            send_id=0x1ff;
            motor_id-=4;

        RM2006_Sendbuf1[2*motor_id-2]=target_i>>8;                  //����ֵ��8λ
        RM2006_Sendbuf1[2*motor_id-1]=target_i & 0x00ff;            //����ֵ��8λ

        }

    if(send_id==0x200)//ID1-4��can1��
		 RM2006_CAN_Send_Data(&hcan1, RM2006_Sendbuf2, send_id ,8);
    else
		 RM2006_CAN_Send_Data(&hcan2, RM2006_Sendbuf1, send_id ,8);
      
		return 0;
	}
	else 
	return 1;
}
/********************************************************************************
  *@  name      : RM2006_CAN_Send_Data
  *@  function  : CAN��������
  *@  input     : ����CAN�ߣ�Ҫ���͵����ݣ����͵��ٲö�ID�����ݳ���
  *@  output    : �ɹ�����0��ʧ�ܷ���2
*********************************************************************************/
uint8_t RM2006_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
	uint8_t FreeTxNum = 0;
	CAN_TxHeaderTypeDef TxMessage;
	
	TxMessage.IDE = CAN_ID_STD;  //��׼֡,CAN_ID_EXT��չ֡;
	TxMessage.RTR = CAN_RTR_DATA;  //����֡,CAN_RTR_REMOTEң��֡
	TxMessage.StdId = ID;
	TxMessage.DLC = Len;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);	
		
	while(FreeTxNum==0)  //�ȴ�������
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
	
	//HAL_Delay(2); //û����ʱ���п��ܻᷢ��ʧ��
	
	HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
	
	if(HAL_RetVal!=HAL_OK)
	{
		return 2;
	}
	
	return 0;
}
/********************************************************************************
  *@  name      : RM2006_Set_Speed
  *@  function  : RM2006�ٶ�����
  *@  input     : Ŀ���ٶȣ�-15000~15000���ɽ��ܣ������id
  *@  output    : ��
********************************************************************************/
void RM2006_Set_Speed(int goal_speed,int ID)
{
	uint8_t id = ID-1;
	M2006_Speed_Pid[id].Target = goal_speed;
	M2006_Speed_Pid[id].NowIS = RM2006_Get_Speed(ID);
	M2006_Speed_Pid[id].Err_last = M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].Err 	= M2006_Speed_Pid[id].Target -  M2006_Speed_Pid[id].NowIS;
	
	if(FabsD(M2006_Speed_Pid[id].Err) < M2006_Speed_Pid[id].DeadBand) M2006_Speed_Pid[id].Err = 0;
  M2006_Speed_Pid[id].Err_sum += M2006_Speed_Pid[id].Err; 
	M2006_Speed_Pid[id].Err_sum = CLAMP(M2006_Speed_Pid[id].Err_sum,-M2006_Speed_Pid[id].IntegralLimit,M2006_Speed_Pid[id].IntegralLimit);
	M2006_Speed_Pid[id].P_Out = M2006_Speed_Pid[id].Kp * M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].I_Out = M2006_Speed_Pid[id].Ki * M2006_Speed_Pid[id].Err_sum;
	M2006_Speed_Pid[id].D_Out = M2006_Speed_Pid[id].Kd * (M2006_Speed_Pid[id].Err - M2006_Speed_Pid[id].Err_last); 
	
	M2006_Speed_Pid[id].PID_Out = M2006_Speed_Pid[id].P_Out +M2006_Speed_Pid[id].I_Out + M2006_Speed_Pid[id].D_Out;
	M2006_Speed_Pid[id].PID_Out = CLAMP(M2006_Speed_Pid[id].PID_Out,M2006_Speed_Pid[id].Min,M2006_Speed_Pid[id].Max);

	
	RM2006_Set_I(M2006_Speed_Pid[id].PID_Out,ID);	
}
/********************************************************************************
  *@  name      : Head_Set_Speed
  *@  function  : rm2006�ٶ�����
  *@  input     : Ŀ���ٶȣ�-15000~15000���ɽ��ܣ������id
  *@  output    : ��
********************************************************************************/
//void Head_Set_Speed(int goal_speed,int ID)
//{
//	uint8_t id = ID-1;
//	M2006_Speed_Pid[id].Err_last = M2006_Speed_Pid[id].Err;
//	M2006_Speed_Pid[id].Err 	= goal_speed -  RM2006_Get_Speed(ID);
//	M2006_Speed_Pid[id].Err_sum += M2006_Speed_Pid[id].Err; 
//	M2006_Speed_Pid[id].Err_sum = CLAMP(M2006_Speed_Pid[id].Err_sum,-1000,1000);
//	
//	M2006_Speed_Pid[id].P_Out = M2006_Speed_Pid[id].Kp * M2006_Speed_Pid[id].Err;
//	M2006_Speed_Pid[id].I_Out = M2006_Speed_Pid[id].Ki * M2006_Speed_Pid[id].Err_sum;
//	M2006_Speed_Pid[id].D_Out = M2006_Speed_Pid[id].Kd * (M2006_Speed_Pid[id].Err - M2006_Speed_Pid[id].Err_last); 
//	
//	M2006_Speed_Pid[id].PID_Out = M2006_Speed_Pid[id].P_Out + M2006_Speed_Pid[id].I_Out + M2006_Speed_Pid[id].D_Out;
//	M2006_Speed_Pid[id].PID_Out = CLAMP(M2006_Speed_Pid[id].PID_Out,M2006_Speed_Pid[id].Min,M2006_Speed_Pid[id].Max);

//	
//	RM2006_Set_I(M2006_Speed_Pid[id].PID_Out,ID);	
//}


/********************************************************************************
  *@  name      : RM2006_Set_Pos
  *@  function  : rm2006λ������
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id
  *@  output    : ��
********************************************************************************/
void RM2006_Set_Pos(float pos,int ID)
{
	uint8_t id = ID-1;
	float goal_cnt;
	goal_cnt=pos;
//	goal_cnt=RM2006_Ang2Cnt(angle,ID);//�ú���Ϊ����Ƕ�תcnt
	M2006_Pos_Pid[id].Target = goal_cnt;
	M2006_Pos_Pid[id].NowIS = RM2006_Get_Pos(ID);
	M2006_Pos_Pid[id].Err_last 	= M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].Err 	= goal_cnt - M2006_Pos_Pid[id].NowIS;
	
	if(FabsD(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].DeadBand) M2006_Pos_Pid[id].Err = 0;
	
	M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err; 
	M2006_Pos_Pid[id].Err_sum = CLAMP(M2006_Pos_Pid[id].Err_sum,-M2006_Pos_Pid[id].IntegralLimit,M2006_Pos_Pid[id].IntegralLimit);
	
	M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].I_Out = M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum;
	M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last); 
	
	M2006_Pos_Pid[id].PID_Out = M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out;
	M2006_Pos_Pid[id].PID_Out = CLAMP(M2006_Pos_Pid[id].PID_Out,M2006_Pos_Pid[id].Min,M2006_Pos_Pid[id].Max );

	RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out,ID);	
}
/********************************************************************************
  *@  name      : RM2006_Set_Pos
  *@  function  : rm2006λ������
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id
  *@  output    : ��
********************************************************************************/
void RM2006_Set_Ang(float angle,int ID)
{
	uint8_t id = ID-1;
	float goal_cnt;
	goal_cnt=RM2006_Ang2Cnt(angle,ID);//�ú���Ϊ����Ƕ�תcnt
	M2006_Pos_Pid[id].Target = goal_cnt;
	M2006_Pos_Pid[id].NowIS = RM2006_Get_Pos(ID);
	M2006_Pos_Pid[id].Err_last 	= M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].Err 	= goal_cnt - M2006_Pos_Pid[id].NowIS;
	
	if(FabsD(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].DeadBand) M2006_Pos_Pid[id].Err = 0;
	
	M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err; 
	M2006_Pos_Pid[id].Err_sum = CLAMP(M2006_Pos_Pid[id].Err_sum,-M2006_Pos_Pid[id].IntegralLimit,M2006_Pos_Pid[id].IntegralLimit);
	
	
	
	M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
	M2006_Pos_Pid[id].I_Out = M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum;
	M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last); 
	
	M2006_Pos_Pid[id].PID_Out = M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out;
	M2006_Pos_Pid[id].PID_Out = CLAMP(M2006_Pos_Pid[id].PID_Out,M2006_Pos_Pid[id].Min,M2006_Pos_Pid[id].Max );

	RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out,ID);	
}
/*********************************************************************************
  *@  name      : RM2006_Get_Feedback
  *@  function  : ��ȡRM2006����ķ���������ȫ�ֱ���RM2006_Feedback_Buf[8][7];
  *@  input     : message_id,message����ָ��
  *@  output    : ��
*********************************************************************************/
void RM2006_Get_Feedback(uint32_t std_id,uint8_t* data_p)
{
	int i;
	for(i=1;i<9;i++)
	{
		if(std_id==0x200+i)
		{
			memcpy(RM2006_Feedback_Buf [i-1],data_p,7);
			RM2006_Pos_Rec(i);
			return;
		}
	}
}
/*********************************************************************************
  *@  name      : RM2006_Get_Torque
  *@  function  : ��ȡRM2006�����ʵ��ת����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����ת��,��ȡʧ�ܷ���0
*********************************************************************************/
int RM2006_Get_Torque(uint8_t motor_id)
{
	int torque = 0;
	if(RM2006_Feedback_Buf[motor_id-1][2]>>7==1)
		torque = -( 0xffff-(  (RM2006_Feedback_Buf[motor_id-1][4]<<8)+RM2006_Feedback_Buf[motor_id-1][5])  ) ;
	else 
		torque = (RM2006_Feedback_Buf[motor_id-1][4]<<8)+RM2006_Feedback_Buf[motor_id-1][5];
	return torque;
}
/*********************************************************************************
  *@  name      : RM2006_Get_Speed
  *@  function  : ��ȡRM2006����ķ������ٶ���Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid������ٶ�,��ȡʧ�ܷ���0
*********************************************************************************/
int RM2006_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if(RM2006_Feedback_Buf[motor_id-1][2]>>7==1)
		speed = -( 0xffff-(  (RM2006_Feedback_Buf[motor_id-1][2]<<8)+RM2006_Feedback_Buf[motor_id-1][3])  ) ;
	else 
		speed = (RM2006_Feedback_Buf[motor_id-1][2]<<8)+RM2006_Feedback_Buf[motor_id-1][3];
	return speed;
}
/*********************************************************************************
  *@  name      : RM2006_Get_Pos
  *@  function  : ��ȡRM2006�����ǰ��λ����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ�ã���������CNTֵ
*********************************************************************************/
int RM2006_Get_Pos(uint8_t motor_id)
{
	return RM2006_Pos[motor_id - 1];
}
/*********************************************************************************
  *@  name      : RM2006_Temperature
  *@  function  : ��ȡRM2006�����ǰ���¶���Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid������¶�ֵ
*********************************************************************************/
uint8_t RM2006_Temperature(uint8_t id)
{
    uint8_t Tem;
    Tem = RM2006_Feedback_Buf[id-1][6];
    return Tem;
}
/*********************************************************************************
  *@  name      : RM2006_Pos_Rec
  *@  function  : ��ȡRM2006����ķ�����λ����Ϣ  //�ۻ�·��
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ����Ϣ,��ȡʧ�ܷ���-1
*********************************************************************************/
static int32_t	RM2006_base[8] = {0};	//��������Ѿ�ת���ı�����������һȦ8192

void RM2006_Pos_Rec(uint8_t motor_id)
{
	int id=motor_id-1;
	int32_t	RM2006_tmp[8];
	
	static int32_t RM2006tmp_pre[8] = {0};

	RM2006_tmp[id]=(RM2006_Feedback_Buf[id][0]<<8)+RM2006_Feedback_Buf[id][1];
	if ( RM2006_tmp[id] - RM2006tmp_pre[id] > 4095 )  //ת��8191��0ʱ��¼Ȧ��
		RM2006_base[id] -= 8191;
	else if ( RM2006_tmp[id] - RM2006tmp_pre[id] < -4095 )
		RM2006_base[id] += 8191;
	
	RM2006tmp_pre[id] = RM2006_tmp[id];
	RM2006_Pos[id] = RM2006_base[id] + RM2006_tmp[id];	
	
}
/********************************************************************************
  *@  name      : RM2006_Ang2Cnt
  *@  function  : �Ƕ�ת��Ϊʵ�ʵ��Ӧ��ת����λ���� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���λ��
********************************************************************************/
int RM2006_Ang2Cnt(float angle,int ID)  
{

	int cnt;
	cnt = (int)(RM2006_CNT_PER_ROUND_OUT(ID) * angle/360);
	return cnt;
}
/********************************************************************************
  *@  name      : RM2006_Cnt2Ang
  *@  function  : ���λ��ת��Ϊ�Ƕ� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : ���λ�ã����id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���ת���ĽǶ�
********************************************************************************/
double RM2006_Cnt2Ang(int32_t cnt,int ID)
{
	double angled;
	angled = (double)((cnt * 360.0)/RM2006_CNT_PER_ROUND_OUT(ID));
	return angled;
}

/********************************************************************************
  *@  name      : RM2006_Set_NowPos
  *@  function  : ��M2006����ĵ�ǰֵ����Ϊ����λ��
  *@  input     : ���id����ǰλ������Ϊ���� //��λ�Ǳ���������
  *@  output    : void
********************************************************************************/
void RM2006_Set_NowPos(uint8_t ID,int32_t Pos_Angle)
{
	uint8_t id;
	id=ID-1;
	RM2006_base[id]=Pos_Angle;
}
