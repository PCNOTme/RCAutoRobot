#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <math.h>
#include "Moto.h"
#include "Xunji.h"


//电机驱动-PID控制C文件
//外部全局变量定义在Moto.h

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5; //tim1,2,5 编码器模式
extern TIM_HandleTypeDef htim8;//tim8 PWM模式

extern osSemaphoreId SPD_overHandle;


extern osMessageQId MotoSetQueues; //电机速度数值设置队列 setpiont1

/*************************************/
//定义PID相关宏
// 这三个参数设定对电机运行影响非常大
/*************************************/
#define  P_DATA      3.2                                 //P参数
#define  I_DATA      1.1                                //I参数
#define  D_DATA      0.15                              //D参数


CCMRAM SPD spd; //轮子速度结构体
CCMRAM PID pid1;
CCMRAM PID pid2;
CCMRAM PID pid3;


/*************************************/
//功能：控制电机正反转
// num：电机号 1~3号电机 4 3个电机刹车 其他num 3个电机停止
// fb :0正转 1 反转 2 停止 3刹车
/*************************************/
void motoX_FB(uint8_t num,uint8_t fb)
{
	switch(num)
	{
		case 1:
			switch(fb)
			{
				case 0:	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_RESET);
            break;
				case 1:	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_SET);
            break;            				
				case 2:	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_SET);
            break;
				case 3:	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_RESET);
            break;
            default: break;
			}
      break;
		case 2:
			switch(fb)
			{
				case 0:	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_RESET);
            break;
				case 1:	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_SET);
            break;            				
				case 2:	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_SET);
            break;
				case 3:	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_RESET);
            break;
            default: break;
			}
      break;
      case 3:
			switch(fb)
			{
				case 0:	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_RESET);
            break;
				case 1:	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_SET);
            break;            				
				case 2:	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_SET),
                     HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_SET);
            break;
				case 3:	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_RESET),
                     HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_RESET);
            break;
            default: break;
			}
      break;
		case 4 : HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_RESET),
					HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_RESET),	
					HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET),
					HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_RESET),
					HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_RESET),
					HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_RESET);
		break;	
		default :HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_SET),
					HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_SET),	
					HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_SET),
					HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_SET),
					HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_SET),
					HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_SET);
		break;			
	
	}		
}

/*************************************/
// 功能：三轮全向移动机器人的控制
// Vx,Vy : 移动平台自身的线速度 X轴，Y轴
// angularVell ：平台绕几何中心的转速
// V1,V2,V3 ：三个全向轮线速度
// theta : 全向轮与y轴夹角AFA的正负偏移量
/*************************************/

void ThreeWheelVellControl(float *Vx,float *Vy, float *angularVell,
	                        volatile int *V1,volatile int *V2,volatile int *V3,float *theta)
{
	#define AFA (60.0f)  //全向轮与y轴夹角
   #define L    (3.0f) //三轮底盘中心到轮中心的距离
	
	*V1 = (int)(-cosf((AFA + *theta) / 180.0f*3.141592653f) * *Vx - sinf((*theta + AFA) / 180.0f*3.141592653f) * *Vy + L * *angularVell);
   *V3 = (int)(cosf(*theta / 180.00f*3.141592653f) * *Vx + sinf(*theta /180.0f*3.141592653f) * *Vy      + L * *angularVell);
   *V2 = (int)(-cosf((AFA - *theta) / 180.0f * 3.141592653f) * *Vx + sinf((AFA - *theta) / 180.0f*3.141592653f) * *Vy + L * *angularVell);

}

//极坐标系表示
//p 机器人朝某一方向速度
//angle 机器人角度 例如60°
void ThreeWheelVellControl2(float *p,float *angle, float *angularVell,
	                        volatile int *V1,volatile int *V2,volatile int *V3,float *theta)
{
   float Vx,Vy;
	Vx = *p*cosf(*angle/180.0f*3.141592653f);
	Vy = *p*sinf(*angle/180.0f*3.141592653f);
	ThreeWheelVellControl(&Vx,&Vy,angularVell,V1,V2,V3,theta);
}

void IncPIDInit(PID *pidconfig) 
{
    pidconfig->LastError=0;            //Error[-1]
    pidconfig->PrevError=0;            //Error[-2]
    pidconfig->Proportion=P_DATA;      //比例常数 Proportional Const
    pidconfig->Integral=I_DATA;        //积分常数  Integral Const
    pidconfig->Derivative=D_DATA;      //微分常数 Derivative Const
    pidconfig->SetPoint=0;           //设定目标Desired Value
}

/********************增量式PID控制设计************************************
int IncPIDCalc(int NextPoint) 
{
  int iError,iIncpid;                                 //当前误差
  iError=sptr->SetPoint-NextPoint;                    //增量计算
  iIncpid=(sptr->Proportion * iError)                 //E[k]项
              -(sptr->Integral * sptr->LastError)     //E[k-1]项
              +(sptr->Derivative * sptr->PrevError);  //E[k-2]项
              
  sptr->PrevError=sptr->LastError;                    //存储误差，用于下次计算
  sptr->LastError=iError;
  return(iIncpid);                                    //返回增量值
}
***************************************************************************/


/********************位置式 PID 控制设计************************************/
int LocPIDCalc(float NextPoint,PID *pidconfig)
{ 
  int iError,dError;
  iError = pidconfig->SetPoint - NextPoint; //偏差
  pidconfig->SumError += iError; //积分
  dError = iError - pidconfig->LastError; //微分
  pidconfig->LastError = iError;
  return(pidconfig->Proportion * iError //比例项
  + pidconfig->Integral * pidconfig->SumError //积分项
  + pidconfig->Derivative * dError); //微分项
}

osPoolId  mpool;
osMessageQId MotoQueues; //电机编码器数值读取队列 spd1 spd2 spd3
void MotoTask(void const * argument)
{
//变量初始化
	spd.SPD1 = 0;
	spd.SPD2 = 0;
	spd.SPD3 = 0;
   spd.PWM_Duty1 = 0;
	spd.PWM_Duty2 = 0;
	spd.PWM_Duty3 = 0;
	
//创建内存池管理函数 	
  osPoolDef(mpool,5,uint32_t); 	
  mpool = osPoolCreate(osPool(mpool));
//创建电机编码器数值读取队列
  osMessageQDef(MotoQueues,5, SPD);
  MotoQueues = osMessageCreate(osMessageQ(MotoQueues), NULL);
//PWM输出开启
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);	
//编码器开启
/**************************************************/
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	
/************************************************/ 
//	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(IN42_GPIO_Port,IN42_Pin,GPIO_PIN_RESET);
   IncPIDInit(&pid1);
	IncPIDInit(&pid2);
	IncPIDInit(&pid3);
   //while(1) {osDelay(1);}
	//PWM_Duty = 10;
	pid1.SetPoint = 0;
	pid2.SetPoint = 0;  ///调节转速设置 在此处
	pid3.SetPoint = 0;  //1 :每4秒1圈 最大80
	HAL_TIM_Base_Start_IT(&htim6);
	osEvent event;
	SPD *MotoSpdXRe;
	uint32_t *MotoSetRe;
	for(;;)
	{
		event = osMessageGet(MotoQueues,50);  //采用消息队列 TIM6中断每50ms发送消息 每秒脉冲数为 50ms*20*spd.SPD* 
		if (event.status == osEventMessage)
		{
			MotoSpdXRe = event.value.p;
			spd.SPD1 = MotoSpdXRe->SPD1;
			spd.SPD2 = MotoSpdXRe->SPD2;
			spd.SPD3 = MotoSpdXRe->SPD3;
		   osPoolFree(mpool, MotoSpdXRe); //释放之前的内存
		
	//采用二值信号量 TIM6中断每50ms释放信号量 每秒脉冲数为 50ms*20*spd.SPD* 
	 //if(SPD_overHandle != NULL){
	//	if(osSemaphoreWait(SPD_overHandle,100) == osOK) {
			if(spd.SPD1<0) spd.SPD1 = -spd.SPD1;
			if(spd.SPD2<0) spd.SPD2 = -spd.SPD2;
			if(spd.SPD3<0) spd.SPD3 = -spd.SPD3;
			
	//			Setpointtemp[0] = pid1.SetPoint;
	//			Setpointtemp[1] = pid2.SetPoint;
	//			Setpointtemp[2] = pid3.SetPoint;		
			//ThreeWheelVellControl2(&Moto_p,&Moto_angle,&Moto_angularVell,&pid1.SetPoint,&pid2.SetPoint,&pid3.SetPoint,&Moto_theta); //大于2
//			pid1.SetPoint = (int)Setpointtemp[0];
//			pid2.SetPoint = (int)Setpointtemp[1];
//			pid3.SetPoint = (int)Setpointtemp[2];
			
		   event = osMessageGet(MotoSetQueues,50);  //采用消息队列 TIM6中断每50ms发送消息 每秒脉冲数为 50ms*20*spd.SPD* 
			if (event.status == osEventMessage)
			{	
				MotoSetRe = event.value.p;
				pid1.SetPoint = MotoSetRe[0];
				pid2.SetPoint = MotoSetRe[1];
				pid3.SetPoint = MotoSetRe[2];
				if(MotoSetRe[3] == 3) //3个电机刹车 电机要反向运行的时候必须刹车！！
				{
					motoX_FB(4,2);
					continue; //中止本次循环
				}
				if(pid1.SetPoint > 0) motoX_FB(1,0);	
				else if(pid1.SetPoint < 0) motoX_FB(1,1);
				else motoX_FB(1,2);
				if(pid2.SetPoint > 0) motoX_FB(2,0);
				else if(pid2.SetPoint < 0) motoX_FB(2,1);
				else motoX_FB(2,2);
				if(pid3.SetPoint > 0) motoX_FB(3,0);				
				else if(pid3.SetPoint < 0) motoX_FB(3,1);
			   else motoX_FB(3,2);
				
				if(pid1.SetPoint<0) pid1.SetPoint = -pid1.SetPoint;
				if(pid2.SetPoint<0) pid2.SetPoint = -pid2.SetPoint;
				if(pid3.SetPoint<0) pid3.SetPoint = -pid3.SetPoint;

				spd.PWM_Duty1 = LocPIDCalc(spd.SPD1*80/209.0,&pid1);
				spd.PWM_Duty2 = LocPIDCalc(spd.SPD2*80/209.0,&pid2);
				spd.PWM_Duty3 = LocPIDCalc(spd.SPD3*80/209.0,&pid3);
				/***************************************************/ 
				if(spd.PWM_Duty1<0)
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	//				if(spd.PWM_Duty1>-100)
	//			 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, -spd.PWM_Duty1);
	//           	else 	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 100);		
				}
				else if(spd.PWM_Duty1>100)  
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 100);
					//PWM_Duty = 100;
				}
				else __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, spd.PWM_Duty1); 
				/***************************************************************/ 
				if(spd.PWM_Duty2<0)		
				 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);   
				
				else if(spd.PWM_Duty2>100)  
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 100);
					//PWM_Duty = 100;
				}
				else __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, spd.PWM_Duty2);
				/***************************************************************/  			
				if(spd.PWM_Duty3<0)		
				 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  
				
				else if(spd.PWM_Duty3>100)  
				{
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 100);
				}
				else __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, spd.PWM_Duty3);
			}
      }
	 //osDelay(25);
	}
}
