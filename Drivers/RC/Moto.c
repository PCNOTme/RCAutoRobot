#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "Moto.h"
#include <math.h>

//电机驱动-PID控制C文件
//外部全局变量定义在Moto.h

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5; //tim1,2,5 编码器模式
extern TIM_HandleTypeDef htim8;//tim8 PWM模式
extern osSemaphoreId SPD_overHandle;

/*************************************/
//定义PID相关宏
// 这三个参数设定对电机运行影响非常大
/*************************************/
#define  P_DATA      3.2                                 //P参数
#define  I_DATA      1.1                                //I参数
#define  D_DATA      -0.15                              //D参数



//CCMRAM int16_t SPD1,SPD2,SPD3;
CCMRAM SPD spd; //轮子速度结构体
CCMRAM PID pid1;
CCMRAM PID pid2;
CCMRAM PID pid3;
//static PID CCMRAM sPID;
//static PID *sptr = &sPID;

//void IncPIDInit(void);
//int IncPIDCalc(int NextPoint);
//unsigned int LocPIDCalc(int NextPoint,PID *pidconfig);
	
//CCMRAM int32_t PWM_Duty;
/*************************************/
//功能：控制电机正反转
// num：电机号 1~3号电机 其他num 3个电机停止
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

void ThreeWheelVellControl(float Vx,float Vy, float angularVell,
	                        float *V1,float *V2,float *V3,float theta)
{
	#define AFA (60.0000000f)  //全向轮与y轴夹角
   #define L    (3.0000000f) //三轮底盘中心到轮中心的距离
	
	*V1 = (float)(-cosf((AFA + theta) / 180.0000000f*3.141592653f) * Vx - sinf((theta + AFA) / 180.0000000f*3.141592653f) * Vy + L * angularVell);
   *V3 = (float)(cosf(theta / 180.0000000f*3.141592653f) * Vx + sinf(theta /180.0000000f*3.141592653f) * Vy      + L * angularVell);
   *V2 = (float)(-cosf((AFA - theta) / 180.0000000f * 3.141592653f) * Vx + sinf((AFA - theta) / 180.0000000f*3.141592653f) * Vy + L * angularVell);

}

void IncPIDInit(PID *pidconfig) 
{
    pidconfig->LastError=0;            //Error[-1]
    pidconfig->PrevError=0;            //Error[-2]
    pidconfig->Proportion=P_DATA;      //比例常数 Proportional Const
    pidconfig->Integral=I_DATA;        //积分常数  Integral Const
    pidconfig->Derivative=D_DATA;      //微分常数 Derivative Const
    pidconfig->SetPoint=100;           //设定目标Desired Value
}
///********************增量式PID控制设计************************************/
//int IncPIDCalc(int NextPoint) 
//{
//  int iError,iIncpid;                                 //当前误差
//  iError=sptr->SetPoint-NextPoint;                    //增量计算
//  iIncpid=(sptr->Proportion * iError)                 //E[k]项
//              -(sptr->Integral * sptr->LastError)     //E[k-1]项
//              +(sptr->Derivative * sptr->PrevError);  //E[k-2]项
//              
//  sptr->PrevError=sptr->LastError;                    //存储误差，用于下次计算
//  sptr->LastError=iError;
//  return(iIncpid);                                    //返回增量值
//}

/********************位置式 PID 控制设计************************************/
int LocPIDCalc(int NextPoint,PID *pidconfig)
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

CCMRAM float Setpointtemp[3];
void MotoTask(void const * argument)
{
//变量初始化
	spd.SPD1 = 0;
	spd.SPD2 = 0;
	spd.SPD3 = 0;
   spd.PWM_Duty1 = 0;
	spd.PWM_Duty2 = 0;
	spd.PWM_Duty3 = 0;
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
	pid3.SetPoint = 0;  //1 :每10秒1圈 最大400	
	HAL_TIM_Base_Start_IT(&htim6);
	for(;;)
	{
	//采用二值信号量 TIM6中断每50ms释放信号量 每秒脉冲数为 50ms*20*spd.SPD* 
	if(SPD_overHandle != NULL){
		if(osSemaphoreWait(SPD_overHandle,100) == osOK) {
			if(spd.SPD1<0) spd.SPD1 = -spd.SPD1;
			if(spd.SPD2<0) spd.SPD2 = -spd.SPD2;
			if(spd.SPD3<0) spd.SPD3 = -spd.SPD3;
			
			Setpointtemp[0] = pid1.SetPoint;
			Setpointtemp[1] = pid2.SetPoint;
			Setpointtemp[2] = pid3.SetPoint;
			ThreeWheelVellControl(-30,-sqrtf(30),0,&Setpointtemp[0],&Setpointtemp[1],&Setpointtemp[2],0); //大于2 
			pid1.SetPoint = (int)Setpointtemp[0];
			pid2.SetPoint = (int)Setpointtemp[1];
			pid3.SetPoint = (int)Setpointtemp[2];
			
			if(pid1.SetPoint > 0) motoX_FB(1,0);
			else motoX_FB(1,1);
			if(pid2.SetPoint > 0) motoX_FB(2,0);
			else motoX_FB(2,1);
			if(pid3.SetPoint > 0) motoX_FB(3,0);
			else motoX_FB(3,1);
			
			if(pid1.SetPoint<0) pid1.SetPoint = -pid1.SetPoint;
			if(pid2.SetPoint<0) pid2.SetPoint = -pid2.SetPoint;
			if(pid3.SetPoint<0) pid3.SetPoint = -pid3.SetPoint;

			spd.PWM_Duty1 = LocPIDCalc(spd.SPD1*2/209.0,&pid1);
			spd.PWM_Duty2 = LocPIDCalc(spd.SPD2*2/209.0,&pid2);
			spd.PWM_Duty3 = LocPIDCalc(spd.SPD3*2/209.0,&pid3);
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
	osDelay(25);
	}
}
