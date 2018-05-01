#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <math.h>
#include "Moto.h"
#include "Xunji.h"


//�������-PID����C�ļ�
//�ⲿȫ�ֱ���������Moto.h

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5; //tim1,2,5 ������ģʽ
extern TIM_HandleTypeDef htim8;//tim8 PWMģʽ

extern osSemaphoreId SPD_overHandle;


extern osMessageQId MotoSetQueues; //����ٶ���ֵ���ö��� setpiont1

/*************************************/
//����PID��غ�
// �����������趨�Ե������Ӱ��ǳ���
/*************************************/
#define  P_DATA      3.2                                 //P����
#define  I_DATA      1.1                                //I����
#define  D_DATA      0.15                              //D����


CCMRAM SPD spd; //�����ٶȽṹ��
CCMRAM PID pid1;
CCMRAM PID pid2;
CCMRAM PID pid3;


/*************************************/
//���ܣ����Ƶ������ת
// num������� 1~3�ŵ�� 4 3�����ɲ�� ����num 3�����ֹͣ
// fb :0��ת 1 ��ת 2 ֹͣ 3ɲ��
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
// ���ܣ�����ȫ���ƶ������˵Ŀ���
// Vx,Vy : �ƶ�ƽ̨��������ٶ� X�ᣬY��
// angularVell ��ƽ̨�Ƽ������ĵ�ת��
// V1,V2,V3 ������ȫ�������ٶ�
// theta : ȫ������y��н�AFA������ƫ����
/*************************************/

void ThreeWheelVellControl(float *Vx,float *Vy, float *angularVell,
	                        volatile int *V1,volatile int *V2,volatile int *V3,float *theta)
{
	#define AFA (60.0f)  //ȫ������y��н�
   #define L    (3.0f) //���ֵ������ĵ������ĵľ���
	
	*V1 = (int)(-cosf((AFA + *theta) / 180.0f*3.141592653f) * *Vx - sinf((*theta + AFA) / 180.0f*3.141592653f) * *Vy + L * *angularVell);
   *V3 = (int)(cosf(*theta / 180.00f*3.141592653f) * *Vx + sinf(*theta /180.0f*3.141592653f) * *Vy      + L * *angularVell);
   *V2 = (int)(-cosf((AFA - *theta) / 180.0f * 3.141592653f) * *Vx + sinf((AFA - *theta) / 180.0f*3.141592653f) * *Vy + L * *angularVell);

}

//������ϵ��ʾ
//p �����˳�ĳһ�����ٶ�
//angle �����˽Ƕ� ����60��
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
    pidconfig->Proportion=P_DATA;      //�������� Proportional Const
    pidconfig->Integral=I_DATA;        //���ֳ���  Integral Const
    pidconfig->Derivative=D_DATA;      //΢�ֳ��� Derivative Const
    pidconfig->SetPoint=0;           //�趨Ŀ��Desired Value
}

/********************����ʽPID�������************************************
int IncPIDCalc(int NextPoint) 
{
  int iError,iIncpid;                                 //��ǰ���
  iError=sptr->SetPoint-NextPoint;                    //��������
  iIncpid=(sptr->Proportion * iError)                 //E[k]��
              -(sptr->Integral * sptr->LastError)     //E[k-1]��
              +(sptr->Derivative * sptr->PrevError);  //E[k-2]��
              
  sptr->PrevError=sptr->LastError;                    //�洢�������´μ���
  sptr->LastError=iError;
  return(iIncpid);                                    //��������ֵ
}
***************************************************************************/


/********************λ��ʽ PID �������************************************/
int LocPIDCalc(float NextPoint,PID *pidconfig)
{ 
  int iError,dError;
  iError = pidconfig->SetPoint - NextPoint; //ƫ��
  pidconfig->SumError += iError; //����
  dError = iError - pidconfig->LastError; //΢��
  pidconfig->LastError = iError;
  return(pidconfig->Proportion * iError //������
  + pidconfig->Integral * pidconfig->SumError //������
  + pidconfig->Derivative * dError); //΢����
}

osPoolId  mpool;
osMessageQId MotoQueues; //�����������ֵ��ȡ���� spd1 spd2 spd3
void MotoTask(void const * argument)
{
//������ʼ��
	spd.SPD1 = 0;
	spd.SPD2 = 0;
	spd.SPD3 = 0;
   spd.PWM_Duty1 = 0;
	spd.PWM_Duty2 = 0;
	spd.PWM_Duty3 = 0;
	
//�����ڴ�ع����� 	
  osPoolDef(mpool,5,uint32_t); 	
  mpool = osPoolCreate(osPool(mpool));
//���������������ֵ��ȡ����
  osMessageQDef(MotoQueues,5, SPD);
  MotoQueues = osMessageCreate(osMessageQ(MotoQueues), NULL);
//PWM�������
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);	
//����������
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
	pid2.SetPoint = 0;  ///����ת������ �ڴ˴�
	pid3.SetPoint = 0;  //1 :ÿ4��1Ȧ ���80
	HAL_TIM_Base_Start_IT(&htim6);
	osEvent event;
	SPD *MotoSpdXRe;
	uint32_t *MotoSetRe;
	for(;;)
	{
		event = osMessageGet(MotoQueues,50);  //������Ϣ���� TIM6�ж�ÿ50ms������Ϣ ÿ��������Ϊ 50ms*20*spd.SPD* 
		if (event.status == osEventMessage)
		{
			MotoSpdXRe = event.value.p;
			spd.SPD1 = MotoSpdXRe->SPD1;
			spd.SPD2 = MotoSpdXRe->SPD2;
			spd.SPD3 = MotoSpdXRe->SPD3;
		   osPoolFree(mpool, MotoSpdXRe); //�ͷ�֮ǰ���ڴ�
		
	//���ö�ֵ�ź��� TIM6�ж�ÿ50ms�ͷ��ź��� ÿ��������Ϊ 50ms*20*spd.SPD* 
	 //if(SPD_overHandle != NULL){
	//	if(osSemaphoreWait(SPD_overHandle,100) == osOK) {
			if(spd.SPD1<0) spd.SPD1 = -spd.SPD1;
			if(spd.SPD2<0) spd.SPD2 = -spd.SPD2;
			if(spd.SPD3<0) spd.SPD3 = -spd.SPD3;
			
	//			Setpointtemp[0] = pid1.SetPoint;
	//			Setpointtemp[1] = pid2.SetPoint;
	//			Setpointtemp[2] = pid3.SetPoint;		
			//ThreeWheelVellControl2(&Moto_p,&Moto_angle,&Moto_angularVell,&pid1.SetPoint,&pid2.SetPoint,&pid3.SetPoint,&Moto_theta); //����2
//			pid1.SetPoint = (int)Setpointtemp[0];
//			pid2.SetPoint = (int)Setpointtemp[1];
//			pid3.SetPoint = (int)Setpointtemp[2];
			
		   event = osMessageGet(MotoSetQueues,50);  //������Ϣ���� TIM6�ж�ÿ50ms������Ϣ ÿ��������Ϊ 50ms*20*spd.SPD* 
			if (event.status == osEventMessage)
			{	
				MotoSetRe = event.value.p;
				pid1.SetPoint = MotoSetRe[0];
				pid2.SetPoint = MotoSetRe[1];
				pid3.SetPoint = MotoSetRe[2];
				if(MotoSetRe[3] == 3) //3�����ɲ�� ���Ҫ�������е�ʱ�����ɲ������
				{
					motoX_FB(4,2);
					continue; //��ֹ����ѭ��
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
