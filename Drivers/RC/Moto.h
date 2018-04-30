#ifndef __MOTO_H
#define __MOTO_H
//#define CCMRAM  __attribute__((at(CCMDATARAM_BASE))) //ʹ��64KB CCMRAM

typedef struct //�������������ٶȽṹ��
{
  int16_t SPD1;
  int16_t SPD2;
  int16_t SPD3;
  int32_t PWM_Duty1;
  int32_t PWM_Duty2;
  int32_t PWM_Duty3;
}SPD;

//����PID�ṹ��
typedef struct 
{
  __IO int      SetPoint;                                 //�趨Ŀ�� Desired Value
  __IO long     SumError;                                 //����ۼ�
  __IO double   Proportion;                               //�������� Proportional Const
  __IO double   Integral;                                 //���ֳ��� Integral Const
  __IO double   Derivative;                               //΢�ֳ��� Derivative Const
  __IO int      LastError;                                //Error[-1]
  __IO int      PrevError;                                //Error[-2]
}PID;

extern SPD spd; //�����ⲿȫ�ֽṹ��
extern PID pid1;
extern PID pid2;
extern PID pid3;

//extern int16_t SPD1,SPD2,SPD3;

void MotoTask(void const * argument);
void IncPIDInit(PID *pidconfig) ;
int IncPIDCalc(int NextPoint);
int LocPIDCalc(int NextPoint,PID *pidconfig);
void motoX_FB(uint8_t num,uint8_t fb);
//void ThreeWheelVellControl(float *Vx,float *Vy, float *angularVell,
//	                        float *V1,float *V2,float *V3,float *theta);
void ThreeWheelVellControl(float Vx,float Vy, float angularVell,
	                        float *V1,float *V2,float *V3,float theta);
#endif 

