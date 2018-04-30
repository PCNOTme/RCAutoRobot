#ifndef __MOTO_H
#define __MOTO_H
//#define CCMRAM  __attribute__((at(CCMDATARAM_BASE))) //使用64KB CCMRAM

typedef struct //定义三个轮子速度结构体
{
  int16_t SPD1;
  int16_t SPD2;
  int16_t SPD3;
  int32_t PWM_Duty1;
  int32_t PWM_Duty2;
  int32_t PWM_Duty3;
}SPD;

//定义PID结构体
typedef struct 
{
  __IO int      SetPoint;                                 //设定目标 Desired Value
  __IO long     SumError;                                 //误差累计
  __IO double   Proportion;                               //比例常数 Proportional Const
  __IO double   Integral;                                 //积分常数 Integral Const
  __IO double   Derivative;                               //微分常数 Derivative Const
  __IO int      LastError;                                //Error[-1]
  __IO int      PrevError;                                //Error[-2]
}PID;

extern SPD spd; //定义外部全局结构体
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

