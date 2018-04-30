#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "pidcontoller.h"



//绝对式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

 PID->errI += PID->errNow; //误差积分，用于ki控制

 if(PID->errILim != 0)	   //微分上限和下限
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

 PID->errOld = PID->errNow;	//保存现在的误差
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}


/*******************************************************************************************************/



//增量式PID算法
void PID_IncrementMode(PID_IncrementType* PID)
{
 float dErrP, dErrI, dErrD;
 
 if(PID->kp < 0)    PID->kp = -PID->kp;
 if(PID->ki < 0)	PID->ki = -PID->ki;
 if(PID->kd < 0)    PID->kd = -PID->kd;

 dErrP = PID->errNow - PID->errOld1;

 dErrI = PID->errNow;

 dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

 PID->errOld2 = PID->errOld1; //二阶误差微分
 PID->errOld1 = PID->errNow;  //一阶误差微分

 /*增量式PID计算*/
 PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
 
 if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;

 else PID->ctrOut += PID->dCtrOut;
}


/*****************************************电机速度环伺服***********************************************/

//速度
PID_AbsoluteType PID_ControlA;//定义PID算法的结构体
PID_AbsoluteType PID_ControlB;//定义PID算法的结构体

//横向距离
PID_AbsoluteType PID_ControlJL1;
PID_AbsoluteType PID_ControlJL2;

int32_t User_PidSpeedControlA(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlA.errNow = SpeedTag - spnow; //计算并写入速度误差
   	
   PID_ControlA.kp      = 2.22;             //写入比例系数为15
   PID_ControlA.ki      = 0.785;              //写入积分系数为5
   PID_ControlA.kd      = 1.196;              //写入微分系数为5
   PID_ControlA.errILim = 1000;           //写入误差积分上限为1000 下限为-1000

   PID_AbsoluteMode(&PID_ControlA);       //执行绝对式PID算法
	
   return  PID_ControlA.ctrOut;         //读取控制值

}
//extern uint8_t aa[2];
int32_t User_PidSpeedControlB(int32_t SpeedTag,int32_t spnow)
{
//	PID_ControlB.kp      = 2.22;             //写入比例系数为15 2.58
//   PID_ControlB.ki      = 0.785;              //写入积分系数为5
//   PID_ControlB.kd      = 1.196; 
//  PID_ControlB.kp      = 2.78;             //写入比例系数为15

//   PID_ControlB.ki      = 1.26;              //写入积分系数为5
//   PID_ControlB.kd      = 0.5;              //写入微分系数为5
//  PID_ControlB.errILim = 1000; 
   PID_ControlB.errNow = SpeedTag - spnow; //计算并写入速度误差
   	
   PID_ControlB.kp      = 2.22;             //写入比例系数为15 2.58
   PID_ControlB.ki      = 0.785;              //写入积分系数为5
   PID_ControlB.kd      = 1.196;              //写入微分系数为5
   PID_ControlB.errILim = 1000;           //写入误差积分上限为1000 下限为-1000

   PID_AbsoluteMode(&PID_ControlB);       //执行绝对式PID算法
	
   return  PID_ControlB.ctrOut;         //读取控制值

}

int32_t User_PidSpeedControlJL1(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlJL1.errNow = SpeedTag - spnow; //计算并写入速度误差
   	
   PID_ControlJL1.kp      = 10;             //写入比例系数为15
   PID_ControlJL1.ki      = 5;              //写入积分系数为5
   PID_ControlJL1.kd      = 0;              //写入微分系数为5
   PID_ControlJL1.errILim = 2;           //写入误差积分上限为1000 下限为-1000

   PID_AbsoluteMode(&PID_ControlJL1);       //执行绝对式PID算法
	
   return  PID_ControlJL1.ctrOut;         //读取控制值

}

int32_t User_PidSpeedControlJL2(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlJL2.errNow = SpeedTag - spnow; //计算并写入速度误差
   	
   PID_ControlJL2.kp      = 10;             //写入比例系数为15
   PID_ControlJL2.ki      = 5;              //写入积分系数为5
   PID_ControlJL2.kd      = 0;              //写入微分系数为5
   PID_ControlJL2.errILim = 2;           //写入误差积分上限为1000 下限为-1000

   PID_AbsoluteMode(&PID_ControlJL2);       //执行绝对式PID算法
	
   return  PID_ControlJL2.ctrOut;         //读取控制值

}


