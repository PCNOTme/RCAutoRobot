#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "BJDJ.h"


//步进电机-抛球C文件
//外部全局变量定义在Moto.h


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
CCMRAM uint32_t PulseNum_Over;

/*
********************************
//函数功能：输出任意频率、任意数 **
//          量的脉冲            **
//Cycle：脉冲周期  *us          **
//Pulse_Num:脉冲数量 >1         **
//定时器主从模式 主：TIM3 CH1    **
//         从：TIM4 外部触发模式 **
// ！！！ TIM4 分频必须设置为零  **
// TIM3分频84-1 方可正常工作     **
*********************************
*/
void Pulse_output(uint32_t Cycle,uint32_t Pulse_Num)
{
	uint32_t pul;
   pul = Pulse_Num-1;
	__HAL_TIM_SetAutoreload(&htim4,pul); //Period
   //TIM4->ARR = pul; 
	__HAL_TIM_SetCounter(&htim4,0); //计数器计数清零
	//TIM4->CNT = 0;   
	__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
	PulseNum_Over = 1;
	HAL_TIM_Base_Start_IT(&htim4);
	//TIM3->ARR = Cycle-1;
	__HAL_TIM_SetAutoreload(&htim3,Cycle-1); //Period
	//TIM3->CCR1 = Cycle/2;  
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Cycle/2);//Pulse 占空比
	while(PulseNum_Over == 1) osDelay(1);
}

	
void test()
{  PulseNum_Over = 0;
	//HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 2500); 
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET); ///DIR 低电平为正转 高点平反转
	osDelay(3000);
	while(1)
	{
	  HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
	  Pulse_output(260,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
	  Pulse_output(260,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
	  Pulse_output(240,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
	  Pulse_output(230,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
	  Pulse_output(220,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
	  Pulse_output(210,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
	  Pulse_output(190,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
	  Pulse_output(180,1600);
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
	  Pulse_output(120,1600);
		//osDelay(2000);
	  //Pulse_output(100,1600);
	  //Pulse_output(100,6400);
	  Pulse_output(100,3200*5);
		Pulse_output(90,3200*5);
		Pulse_output(90,400);
		
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 1000);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	   //Pulse_output(45,320000);
		osDelay(2000);
		while(1){osDelay(1);}
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
		
	}
}

