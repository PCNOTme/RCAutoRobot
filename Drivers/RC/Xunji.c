#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "Xunji.h"
#include "mpu6050.h"
#include "Moto.h"
//循迹C文件 控制步进电机 三个电机 未写

extern UART_HandleTypeDef huart2;
//#define mputest 1
extern osPoolId  mpool;
osMessageQId MotoSetQueues; //电机速度数值设置队列 setpiont1
CCMRAM float Moto_p,Moto_angle,Moto_angularVell,Moto_theta;
uint8_t CMD[5] = {0}; //freertos在CCMRAM中运行 DMA变量必须在普通RAM
void XunjiTask(void const * argument)
{
	int motoset[4];
	motoset[3] = 0;
	Moto_p = 10;
	Moto_angle = -90;
	Moto_angularVell = 0;
	Moto_theta = 0;
	
	HAL_UART_Receive_DMA(&huart2,CMD,5);
	osMessageQDef(MotoSetQueues,10, uint32_t); //创建电机速度数值设置队列
   MotoSetQueues = osMessageCreate(osMessageQ(MotoSetQueues), NULL);
	for(;;)
	{
		//motoset = osPoolAlloc(mpool);
//		
		if(CMD[0] != 0)
		{
			 if(CMD[0] != 0xaa)//如果帧头错误，清缓存
			 {	
				HAL_UART_DMAStop(&huart2);
				HAL_UART_Receive_DMA(&huart2, CMD,5); 
				CMD[0]=0;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)"设置错误",8);
			 }
			 else
			 { 
				 if(CMD[4]== 0xc0)
				 {
					 
					 Moto_p = CMD[1];
					 Moto_angle = CMD[2];
					 motoset[3] = CMD[3];
					 HAL_UART_Transmit_DMA(&huart2,CMD,5);
					 CMD[4]=0x00;
				 }
			 }
		 }
		ThreeWheelVellControl2(&Moto_p,&Moto_angle,&Moto_angularVell,&motoset[0],&motoset[1],&motoset[2],&Moto_theta);		
		osMessagePut(MotoSetQueues, (uint32_t)motoset, 50);
		osDelay(50);
		//Moto_p++;Moto_angle++;
		
	}
}

