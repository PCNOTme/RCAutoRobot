#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "Xunji.h"
#include "mpu6050.h"
//循迹C文件 控制步进电机 三个电机 未写

//#define mputest 1

void XunjiTask(void const * argument)
{
	for(;;)
	{
		osDelay(5);
		
	}
}

