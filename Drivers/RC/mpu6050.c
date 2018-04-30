#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include <math.h>
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
/**************************************************************************/
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (100)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

#define mputest  (0) // 1 ������λ������ 0 �ر���λ������

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId DMP_overHandle;  //��ʼ����ֵ�ź���
CCMRAM float Pitch,Roll,Yaw;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if (GPIO_Pin == GPIO_PIN_5)
	 {
		 if(DMP_overHandle!=NULL) osSemaphoreRelease(DMP_overHandle); //�ͷŶ�ֵ�ź���
	 }
}
void mpu6050Task(void const * argument)
{
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //��StartDefaultTask�����ر��ⲿ�ж� �����ڿ����ж�
	for(;;)
	{
		if(DMP_overHandle != NULL){
			if(osSemaphoreWait(DMP_overHandle,50) == osOK) {
		
		     Read_DMP(&Pitch,&Roll,&Yaw);
#if ( mputest == 1 )
				uint8_t out[8];
	         int16_t t[3];
				t[0] = Yaw*100;
				t[1] = Pitch*100;
				t[2] = Roll*100;
				out[0]=0xAA;   //֡ͷ
				out[1]=t[0]>>8;   //��8λ
				out[2]=t[0];       //��8λ
				out[3]=t[1]>>8;
				out[4]=t[1];
				out[5]=t[2]>>8;
				out[6]=t[2]; 
				out[7]=0x55;  //֡β				
				printf("%s",out);
#endif
			}
			 
		}
		osDelay(5);
		
	}
}
/**
  * ��������: I2Cͨ�Ŵ���������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: һ����I2Cͨ�ų�ʱʱ���øú���
  * ���ִ����� void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) ���i2c����
  */
void I2C_MPU6050_Error (void)
{
	//GPIO_InitTypeDef GPIO_InitStruct;
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
//	HAL_GPIO_WritePin(GPIOB, 6, GPIO_PIN_SET); 
   
  /* ����ʼ��I2Cͨ������ */
  HAL_I2C_DeInit(&hi2c1);
	 //__HAL_RCC_GPIOB_CLK_ENABLE();
//  	 GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;      //����ԭ��
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   //GPIO����Ϊ���
//    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;         //ǿ����
//    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
//	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7) == GPIO_PIN_RESET)
//	{ //i2c stop�ź�
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); 
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//		osDelay(1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); 
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 
//		osDelay(1);
//	}
//		for(uint8_t i = 0;i<9;i++) 
//	   {
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);
//			osDelay(1);
//		}
  /* ���³�ʼ��I2Cͨ������*/
  HAL_I2C_Init(&hi2c1);
#if ( mputest == 1 ) 
	printf("MPU6050 I2Cͨ�ų�ʱ������ ��������I2C...\n");
#endif
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	osDelay(1);
	//if(HAL_I2C_Master_Transmit(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	if(HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,len,1) != HAL_OK)
	{
      I2C_MPU6050_Error();
		osDelay(1);
		if(HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,len,1) != HAL_OK)
		return -1;
	}
	return 0;
	
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	osDelay(1);
	//if(HAL_I2C_Master_Receive(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	if(HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)buf,len,1)!= HAL_OK)
	{
      I2C_MPU6050_Error();
		osDelay(1);
		if(HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)buf,len,1)!= HAL_OK)
		return -1;
	}
	return 0;
}

uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t * data)
{
	if(HAL_I2C_Mem_Write(&hi2c1,dev,reg,I2C_MEMADD_SIZE_8BIT,data,(uint16_t)length,0xff)!= HAL_OK)
	{
      I2C_MPU6050_Error();
	   return 0;
	}
	else return 1;
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	if(HAL_I2C_Mem_Read(&hi2c1,dev,reg,I2C_MEMADD_SIZE_8BIT,data,(uint16_t)length,0xff)!= HAL_OK)
	{
      I2C_MPU6050_Error();
	   return 1;
	}
	else
	return 0;
}

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(&hi2c1,I2C_Addr,addr,I2C_MEMADD_SIZE_8BIT,&data,1,0xff)!= HAL_OK)
	{
      I2C_MPU6050_Error();
	   return 0;
	}
	else
	return data;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{
	    uint8_t b;
	 
    if (HAL_I2C_Mem_Write(&hi2c1,dev,reg,I2C_MEMADD_SIZE_8BIT,&b,1,1) == 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteBytes(dev, reg,1,&b);
    } 
	 else
	 {
		  I2C_MPU6050_Error();
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
uint8_t IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data){
    uint8_t b;
	if(HAL_I2C_Mem_Read(&hi2c1,dev,reg,I2C_MEMADD_SIZE_8BIT,&b,1,1)!= HAL_OK)
	{
      I2C_MPU6050_Error();
	   return 0;
	}
   b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteBytes(dev, reg,1,&b);
}

/********************************************************************/

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*��������:	    ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����
*******************************************************************************/

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
		unsigned char i ;
		int32_t sum=0;
		for(i=1;i<10;i++){	//FIFO ����
			MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
			MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
			MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
			MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
			MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
			MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
		MPU6050_FIFO[1][9]=ay;
		MPU6050_FIFO[2][9]=az;
		MPU6050_FIFO[3][9]=gx;
		MPU6050_FIFO[4][9]=gy;
		MPU6050_FIFO[5][9]=gz;

		sum=0;
		for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
			sum+=MPU6050_FIFO[0][i];
		}
		MPU6050_FIFO[0][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
			sum+=MPU6050_FIFO[1][i];
		}
		MPU6050_FIFO[1][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
			sum+=MPU6050_FIFO[2][i];
		}
		MPU6050_FIFO[2][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
			sum+=MPU6050_FIFO[3][i];
		}
		MPU6050_FIFO[3][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
			sum+=MPU6050_FIFO[4][i];
		}
		MPU6050_FIFO[4][10]=sum/10;

		sum=0;
		for(i=0;i<10;i++){
			sum+=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[5][10]=sum/10;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				enabled =1   ˯��
			    enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_testConnection(void)
*��������:	    ���MPU6050 �Ƿ��Ѿ�����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_initialize(void) {
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-1000��ÿ��
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
    MPU6050_setSleepEnabled(0); //���빤��״̬
	 MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	 MPU6050_setI2CBypassEnabled(0);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
}




/**************************************************************************
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DMP_Init(void)
{ 
   uint8_t temp[1]={0};
	struct int_param_s int_param_s;
   if(i2cRead(0xD0,0x75,1,temp) == 0)
	 printf("mpu online ......\r\n");
//	else printf("mpu_set_sensor uncomplete ......\r\n");
	//if(temp[0]!=0x68)NVIC_SystemReset();
	
	if(!mpu_init(&int_param_s))
  {
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  	 printf("mpu_set_sensor complete ......\r\n");
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  	 printf("mpu_configure_fifo complete ......\r\n");
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  	 printf("mpu_set_sample_rate complete ......\r\n");
	  if(!dmp_load_motion_driver_firmware())
	  	printf("dmp_load_motion_driver_firmware complete ......\r\n");
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  	 printf("dmp_set_orientation complete ......\r\n");
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))
	  	 printf("dmp_enable_feature complete ......\r\n");
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  	 printf("dmp_set_fifo_rate complete ......\r\n");
	  run_self_test();
	  if(!mpu_set_dmp_state(1))
	  	 printf("mpu_set_dmp_state complete ......\r\n");
  }
  else NVIC_SystemReset();
}
/**************************************************************************
�������ܣ���ȡMPU6050����DMP����̬��Ϣ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
CCMRAM float q0,q1,q2,q3;
uint8_t Read_DMP(float* Pitch,float* Roll,float* Yaw)
{	
		short gyro[3], accel[3], sensors;
	  q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	  unsigned long sensor_timestamp;
		unsigned char more;
		long quat[4];
				if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) return 1;		
				if (sensors & INV_WXYZ_QUAT)
				{    
					 q0=quat[0] / q30;
					 q1=quat[1] / q30;
					 q2=quat[2] / q30;
					 q3=quat[3] / q30;
					 *Pitch = (float)asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f; 	
					 *Roll = (float)atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f; // roll
					 *Yaw = (float)atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3f;
					 return 0;
				}	
				else 
					return 2;
				
}
uint8_t Read_DMPt(short gyro[3], short accel[3],long quat[4],unsigned long sensor_timestamp,short sensors,unsigned char more)
{
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) return 1;
	else return 0;
}
/**************************************************************************
�������ܣ���ȡMPU6050�����¶ȴ���������
��ڲ�������
����  ֵ�������¶�
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Read_Temperature(void)
{	   
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;
		Temp=(36.53f+Temp/340)*10;
	  return (int)Temp;
}
//------------------End of File----------------------------
