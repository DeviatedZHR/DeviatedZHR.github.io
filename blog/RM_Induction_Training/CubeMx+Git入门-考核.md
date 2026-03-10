本次任务：结合**CubeMx**使用**HAL**库调试六轴**IMU**，得到**欧拉角**与四元数，并必要绘制曲线展示基本性能，最后将代码作为一个新建分支上传至自己的**Git**仓库。



## 时钟树配置
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726912450128-5919309c-0030-4d5f-82dc-ff53f1e9a8cb.png)

## 其他配置
### 配置硬件I2C
选用I2C1

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726912499057-055cdeb7-2660-4853-9f0c-34c7386986fc.png)

### 配置串口
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726912521343-798ce7fa-ed47-4d07-a481-e22a13a291aa.png)

由于要求<font style="background-color:#FBDE28;">将关键数据通过串口以恒定频率通过DMA发送到vofa+</font>，所以在此添加DMA的配置。



### 生成工程文件
修改好路径就可以产生代码了。（路径不能有中文名）

## 代码部分
### mpu6050
```c
#include "mpu6050.h" 
#include "stdio.h"
#include "i2c.h"

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Init(void)
{ 
  uint8_t res;
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Init(&hi2c1);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
  MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
  MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
  MPU_Set_Rate(200);						//设置采样率200Hz
  MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
  MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
  MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
  MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
  res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	printf("\r\nMPU6050:0x%2x\r\n",res);
  if(res==MPU_ADDR)//器件ID正确
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
    MPU_Set_Rate(50);						//设置采样率为50Hz
  }else 
		return 1;
  return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
 
//得到温度值
//返回值:温度值(扩大了100倍)
float MPU_Get_Temperature(void)
{
  unsigned char  buf[2]; 
  short raw;
  float temp;
  
  MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buf); 
  raw=(buf[0]<<8)| buf[1];  
  temp=(36.53+((double)raw)/340)*100;  
//  temp = (long)((35 + (raw / 340)) * 65536L);
  return temp/100.0f;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
  uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
 
 
//IIC连续写
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
  extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;
 
  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
  extern I2C_HandleTypeDef hi2c1;
  unsigned char R_Data=0;
  
  HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, &R_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return R_Data;		
}


int16_t gyro_raw[3],acc_raw[3];//原数据
float gyro_processed[3],acc_processed[3];//转化为标准单位的数据
float gyro_zero[3],acc_zero[3];//存储零漂

/* 读取MPU6050数据并加滤波 */
void MpuGetData(void)
{
	uint8_t i;
  uint8_t buffer[12];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU_READ, 0X3B, I2C_MEMADD_SIZE_8BIT, buffer, 6, 0xfff);				/* 读取加速度 */
	HAL_I2C_Mem_Read(&hi2c1, MPU_READ, 0x43, I2C_MEMADD_SIZE_8BIT, &buffer[6], 6, 0xfff);		/* 读取角速度 */

	//printf("buff:%d\r\n",buffer[1]);
	
	for(i=0;i<6;i++)
	{
		if(i<3){
			acc_raw[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);							/* 将数据整为16bit */
			acc_processed[i] = (float)acc_raw[i] / 32768 * 2 * 9.8;
		}else{
			gyro_raw[i-3] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);							/* 将数据整为16bit */
			gyro_processed[i-3] = (float)gyro_raw[i-3] / 32768 * 2000 / 57.3;
		}
	}	
	//printf("%f,%f,%f,%f,%f,%f\n",gyro_processed[0],gyro_processed[1],gyro_processed[2],acc_processed[0],acc_processed[1],acc_processed[2]);
}

//计算零漂
void Cal_Zero_Drift(void)
{
	uint16_t i,j;
	float gyrosum[3],accsum[3];
	for(i=0;i<3;i++)
	{
		gyrosum[i]=0;
		accsum[i]=0;
	}
	for(i=0;i<100;i++)
	{
		MpuGetData();
		for(j=0;j<3;j++)
		{
			gyrosum[j]+=gyro_processed[j];
			accsum[j]+=acc_processed[j];
		}
	}
	for(i=0;i<3;i++)
	{
		gyro_zero[i]=gyrosum[i]/100;
		acc_zero[i]=accsum[i]/100;
	}
}
```

参考内容：[【HAL库代码】之MPU6050_mpu650.h-CSDN博客](https://blog.csdn.net/baidu_39603247/article/details/107401744)



### Mahony算法
来源于大疆开源

```c
//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "Algorithm_Mahony.h"
#include "mpu6050.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	200.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 3.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void get_angle(float q0,float q1,float q2,float q3, float *yaw, float *pitch, float *roll)
{
	*yaw = atan2f(2.0f*(q0*q3+q1*q2), 2.0f*(q0*q0+q1*q1)-1.0f);
	*pitch = asinf(-2.0f*(q1*q3-q0*q2));
	*roll = atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f);
}


//====================================================================================================
// END OF CODE
//====================================================================================================

```

由于该函数自带滤波，所以无需考虑其他滤波。



### main函数定时器部分
```c
extern int16_t gyro_raw[3],acc_raw[3];
extern float gyro_processed[3],acc_processed[3];
extern float gyro_zero[3],acc_zero[3];//存储零漂
extern volatile float q0, q1, q2, q3;	
float Yaw,Pitch,Roll;

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim == (&htim2))	//如果定时器2触发中断
  {	
		MpuGetData();
		
		//注：z轴加速度零漂无法消除
		MahonyAHRSupdateIMU(gyro_processed[0]-gyro_zero[0],gyro_processed[1]-gyro_zero[1],gyro_processed[2]-gyro_zero[2],acc_processed[0]-acc_zero[0],acc_processed[1]-acc_zero[1],acc_processed[2]);
		
		get_angle(q0,q1,q2,q3,&Yaw,&Pitch,&Roll);
		
		//printf("%f,%f,%f,%f,%f,%f\n",gyro_processed[0],gyro_processed[1],gyro_processed[2],acc_processed[0],acc_processed[1],acc_processed[2]);
		//printf("%f,%f,%f\n",gyro_processed[0],gyro_processed[1],gyro_processed[2]);
		//printf("%f,%f,%f\n",acc_processed[0],acc_processed[1],acc_processed[2]);
		printf("%f,%f,%f\n",Yaw,Pitch,Roll);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
```



### DMA部分
#### DMA发送
通过查询论坛，发现H7板子不支持DMA不定长发送，所以只能考虑定长数据发送。

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726988682616-06c2fac3-50b9-4b3a-9245-238dd68fc568.png)

经过多次尝试仍然失败，一直在发送`\0`，导致串口不显示任何东西，经与其他同学商讨可能是H7系列板子自身问题。

即函数`HAL_UART_Transmit_DMA`存在问题。



#### DMA接收
配置rx的dma后需要写出回调函数

由于我看的这篇文章是自己手写的一个回调函数，尝试后发现效果还不错。



先在主函数部分写以下代码（放在`while(1)`前面）：

```c
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 255);     //设置DMA传输，将串口1的数据搬运到recvive_buff中，每次255个字节
```



在`usart.c`中如此配置

```c
// 声明外部变量 
extern uint8_t receive_buff[255];                                                  
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	// 停止本次DMA传输
    HAL_UART_DMAStop(&huart1);  
                                                       
    // 计算接收到的数据长度
    uint8_t data_length  = 255 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    
	// 测试函数：将接收到的数据打印出去
    printf("Receive Data(length = %d): ",data_length);
    HAL_UART_Transmit(&huart1,receive_buff,data_length,0x200);                     
    printf("\r\n");
    
	// 清零接收缓冲区
    memset(receive_buff,0,data_length);                                            
    data_length = 0;
    
    // 重启开始DMA传输 每次255字节数据
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 255);                    
}


void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{	// 判断是否是串口1
    if(USART1 == huart1.Instance)                                   
    {	// 判断是否是空闲中断
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   
        {	 // 清除空闲中断标志（否则会一直不断进入中断）
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                    
            printf("\r\nUART1 Idle IQR Detected\r\n");
            // 调用中断处理函数
            USAR_UART_IDLECallback(huart);                          
        }
    }
}
```

注：如果发现数据和长度不对应可能是vofa发送数据后默认追加`\r\n`的原因，如果将追加的长度来算的话是正确的。



找到`stm32h7xx_it.c`，写入以下代码：

```c
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	
	// 新添加的函数，用来处理串口空闲中断
  USER_UART_IRQHandler(&huart1);

  /* USER CODE END USART1_IRQn 1 */
}
```



然后打开串口就可以使用了。先关闭发送的内容，发送一些数字

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726995345690-b0c14233-c7ef-44a4-8216-6cdc122fc296.png)



可以看到有了反馈。



由于要求是接收数据后翻转led或蜂鸣器，而所给板子只有两个GND引脚并且全部被占用（MPU6050和usb转ttl），所以只能做回传。



<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726995699787-1e27b31c-920c-40aa-9d0c-54ebb3416166.png)

可以发现在发送mpu数据的时候仍然能够接收到信息。



参考文章：

[STM32 HAL库学习（四）：DMA之串口空闲中断_stm32串口dma空闲中断-CSDN博客](https://blog.csdn.net/la_fe_/article/details/100543141)



### flash部分
观察H7板数据手册，可知其有2MB的flash存储空间。

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1726998515292-e1231e3d-4cd3-4829-b9b8-67ce552c5e3f.png)



网上也有大佬讲解了H7板flash各函数的使用：

[【STM32H7教程】第70章 STM32H7的内部Flash基础知识和HAL库API - 硬汉嵌入式 - 博客园](https://www.cnblogs.com/armfly/p/12425884.html)



#### 配置
根据csdn上的教程以及手册配置了相关内容

```c
#ifndef __FLASH_H_
#define __FLASH_H_

#include "main.h"
#include "stm32h753xx.h" 

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
 
 
#define STM32_FLASH_SIZE 	1024 	 	//所选STM32的FLASH容量大小(单位为K)
    #if     STM32_FLASH_SIZE < 256      //设置扇区大小
    #define STM_SECTOR_SIZE     1024    //1K字节
    #else 
    #define STM_SECTOR_SIZE	    2048*64    //2K字节
#endif
 
#define STM32_FLASH_BASE            0x08000000 		//STM32 FLASH的起始地址
#define FLASH_USER_START_ADDR   ( STM32_FLASH_BASE + STM_SECTOR_SIZE * 3 ) //写Flash的地址，这里从第62页开始
#define FLASH_USER_END_ADDR     ( STM32_FLASH_BASE + STM_SECTOR_SIZE * 4 ) //写Flash的地址，这里以第64页结束
 
 
void Flash_Erase(void); 
void Flash_Write(u32 *pBuffer,u32 NumToWrite);
void Flash_Read(u32 *pBuffer,u32 NumToRead);
 
 
#endif
```



```c
#include "flash.h"
#include <stdio.h>
 
static FLASH_EraseInitTypeDef EraseInitStruct;
u32 PAGEError = 0;
 /**********************************************************************************
  * 函数功能: 页擦除
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无 
  */
void Flash_Erase(void)
{  	
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Sector    = FLASH_SECTOR_3;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_1;
    EraseInitStruct.NbSectors     = 1;
    
     if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)  
	 {
		 HAL_FLASH_Lock();  
		 printf(" Error...1\r\n");
         Error_Handler( );
	 }
}
 /**********************************************************************************
  * 函数功能: 数据写入
  * 输入参数: 写入数据缓存数组指针、写入数据数
  * 返 回 值: 无
  * 说    明：无 
  */    
void Flash_Write(u32 *pBuffer,u32  NumToWrite)
{
 
    u16  i=0;
    u32 Address = FLASH_USER_START_ADDR;
    HAL_FLASH_Unlock();	    //解锁
    Flash_Erase( );         //先擦除
                            //再写入
       printf("擦除完成，准备写入......\r\n");
     while ( (Address < FLASH_USER_END_ADDR) && (i<NumToWrite)  )    
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint32_t) &pBuffer[i]) == HAL_OK)
        {
            Address = Address + 4;  //地址后移4个字节
            i++;
        }
        else
		{  
			printf(" Error...2\r\n"); 
            Error_Handler( );            
		}
    }
  
    HAL_FLASH_Lock();   //上锁
 
 
}
 
 /**********************************************************************************
  * 函数功能: 数据读取
  * 输入参数: 读取数据缓存数组指针、读出数据数
  * 返 回 值: 无
  * 说    明：无
  */
void Flash_Read(u32  *pBuffer,u32  NumToRead)
{
    u16  i=0;
    u32 Address = FLASH_USER_START_ADDR;
    
    while ( (Address < FLASH_USER_END_ADDR) && (i<NumToRead)  )
  {
    pBuffer[i++]= *(__IO u32 *)Address;  
    Address = Address + 4;   //地址后移4个字节
  }
  
}
```



#### 注意
不同型号板子对应的HAL库内容不同，相关的`HAL_FLASH_Program`函数也会有所不同。由于这里输入问题导致我们测试时程序卡住，究其原因是那个函数第三个参数**应上传数据的地址而不是数据本身**。

参考对应HAL库函数

```c
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t FlashAddress, uint32_t DataAddress)
{
  HAL_StatusTypeDef status;
  __IO uint32_t *dest_addr = (__IO uint32_t *)FlashAddress;
  __IO uint32_t *src_addr = (__IO uint32_t*)DataAddress;
  uint32_t bank;
  uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  assert_param(IS_FLASH_PROGRAM_ADDRESS(FlashAddress));

  /* Process Locked */
  __HAL_LOCK(&pFlash);

#if defined (FLASH_OPTCR_PG_OTP)
  if((IS_FLASH_PROGRAM_ADDRESS_BANK1(FlashAddress)) || (IS_FLASH_PROGRAM_ADDRESS_OTP(FlashAddress)))
#else
  if(IS_FLASH_PROGRAM_ADDRESS_BANK1(FlashAddress))
#endif /* FLASH_OPTCR_PG_OTP */
  {
    bank = FLASH_BANK_1;
    /* Prevent unused argument(s) compilation warning */
    UNUSED(TypeProgram);
  }
#if defined (DUAL_BANK)
  else if(IS_FLASH_PROGRAM_ADDRESS_BANK2(FlashAddress))
  {
    bank = FLASH_BANK_2;
  }
#endif /* DUAL_BANK */
  else
  {
    return HAL_ERROR;
  }

  /* Reset error code */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, bank);

  if(status == HAL_OK)
  {
#if defined (DUAL_BANK)
    if(bank == FLASH_BANK_1)
    {
#if defined (FLASH_OPTCR_PG_OTP)
      if (TypeProgram == FLASH_TYPEPROGRAM_OTPWORD)
      {
        /* Set OTP_PG bit */
        SET_BIT(FLASH->OPTCR, FLASH_OPTCR_PG_OTP);
      }
      else
#endif /* FLASH_OPTCR_PG_OTP */
      {
        /* Set PG bit */
        SET_BIT(FLASH->CR1, FLASH_CR_PG);
      }
    }
    else
    {
      /* Set PG bit */
      SET_BIT(FLASH->CR2, FLASH_CR_PG);
    }
#else /* Single Bank */
#if defined (FLASH_OPTCR_PG_OTP)
      if (TypeProgram == FLASH_TYPEPROGRAM_OTPWORD)
      {
        /* Set OTP_PG bit */
        SET_BIT(FLASH->OPTCR, FLASH_OPTCR_PG_OTP);
      }
      else
#endif /* FLASH_OPTCR_PG_OTP */
      {
        /* Set PG bit */
        SET_BIT(FLASH->CR1, FLASH_CR_PG);
      }
#endif /* DUAL_BANK */

    __ISB();
    __DSB();

#if defined (FLASH_OPTCR_PG_OTP)
    if (TypeProgram == FLASH_TYPEPROGRAM_OTPWORD)
    {
      /* Program an OTP word (16 bits) */
      *(__IO uint16_t *)FlashAddress = *(__IO uint16_t*)DataAddress;
    }
    else
#endif /* FLASH_OPTCR_PG_OTP */
    {
      /* Program the flash word */
      do
      {
        *dest_addr = *src_addr;
        dest_addr++;
        src_addr++;
        row_index--;
     } while (row_index != 0U);
    }

    __ISB();
    __DSB();

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, bank);

#if defined (DUAL_BANK)
#if defined (FLASH_OPTCR_PG_OTP)
    if (TypeProgram == FLASH_TYPEPROGRAM_OTPWORD)
    {
      /* If the program operation is completed, disable the OTP_PG */
      CLEAR_BIT(FLASH->OPTCR, FLASH_OPTCR_PG_OTP);
    }
    else
#endif /* FLASH_OPTCR_PG_OTP */
    {
      if(bank == FLASH_BANK_1)
      {
        /* If the program operation is completed, disable the PG */
        CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
      }
      else
      {
        /* If the program operation is completed, disable the PG */
        CLEAR_BIT(FLASH->CR2, FLASH_CR_PG);
      }
    }
#else /* Single Bank */
#if defined (FLASH_OPTCR_PG_OTP)
    if (TypeProgram == FLASH_TYPEPROGRAM_OTPWORD)
    {
      /* If the program operation is completed, disable the OTP_PG */
      CLEAR_BIT(FLASH->OPTCR, FLASH_OPTCR_PG_OTP);
    }
    else
#endif /* FLASH_OPTCR_PG_OTP */
    {
      /* If the program operation is completed, disable the PG */
      CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
    }
#endif /* DUAL_BANK */
  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

  return status;
}

```



参考文章：[基于HAL库在STM32H7写入flash时死机，进入default_Handler卡死_嵌入式-CSDN问答](https://ask.csdn.net/questions/7831577?csdn_share_tail=%7B%22type%22%3A%22ask%22,%22rType%22%3A%22question%22,%22rId%22%3A%227831577%22,%22source%22%3A%222301_80127140%22%7D)



#### 结果
简单测试发现成功了

```c
uint32_t a=0x12,b;

//存储flash
Flash_Write(&a,1);
//读取flash
Flash_Read (&b,1);

printf("%x\n",b);
```

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727085442374-15fa5467-ddbf-4f83-a925-1c8976f32437.png)



但是问题的关键在于零漂和初始加速度是小数，而flash存储的是一个`uint32_t`的数。

参考网上的解决思路：[STM32内部flash存储小数——别样的C语言技巧-CSDN博客](https://blog.csdn.net/weixin_33790053/article/details/85977491?app_version=6.3.8&code=app_1562916241&csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%2285977491%22%2C%22source%22%3A%222301_80127140%22%7D&uLinkId=usr1mkqgl919blen&utm_source=app)

配合Flash写入和读取函数，“理论上”可以直接使用，但是输出却仍然有问题

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727100846046-4caa4fed-864a-4dc0-848c-d6e9d6d546d7.png)

使用Debug尝试排查出原因

发现问题出在这里

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727101124397-d71ccd31-0c15-4a67-b4a9-d797849b387a.png)

当第二次循环时，`HAL_FLASH_Program`返回了HAL_ERROR，这样使得NumToWrite大于1时报错。

论坛上也有很多诸如此类的问题，但是都未给出解决方案。



#### 解决方法
我尝试修改`Flash_Write`函数，除去擦除扇区和锁定解锁扇区以及循环部分，修改结果如下：

```c
void Flash_Write_Fixed(u32 *pBuffer,u32 Address)
{
    //HAL_FLASH_Unlock();	    //解锁
    //Flash_Erase( );         //先擦除
                            //再写入
    printf("擦除完成，准备写入......\r\n");
    
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint32_t) &pBuffer[0]) == HAL_OK)
    {
        Address = Address + 4;  //地址后移4个字节
    }
    else
		{  
				printf(" Error...2\r\n"); 
        Error_Handler( );            
		}
  
    //HAL_FLASH_Lock();   //上锁
}
```



尝试写入扇区

```c
HAL_FLASH_Unlock();	    //解锁
Flash_Erase( );         //先擦除
Flash_Write_Fixed((uint32_t *)&gyro_zero[0],FLASH_USER_START_ADDR+0);
HAL_FLASH_Lock();   //上锁

Flash_Read((uint32_t *)&ReadP,6);

printf("%f,%f,%f,%f,%f,%f\n",ReadP[0],ReadP[1],ReadP[2],ReadP[3],ReadP[4],ReadP[5]);
```

正常来讲，他只会写入一个值，但是编译后我却惊奇地发现他把六个数全读取到了。

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727102145255-4af8029e-c60b-4071-b4f5-e9327778955a.png)



然后我尝试只写入第二个数

```c
HAL_FLASH_Unlock();	    //解锁
Flash_Erase( );         //先擦除
Flash_Write_Fixed((uint32_t *)&gyro_zero[1],FLASH_USER_START_ADDR+0);
HAL_FLASH_Lock();   //上锁

Flash_Read((uint32_t *)&ReadP,6);

printf("%f,%f,%f,%f,%f,%f\n",ReadP[0],ReadP[1],ReadP[2],ReadP[3],ReadP[4],ReadP[5]);
```

然后他读出了后面五个数据

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727102255958-a896a5a9-9e60-4fa1-ab35-6f92fe4b0c54.png)



然后我提出一个设想：**Flash写入第一个数的地址时，会将其相邻元素地址依次写入**；**此外，在同一行声明的数组，其存储空间是连续的**。这体现在我写入的是角速度的首地址，却能把加速度的数都读出来（角速度零漂和加速度零漂存储在两个不同数组中）



下面对我的猜想进行验证：

修改数组大小，使角速度和加速度存储有间隔

```c
float gyro_zero[4],acc_zero[4];//存储零漂
```

然后再按照第一次输入的代码尝试：

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727102559185-6fefff9a-9d70-4c71-979c-9f55a4bb51a0.png)

可以发现他读取到了未赋值的数组元素，这样可以证明上述存储的数组地址是连续的。



那么应当如何使用H7板的Flash呢？

只需要**将要存储的数据都放在同一个数组里面**即可。这样就能保证每个数据的地址都是连续的。

```c
float SaveP[6]={gyro_zero[0],gyro_zero[1],gyro_zero[2],acc_zero[0],acc_zero[1],acc_zero[2]},ReadP[6];

HAL_FLASH_Unlock();	    //解锁
Flash_Erase( );         //先擦除
Flash_Write_Fixed((uint32_t *)&SaveP,FLASH_USER_START_ADDR+0);
HAL_FLASH_Lock();   //上锁

Flash_Read((uint32_t *)&ReadP,6);

printf("%f,%f,%f,%f,%f,%f\n",ReadP[0],ReadP[1],ReadP[2],ReadP[3],ReadP[4],ReadP[5]);
```

输出结果：

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2024/png/39231470/1727102744911-4ade589a-bf65-419e-b9fe-a57d1ea3cae9.png)



这样Flash部分内容就完成了。

## 效果视频
陀螺仪部分：

[此处为语雀卡片，点击链接查看](about:blank#Cdy64)





## git使用
先给出一些git常用代码：

```git
git init                    #把当前目录变成git可以管理的仓库
git clone git地址             #克隆项目
git add readme.txt          #添加一个文件，也可以添加文件夹
git add -A                  #添加全部文件
git rm test.txt             #删除一个文件，也可以删除文件夹
git commit -a -m “some commit” #提交修改
git status                  #查看是否还有未提交
git log                     #查看最近日志
git reset --hard HEAD^      #版本回退一个版本
git reset --hard HEAD^^     #版本回退两个版本
git reset --hard HEAD~100   #版本回退多个版本
git remote add origin +地址 #远程仓库的提交（第一次链接）
git push -u origin master   #仓库关联
git push                    #远程仓库的提交（第二次及之后）
git fetch                   #从远程获取代码库
git tag xxx                 #打tag
git tag                     #显示所有tag
git push --tag              #提交tag
git branch -a               #显示所有分支
git checkout 分支名        #切换分支
git merge git分支             #合并分支
```



在gitee上新建分支后，将仓库于分支关联。

注意先切换至对应分支

```git
git checkout  -b xxx
```

然后上传即可。



如果上传失败，多输入几次以下代码

```git
git add .
git commit -m '提交本地代码并且获取最新代码'
```

push之前先pull一下

```git
git pull origin xxx --allow-unrelated-histories
```

出现编辑界面直接 :wq 即可

```git
git push -u origin xxx
```



## 代码下载
代码已上传到gitee：[zhr/HERO_2024_Task](https://gitee.com/Herobrine12345619/hero_2024_-task/tree/%E7%AC%AC%E4%B8%80%E6%AC%A1%E4%BB%BB%E5%8A%A1/)

## 参考教程&开源
[GitHub - Staok/IMU-study: 对常见IMU芯片的原理、驱动和数据融合算法整理，以区分某度、某坛上面碎片化严重到影响入坑的乱象](https://github.com/Staok/IMU-study)

[gitee使用教程，创建项目仓库并上传代码 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/537008417#:~:text=%E4%B8%80%E3%80%81%E5%85%B3%E4%BA%8Egitee)

[[Git] Git将项目上传到指定项目的指定分支_git上传指定分支-CSDN博客](https://blog.csdn.net/GxDong_/article/details/107675063)

