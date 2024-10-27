/*更新时间 2024年9/17
中秋节、重庆理工大学
B站up：今天你也幸福了吗
B站私信我，欢迎讨论问题*/

#include "mpu6050.h"
#include "math.h"
#include "i2c.h"
#define I2C_HANDLE hi2c1 // 注意。我的板子I2C1给显示屏了，所以给陀螺仪的是I2C2，故用&hi2c2
// 定义6个int16_t的数据，一会用于对于接收的数据拼接
// int16_t是有负数的，uint16_t是没负数的
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

// 以下是函数
void MPU6050_Init(void) // 初始化
{
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(&I2C_HANDLE, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	// 注意。我的板子I2C1给显示屏了，所以给陀螺仪的是I2C2，故用&hi2c2

	if (check == 0x68) // 寄存器文档最后一页写了WHO_AM_I_REG就是0x68
	{
		// 电源管理1
		Data = 0x01;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// 电源管理2（这个写不写无所谓）
		Data = 0x00;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);

		// 滤波（这个写不写无所谓）
		Data = 0x06;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);

		// 采样频率分频器寄存器
		Data = 0x09;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// 加速度计配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// 陀螺仪配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&I2C_HANDLE, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}
MPU6050_Data mpu6050_data;
void MPU6050_Read_Accel(void) // 读取加速度
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&I2C_HANDLE, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	mpu6050_data.Ax = Accel_X_RAW / 2048.0;
	mpu6050_data.Ay = Accel_Y_RAW / 2048.0;
	mpu6050_data.Az = Accel_Z_RAW / 2048.0 - 0.5; // 可能需要根据硬件调整
}

void MPU6050_Read_Gyro(void) // 读取角速度
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&I2C_HANDLE, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	mpu6050_data.Gx = Gyro_X_RAW / 131.0f;
	mpu6050_data.Gy = Gyro_Y_RAW / 131.0f;
	mpu6050_data.Gz = Gyro_Z_RAW / 131.0f;
}

void MPU6050_Read_Result(void)
{
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();

	// 计算加速度法的欧拉角
	float roll_a = atan2(mpu6050_data.Ay, mpu6050_data.Az) * (180 / 3.1415f);
	float pitch_a = -atan2(mpu6050_data.Ax, mpu6050_data.Az) * (180 / 3.1415f);

	// 计算角速度法的欧拉角
	mpu6050_data.Yaw += mpu6050_data.Gz * 0.005;   // 更新yaw
	mpu6050_data.Roll += mpu6050_data.Gx * 0.005;  // 更新roll
	mpu6050_data.Pitch += mpu6050_data.Gy * 0.005; // 更新pitch

	// 一阶低通滤波
	const float alpha = 0.9; // 调整滤波器参数
	mpu6050_data.Roll = (1 - alpha) * roll_a + alpha * mpu6050_data.Roll;
	mpu6050_data.Pitch = (1 - alpha) * pitch_a + alpha * mpu6050_data.Pitch;

	// 更新yaw
	// 这里的yaw值在硬件上可能存在问题，需额外处理
}