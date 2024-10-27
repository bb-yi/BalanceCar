#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "MPU6050.h"

#define DEFAULT_MPU_HZ (100)
#define Q30 (1073741824.0f)
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
										  0, -1, 0,
										  0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
/*
Pitch（俯仰）：绕X轴的旋转。在飞行器姿态控制中，pitch 描述了飞行器围绕其前后轴的倾斜程度。正的pitch角意味着飞行器的鼻子向上倾斜，而负的pitch角则表示鼻子向下倾斜。

Roll（横滚）：绕Y轴的旋转。roll 描述了飞行器围绕其左右轴的旋转。正的roll角意味着飞行器在水平面上向左滚动，而负的roll角表示向右滚动。

Yaw（偏航）：绕Z轴的旋转。yaw 描述了飞行器围绕垂直轴的旋转。正的yaw角意味着飞行器的鼻子相对于其当前方向逆时针转动，而负的yaw角表示顺时针转动。
*/
static unsigned short inv_row_2_scale(const signed char *row)
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
		b = 7; // error
	return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
	const signed char *mtx)
{
	unsigned short scalar;

	/*
	   XYZ  010_001_000 Identity Matrix
	   XZY  001_010_000
	   YXZ  010_000_001
	   YZX  000_010_001
	   ZXY  001_000_010
	   ZYX  000_001_010
	 */

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;

	return scalar;
}

static int run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3)
	{
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
	}
	else
	{
		return -1;
	}
	return 0;
}

int MPU6050_DMP_Init(void)
{
	int result;
	struct int_param_s int_param;
	result = mpu_init(&int_param); // MPU6050 初始化失败。这可能由于 I2C 通信问题、设备未连接或内部设置错误等原因导致。
	if (result != 0)
	{
		return -1;
	}
	result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); // 设置传感器失败。这可能是由于指定的传感器不支持或硬件问题。
	if (result != 0)
	{
		return -2;
	}
	result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); // FIFO 配置失败。这可能是由于配置参数错误或通信问题。
	if (result != 0)
	{
		return -3;
	}
	result = mpu_set_sample_rate(DEFAULT_MPU_HZ); // 设置采样率失败。这可能是因为指定的采样率超出设备支持的范围。
	if (result != 0)
	{
		return -4;
	}
	result = dmp_load_motion_driver_firmware(); // DMP 固件加载失败。这可能是因为固件文件损坏或不兼容。
	if (result != 0)
	{
		return -5;
	}
	result = dmp_set_orientation(
		inv_orientation_matrix_to_scalar(gyro_orientation)); // 设置 DMP 方向失败。可能是因为方向参数不正确。
	if (result != 0)
	{
		return -6;
	}
	result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
								DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
								DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL); // 设置 DMP 方向失败。可能是因为方向参数不正确。
	if (result != 0)
	{
		return -7;
	}
	result = dmp_set_fifo_rate(DEFAULT_MPU_HZ); // 设置 DMP FIFO 率失败。这可能是由于指定的 FIFO 率不被支持。
	if (result != 0)
	{
		return -8;
	}
	result = run_self_test(); // 自检失败。这表示 MPU6050 未能通过自检，可能存在硬件问题。
	if (result != 0)
	{
		return -9;
	}
	result = mpu_set_dmp_state(1); // 启用 DMP 状态失败。这表示无法成功启用 DMP，可能是由于先前步骤中的错误或状态未正确设置。
	if (result != 0)
	{
		return -10;
	}
	return 0;
}

int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw, short *gx, short *gy, short *gz, short *ax, short *ay, short *az)
{
	float q0 = 0.0f;
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;
	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
	{
		return -1;
	}
	if (sensors & INV_WXYZ_QUAT)
	{
		q0 = quat[0] / Q30;
		q1 = quat[1] / Q30;
		q2 = quat[2] / Q30;
		q3 = quat[3] / Q30;

		*pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
		*roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
		*yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
		*gx = gyro[0];
		*gy = gyro[1];
		*gz = gyro[2];
		*ax = accel[0];
		*ay = accel[1];
		*az = accel[2];
	}
	else
	{
		return -2;
	}
	return 0;
}
MPU6050_Data mpu6050_data;
float Filtered_alpha = 0.7;
int MPU6050_Get_Data(void)
{
	int result;
	result = MPU6050_DMP_Get_Data(&mpu6050_data.Pitch, &mpu6050_data.Roll, &mpu6050_data.Yaw, &mpu6050_data.GyroX, &mpu6050_data.GyroY, &mpu6050_data.GyroZ, &mpu6050_data.AccX, &mpu6050_data.AccY, &mpu6050_data.AccZ);
	mpu6050_data.RollRate = mpu6050_data.GyroX / 131.0f;
	mpu6050_data.PitchRate = mpu6050_data.GyroY / 131.0f;
	mpu6050_data.YawRate = mpu6050_data.GyroZ / 131.0f;
	mpu6050_data.FilteredRoll = mpu6050_data.Roll * Filtered_alpha + mpu6050_data.FilteredRoll * (1.0f - Filtered_alpha);
	mpu6050_data.FilteredPitch = mpu6050_data.Pitch * Filtered_alpha + mpu6050_data.FilteredPitch * (1.0f - Filtered_alpha);
	mpu6050_data.FilteredYaw = mpu6050_data.Yaw * Filtered_alpha + mpu6050_data.FilteredYaw * (1.0f - Filtered_alpha);

	return result;
}
