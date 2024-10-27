#include "Control.h"
#include "motor.h"
#include "mpu6050.h"
#include "tool.h"
#include "OLED.h"
extern MPU6050_Data mpu6050_data;

float Balance(float Angle, float Gyro)
{
    float Balance_Kp = 10, Balance_Kd = 2.5f; // 直立环PD参数
    float Angle_bias, Gyro_bias;
    float balance;
    Angle_bias = -7.0f - Angle; // 求出平衡的角度中值 和机械相关
    Gyro_bias = 0 - Gyro;
    balance = Balance_Kp * Angle_bias + Balance_Kd * Gyro_bias; // 计算平衡控制的电机PWM
    return balance;
}

void mian_control(void)
{
    float output;
    output = Balance(-mpu6050_data.Roll, -mpu6050_data.Gx); // 计算平衡控制的电机PWM
    output = clamp(output, -30, 30);                        // 限幅
    // printf("output=%f,roll=%f,rollrate=%f\r\n", output, mpu6050_data.FilteredRoll, mpu6050_data.RollRate);
    OLED_ShowFloatNum(0, 0, output, 2, 1, OLED_6X8);            // 显示输出值
    OLED_ShowFloatNum(0, 8, mpu6050_data.Roll, 2, 1, OLED_6X8); // 显示Roll值
    OLED_ShowFloatNum(0, 16, mpu6050_data.Gx, 2, 1, OLED_6X8);  // 显示RollRate值
    OLED_UpdateArea(0, 0, 36, 24);                              // 更新显示区
    Set_Motor_Velocity(MOTOR_A, output);                        // 设置电机速度
    Set_Motor_Velocity(MOTOR_B, output);                        // 设置电机速度
}