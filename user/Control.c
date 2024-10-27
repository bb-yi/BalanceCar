#include "Control.h"
#include "motor.h"
#include "mpu6050.h"
#include "tool.h"
#include "OLED.h"
extern MPU6050_t MPU6050;
extern MOTOR_Data motor_data;

float Balance(float target_angle)
{
    float Balance_Kp = 6.5f, Balance_Kd = 0.085f; // 直立环PD参数
    float Angle_err, Gyro_bias;
    float balance;
    Angle_err = -(target_angle + 9.0f - MPU6050.KalmanAngleX); // 求出平衡的角度中值 和机械相关
    Gyro_bias = MPU6050.Gx;
    balance = Balance_Kp * Angle_err + Balance_Kd * Gyro_bias; // 计算平衡控制的电机PWM
    return balance;
}

float Velocity_Control(int encoder_left, int encoder_right)
{
    float Integral_max = 10;

    float Velocity_Kp = 1.0f, Velocity_Ki = 0.00f; // 速度环PI参数

    static float velocity, Encoder_bias;
    static float Encoder_Integral;
    // if (Flag_Stop == 1)
    //     Encoder_Integral = 0; // 电机关闭后清除积分

    /*=================速度PI控制器===================*/
    Encoder_bias = (encoder_left + encoder_right);

    Encoder_Integral += Encoder_bias; // 积分出位移 积分时间：10ms
    if (Encoder_Integral > Integral_max)
        Encoder_Integral = Integral_max; // 积分限幅
    if (Encoder_Integral < -Integral_max)
        Encoder_Integral = -Integral_max;                                   // 积分限幅
    velocity = Encoder_bias * Velocity_Kp + Encoder_Integral * Velocity_Ki; // 速度控制
    return velocity;
}
void mian_control(void)
{
    float vel_output, output;
    vel_output = Velocity_Control(motor_data.Filtered_Velocity_A, motor_data.Filtered_Velocity_B);
    output = Balance(vel_output);                         // 计算平衡控制的电机PWM
    output = Abs(MPU6050.KalmanAngleX) > 15 ? 0 : output; // 限幅
    output = clamp(output, -30, 30);                      // 限幅
    // printf("vel_output=%f,output=%f\r\n", vel_output, output);
    // printf("output=%f,roll=%f,rollrate=%f\r\n", output, mpu6050_data.FilteredRoll, mpu6050_data.RollRate);
    // OLED_ShowFloatNum(0, 0, output, 2, 1, OLED_6X8);            // 显示输出值
    // OLED_ShowFloatNum(0, 8, mpu6050_data.Roll, 2, 1, OLED_6X8); // 显示Roll值
    // OLED_ShowFloatNum(0, 16, mpu6050_data.Gx, 2, 1, OLED_6X8);  // 显示RollRate值
    // OLED_UpdateArea(0, 0, 36, 24);                              // 更新显示区
    Set_Motor_Velocity(MOTOR_A, output); // 设置电机速度
    Set_Motor_Velocity(MOTOR_B, output); // 设置电机速度
}
