#include "Control.h"
#include "motor.h"
#include "mpu6050.h"
#include "tool.h"
#include "OLED.h"
extern MPU6050_t MPU6050;
extern MOTOR_Data motor_data;
float balance_angle = 9.5f;
float Balance(float target_angle)
{
    // float Balance_Kp = 6.5f, Balance_Kd = 0.085f; // 直立环PD参数
    float Balance_Kp = 10.0f, Balance_Kd = 0.45f; // 直立环PD参数
    float Angle_err, Gyro_bias;
    float balance;
    Angle_err = -(target_angle + balance_angle - MPU6050.KalmanAngleX); // 求出平衡的角度中值 和机械相关
    Gyro_bias = MPU6050.Gx;
    balance = Balance_Kp * Angle_err + Balance_Kd * Gyro_bias; // 计算平衡控制的电机PWM
    return balance;
}
static float Integral;
float Velocity_Control(float target_velocity, float encoder_left, float encoder_right)
{
    // float kp = 0.4500f, ki = 0.0001f; // 速度环PID参数
    float kp = 0.380f, ki = 0.0005f; // 速度环PID参数

    float integral_max = 10;
    float err_vel, output;
    err_vel = (target_velocity - (encoder_left + encoder_right)); // 计算速度误差
    Integral += err_vel;                                          // 积分
    Integral = clamp(Integral, -integral_max, integral_max);      // 限幅
    output = err_vel * kp + Integral * ki;                        // 输出
    return output;
}

uint8_t OutOfControl_flag = 0;
uint32_t start_time, now_time;
void mian_control(void)
{
    float output_max = 100;
    float vel_output, balance_output, output;
    vel_output = Velocity_Control(0, motor_data.Filtered_Velocity_A, motor_data.Filtered_Velocity_B);
    balance_output = Balance(vel_output);            // 计算平衡控制的电机PWM
    output = vel_output + balance_output;            // 计算总输出
    output = clamp(output, -output_max, output_max); // 限幅
    if (Abs(MPU6050.KalmanAngleX - balance_angle) < 25)
    {
    }
    else
    {
        output = 0;   // 限幅
        Integral = 0; // 积分清零
    }
    if (Abs(output) != output_max)
    {
        OutOfControl_flag = 0;
    }
    if (Abs(output) == output_max && OutOfControl_flag == 0)
    {
        OutOfControl_flag = 1;
        start_time = HAL_GetTick();
    }

    if (OutOfControl_flag == 1)
    {
        now_time = HAL_GetTick();
        if (now_time - start_time > 500)
        {
            OutOfControl_flag = 0;
            start_time = 0;
            output = 0;
            motor_stop();
        }
    }
    // printf("flag%d,start_time%d,now_time%d,output%f\n", OutOfControl_flag, start_time, now_time, output);
    Set_Motor_Velocity(MOTOR_A, output); // 设置电机速度
    Set_Motor_Velocity(MOTOR_B, output); // 设置电机速度
}
