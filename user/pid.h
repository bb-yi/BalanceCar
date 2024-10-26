#ifndef __PID_H
#define __PID_H

typedef struct // 定义pid结构体
{
    float Target;     // 定义目标值
    float Actual;     // 定义真实值
    float err;        // 定义偏差值
    float err_last;   // 定义上一个偏差值
    float Kp, Ki, Kd; // 定义比例，积分，微分
    float integral;   // 定义积分值
    float Output;     // 定义电压值
} pid;
float PID_Control(pid *pid_ctrl, float Angle_Err);
#endif