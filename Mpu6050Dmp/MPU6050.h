#ifndef __MPU6050_H
#define __MPU6050_H
#include "main.h"
typedef struct
{
    int res;
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    float RollRate;
    float PitchRate;
    float YawRate;
    float Roll;
    float Pitch;
    float Yaw;
    float lastRoll;
    float lastPitch;
    float lastYaw;
    // 添加经过一阶低通滤波后的 RollPitchYaw 数据
    float FilteredRoll;
    float FilteredPitch;
    float FilteredYaw;
} MPU6050_Data;

int MPU6050_DMP_Init(void);
int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw, short *gx, short *gy, short *gz, short *ax, short *ay, short *az);
int MPU6050_Get_Data(void);

#endif
