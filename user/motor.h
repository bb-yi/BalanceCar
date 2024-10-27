#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#define MOTOR_A 0 // Right motor
#define MOTOR_B 1 // Left motor
typedef struct
{
    int Encoder_A_Count;
    int Encoder_B_Count;
    int last_Encoder_A_Count;
    int last_Encoder_B_Count;
    float Velocity_A;
    float Velocity_B;
} MOTOR_Data;
// 330脉冲每圈 减速比为30:1
#define CW 0
#define CCW 1
void motor_init(void);
void Encoder_EXTI_Callback(uint16_t GPIO_Pin);
void Set_Motor_Velocity(uint8_t motor, float Velocity);
#endif
