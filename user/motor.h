#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#define MOTOR_A 0 // Right motor
#define MOTOR_B 1 // Left motor

#define CW 0
#define CCW 1
void motor_init(void);
void Encoder_EXTI_Callback(uint16_t GPIO_Pin);
void Set_Motor_Velocity(uint8_t motor, float Velocity);
#endif
