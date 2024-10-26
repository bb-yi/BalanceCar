#ifndef __TOOL_H
#define __TOOL_H

#include "main.h"

#define I2C_W_SCL(x) HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, (GPIO_PinState)(x))
#define I2C_W_SDA(x) HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, (GPIO_PinState)(x))
#define I2C_R_SCL HAL_GPIO_ReadPin(GPIOB, SCL_Pin)
#define I2C_R_SDA HAL_GPIO_ReadPin(GPIOB, SDA_Pin)

#ifndef PI
#define PI 3.14159265359f
#endif

extern float loop_now_time;
extern float loop_last_time;
extern float loop_dt;

void delay_us(__IO uint32_t delay);

float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max);
int int_Map(int input_value, int input_min, int input_max, int output_min, int output_max);
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max);
float S_Curve_Smoothing(float StartSpeed, float TargetSpeed, float Smoothness, float current_time, float total_time, float Threshold);
float Abs(float a);
float clamp(float value, float min, float max);
float clamp_min(float value, float min_positive);
float Cosinelaw(float l, float s1, float s2);
float cos2sin(float cos_val);

float degreesToRadians(float degrees);
float radiansToDegrees(float radians);

#if 1
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, float duty);
#endif
#define DMA_PRINT_MAX 512

#endif