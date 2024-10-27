#include "motor.h"
#include "tim.h"
#include "tool.h"

void Set_Motor_enable(uint8_t enable)
{
    if (enable == 1)
    {
        HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
    }
    else if (enable == 0)
    {
        HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
    }
}

void Set_Motor_Dir(uint8_t motor, uint8_t dir)
{
    if (motor == 0)
    {
        if (dir == 0)
        {
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        }
        else if (dir == 1)
        {
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        }
    }
    else if (motor == 1)
    {
        if (dir == 0)
        {
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        }
        else if (dir == 1)
        {
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        }
    }
}
// 配置PSC预分频值,主频4000000HZ
uint16_t prescaler = 18 - 1;
uint16_t pwm_freq_arr = 4000;
void Set_motor_pwm(void)
{

    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
    // 配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim2, pwm_freq_arr - 1);
    // 配置PWM占空比
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_freq_arr / 2 - 1);
}
void set_pwm_duty(uint8_t motor, float duty)
{
    uint16_t duty_arr = float_Map(duty, 0, 100, 0, pwm_freq_arr - 1);
    if (motor == MOTOR_A)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, duty_arr);
    }
    else if (motor == MOTOR_B)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, duty_arr);
    }
}

void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 启动指定通道的 PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // 启动指定通道的 PWM
    Set_Motor_enable(1);
    Set_motor_pwm();
    set_pwm_duty(MOTOR_A, 0);
    set_pwm_duty(MOTOR_B, 0);
    Set_Motor_Dir(MOTOR_A, CW);
    Set_Motor_Dir(MOTOR_B, CW);
}
MOTOR_Data motor_data = {0};
void Encoder_EXTI_Callback(uint16_t GPIO_Pin)
{
    // HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    if (GPIO_Pin == GPIO_PIN_0)
    {
        // PB0中断处理代码
        if (HAL_GPIO_ReadPin(ENCODE_L_A_GPIO_Port, ENCODE_L_A_Pin) == HAL_GPIO_ReadPin(ENCODE_L_B_GPIO_Port, ENCODE_L_B_Pin))
        {
            motor_data.Encoder_A_Count--;
        }
        else
        {
            motor_data.Encoder_A_Count++;
        }
        // printf("Encoder_A_Count:%d\r\n", Encoder_A_Count);
        // printf("PB0 interrupt\r\n");
    }
    else if (GPIO_Pin == GPIO_PIN_1)
    {
        // PB1中断处理代码
        if (HAL_GPIO_ReadPin(ENCODE_R_A_GPIO_Port, ENCODE_R_A_Pin) == HAL_GPIO_ReadPin(ENCODE_R_B_GPIO_Port, ENCODE_R_B_Pin))
        {
            motor_data.Encoder_B_Count++;
        }
        else
        {
            motor_data.Encoder_B_Count--;
        }
        // printf("Encoder_B_Count:%d\r\n", Encoder_B_Count);
        // printf("PB1 interrupt\r\n");
    }
}

void Set_Motor_Velocity(uint8_t motor, float Velocity)
{
    uint8_t dir = Velocity > 0 ? CW : CCW;
    Velocity = Abs(Velocity);
    Velocity = float_Map(Velocity, 0, 100, 8, 100);
    Velocity = clamp(Velocity, 0, 100);
    Set_Motor_Dir(motor, dir);
    set_pwm_duty(motor, Velocity);
}
