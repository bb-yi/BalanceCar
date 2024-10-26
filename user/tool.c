#include "tool.h"

float loop_now_time;  // 开机时间 毫秒
float loop_last_time; // 上一次循环时间 毫秒
float loop_dt;        // 循环时间间隔 毫秒

#define CPU_FREQUENCY_MHZ 168 // STM32时钟主频
/**
 * @brief微秒级延时函数
 *
 * @param delay
 */
void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            } while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            } while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

/**
 * @brief 浮点类型映射范围函数
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param output_min
 * @param output_max
 * @return float
 */
float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

/**
 * @brief 整数类型映射范围函数
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param output_min
 * @param output_max
 * @return int
 */
int int_Map(int input_value, int input_min, int input_max, int output_min, int output_max)
{
    int output_value;

    // 防止除以零的情况
    if (input_max == input_min)
    {
        return output_min; // 或者根据需求返回合理的值
    }

    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        // 先乘法再除法，避免整数除法导致的精度损失
        output_value = output_min + ((input_value - input_min) * (output_max - output_min)) / (input_max - input_min);
    }

    return output_value;
}

/**
 * @brief 浮点类型映射范围函数，带有中值分割
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param median
 * @param output_min
 * @param output_max
 * @return float
 */
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_value;
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_value < median)
    {
        output_value = float_Map(input_value, input_min, median, output_min, output_median);
        return output_value;
    }
    else
    {
        output_value = float_Map(input_value, median, input_max, output_median, output_max);
        return output_value;
    }
}

/**
 * @brief 钳制范围函数
 *
 * @param value
 * @param min
 * @param max
 * @return float
 */
float clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}
float clamp_min(float value, float min_positive)
{
    if (value > 0 && value < min_positive)
    {
        return min_positive; // 正数小于最小值，返回最小值
    }
    else if (value < 0 && value > -min_positive)
    {
        return -min_positive; // 负数大于最大值（更接近0），返回最大负值
    }
    return value; // 否则返回原始值
}

/**
 * @brief 平滑运动s函数
 *
 * @param StartSpeed  起始速度
 * @param TargetSpeed   目标速度
 * @param Smoothness 平滑系数 越大越平滑
 * @param current_time 当前时间
 * @param total_time 总时间
 * @param Threshold 速度差值阈值 误差小于百分之几时，直接返回目标速度
 * @return float
 */
float S_Curve_Smoothing(float StartSpeed, float TargetSpeed, float Smoothness, float current_time, float total_time, float Threshold)
{
    // 计算归一化的时间范围 (从0到1)
    float normalized_time = (current_time - total_time / 2) / (total_time / 2);

    // 计算速度变化的Sigmoid曲线
    float speed = StartSpeed + (TargetSpeed - StartSpeed) / (1 + expf(-normalized_time / Smoothness));

    // 判断当前速度和目标速度之间的差值是否小于阈值
    if (fabs(speed - TargetSpeed) < Threshold * TargetSpeed)
    {
        return TargetSpeed; // 差值小于阈值，直接返回目标速度
    }

    return speed;
}

/**
 * @brief 求绝对值函数
 * @param a
 * @return float
 */
float Abs(float a)
{
    if (a >= 0)
        return a;
    else
        return (-a);
}

// 角度转弧度的函数
float degreesToRadians(float degrees)
{
    return degrees * (PI / 180.0f);
}

float radiansToDegrees(float radians)
{
    return radians * (180.0f / PI);
}

#if 1
/**
 * @brief 通用接口，主频80MHz，预分频值为80-1，设置PWM的脉冲频率freq(0.16-10kHz)、占空比参数 pulse (0-100)
 *
 * @param htim pwm时钟句柄
 * @param Channel 通道号
 * @param freq PWM频率
 * @param duty PWM占空比
 */
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, float duty)
{
    uint16_t prescaler = 0;
    uint64_t tim_clk_freq = 72000000;
    // 计算PWM频率，所对应的自动重装载值   ---> ARR = 主频 / (预分频+1) / 预期PWM频率(Hz) - 1
    float pwm_freq_arr = (tim_clk_freq * 1.0f) / (prescaler + 1.0f) / freq * 1.0f - 1.0f;
    // 计算PWM占空比，所对应比较寄存器的值 ---> CCR = 预期占空比 * (自动重装载值+1)
    // 占空比则由捕获/比较寄存器（TIMx_CRx）寄存器决定。占空比:duty = Pluse / (ARR+1)
    float pwm_duty_pulse = duty * 1.0f / 100.0f * (pwm_freq_arr + 1);

    // 配置PSC预分频值
    __HAL_TIM_SET_PRESCALER(&htim, prescaler);
    // 配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim, (uint16_t)pwm_freq_arr);
    // 配置PWM占空比
    __HAL_TIM_SetCompare(&htim, Channel, (uint16_t)pwm_duty_pulse);
}
#endif
