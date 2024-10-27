/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "mpu6050.h"
#include "Control.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */

extern MOTOR_Data motor_data;
extern MPU6050_Data mpu6050_data;

/**
 * @brief �???????????查剩余内内存
 *
 * @param task
 */
void check_stack_usage(TaskHandle_t task)
{
  UBaseType_t stack_water_mark = uxTaskGetStackHighWaterMark(task);

  printf("Stack high water mark: %u\n", stack_water_mark);
}
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  Set_Motor_Velocity(MOTOR_A, 0);
  Set_Motor_Velocity(MOTOR_B, 0);

  /* Infinite loop */
  for (;;)
  {
    // printf("Hello from defaultTask!\n");
    // printf("Encoder_A_Count: %d, Encoder_B_Count: %d\n", Encoder_A_Count, Encoder_B_Count);

    // printf("roll=%.2f, pitch=%.2f, yaw=%.2f,roll_rate=%.2f,pitch_rate=%.2f,yaw_rate=%.2f,A_count=%d,B_count=%d\n", mpu6050_data.Roll, mpu6050_data.Pitch, mpu6050_data.Yaw, mpu6050_data.RollRate, mpu6050_data.PitchRate, mpu6050_data.YawRate, Encoder_A_Count, Encoder_B_Count);
    mian_control();

    // osDelay(20);

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  // check_stack_usage(myTask02Handle);

  OLED_Init();

  /* Infinite loop */
  for (;;)
  {
    MPU6050_Read_Result(); // 读roll, pitch, yaw
    printf("roll=%.2f, pitch=%.2f, yaw=%.2f,Gx=%.2f,Gy=%.2f,Gz=%.2f\n", mpu6050_data.Roll, mpu6050_data.Pitch, mpu6050_data.Yaw, mpu6050_data.Gx, mpu6050_data.Gy, mpu6050_data.Gz);
    // osDelay(50);

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for (;;)
  {
    motor_data.Velocity_A = motor_data.Encoder_A_Count - motor_data.last_Encoder_A_Count;
    motor_data.Velocity_B = motor_data.Encoder_B_Count - motor_data.last_Encoder_B_Count;
    motor_data.last_Encoder_A_Count = motor_data.Encoder_A_Count;
    motor_data.last_Encoder_B_Count = motor_data.Encoder_B_Count;
    // printf("Velocity_A: %.2f, Velocity_B: %.2f\n", motor_data.Velocity_A, motor_data.Velocity_B);
    vTaskDelay(pdMS_TO_TICKS(20));
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
