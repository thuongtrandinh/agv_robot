/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* microros libraries */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist_stamped.h>

#include <rosidl_runtime_c/string_functions.h>

/* C libraries */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Variables Struct */
#include "MPU6050.h"
#include "imu_data_struct.h"
#include "motor_data_struct.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Global publisher/subscriber */

 float temperature = 0;
 // rotation angle of the sensor
 float last_read_time;
 float last_x_angle;  // These are the filtered angles
 float last_y_angle;
 float last_z_angle;
 float last_gyro_x_angle;  // Store the gyro angles to compare drift
 float last_gyro_y_angle;
 float last_gyro_z_angle;

 void set_last_read_angle_data(unsigned long time, float x, float y, float z,
 		float x_gyro, float y_gyro, float z_gyro) {
 	last_read_time = time;
 	last_x_angle = x;
 	last_y_angle = y;
 	last_z_angle = z;
 	last_gyro_x_angle = x_gyro;
 	last_gyro_y_angle = y_gyro;
 	last_gyro_z_angle = z_gyro;
 }

 float get_last_time() {return last_read_time;}
 float get_last_x_angle() {return last_x_angle;}
 float get_last_y_angle() {return last_y_angle;}
 float get_last_z_angle() {return last_z_angle;}
 float get_last_gyro_x_angle() {return last_gyro_x_angle;}
 float get_last_gyro_y_angle() {return last_gyro_y_angle;}
 float get_last_gyro_z_angle() {return last_gyro_z_angle;}

 float base_x_gyro = 0;
 float base_y_gyro = 0;
 float base_z_gyro = 0;
 float base_x_accel = 0;
 float base_y_accel = 0;
 float base_z_accel = 0;

 float x =0, y=0, z=0;
 float gyro_xx = 0,gyro_yy = 0,gyro_zz = 0;

 float accel_angle_x = 0, accel_angle_y = 0, accel_angle_z = 0;
 float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
 // This global variable tells us how to scale gyroscope data
 float GYRO_FACTOR = 0;
 unsigned long t_now = 0, t_last = 0;

 // Các biến toàn cục để truyền giá trị IMU trung gian qua UART
 volatile float g_gyro_angle_x = 0, g_accel_angle_x = 0;
 volatile float g_gyro_angle_y = 0, g_accel_angle_y = 0;
 volatile float g_gyro_angle_z = 0, g_accel_angle_z = 0;
 float roll = 0.0, pitch = 0.0, yaw = 0.0; // Các góc Euler
 float dt = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_TIM 	&htim3 // PWM
#define ALPHA 0.98 // Hệ số điều chỉnh của bộ lọc bổ sung
//#define RADIANS_TO_DEGREES 57.2958; //180/3.14159

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osMutexId_t i2c_mutex;

rcl_publisher_t imu_pub;
rcl_publisher_t motor_feedback_pub;
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__TwistStamped cmd_vel_msg;
std_msgs__msg__Float32MultiArray motor_feedback_msg;

//Parameters Robot
const float pi = 3.14159265359;
const float wheel_radius = 0.05;//0.143m
const float wheel_base = 0.455;
static float motor_fb_buf[2] = {0};

//Parameters PID & PWM
int PPR = 998;//=250x4/pulse per revolution
const float Ts = 0.01f;
float Kp = 1200;//2000
float Ki = 1400;//5500
float Kd = 2;//1.55


//Parameters PID for Wheel's Motors
float Kp_L = 1500, Ki_L = 5000, Kd_L = 3.5;   // <-- Tinh chỉnh lại motor TRÁI
float Kp_R = 810, Ki_R = 2500, Kd_R = 3.7;   // <-- Tinh chỉnh lại motor PHẢI

float setpoint = 0;

//Right
float enc_right=0, pre_enc_right = 0;
float delta_right = 0;
float setpoint_right = 0, speed_right = 0;
float Error1 = 0, pre_Error1 = 0, pre_pre_Error1 = 0;
float P_part1 = 0, I_part1 = 0, D_part1 = 0;
float duty_cycle_right = 0, duty_right = 0, pre_duty_right = 0;
//Left
float enc_left=0, pre_enc_left=0;
float delta_left = 0;
float setpoint_left = 0, speed_left = 0;
float Error2=0, pre_Error2=0, pre_pre_Error2=0;
float P_part2 = 0, I_part2 = 0, D_part2 = 0;
float duty_cycle_left = 0, duty_left = 0, pre_duty_left = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void PID_Calculate1(void);
void PID_Calculate2(void);
void PWM_Calculate1(void);
void PWM_Calculate2(void);
void updateRPY(void);

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__TwistStamped *msg =
        (const geometry_msgs__msg__TwistStamped *)msgin;

    // Lấy vận tốc
    float v = msg->twist.linear.x;     // m/s
    float w = msg->twist.angular.z;    // rad/s

    // ==============================
    //  ĐẢO CHIỀU CHẠY TỚI CỦA ROBOT
    // ==============================
    v = -v;


    // Tính tốc độ bánh
    setpoint_left  = v - (wheel_base / 2.0f) * w;
    setpoint_right = v + (wheel_base / 2.0f) * w;
}

void ImuCalibTask(void *argument)
{
	HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
	osMutexAcquire(i2c_mutex, osWaitForever);
    MPU6050_Calibrate(&MPU6050, &base_x_gyro, &base_y_gyro, &base_z_gyro);
    HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
    osMutexRelease(i2c_mutex);
    osThreadExit();  // kết thúc task sau khi calibrate xong
}

void ImuTask(void *argument)
{
    // Khởi tạo mốc thời gian ban đầu
	t_last = osKernelGetTickCount();

    for (;;)
    {
        // Đọc IMU (đã convert sang m/s2 và deg/s trong MPU6050.c)
        osMutexAcquire(i2c_mutex, osWaitForever);
        MPU6050_ProcessData(&MPU6050);

        // Lưu gia tốc (m/s^2) vào biến global
        g_accel_angle_x = MPU6050.acc_x;
        g_accel_angle_y = MPU6050.acc_y;
        g_accel_angle_z = MPU6050.acc_z;

        // Lưu vận tốc góc (deg/s) vào biến global
        g_gyro_angle_x  = MPU6050.gyro_x;
        g_gyro_angle_y  = MPU6050.gyro_y;
        g_gyro_angle_z  = MPU6050.gyro_z;
        osMutexRelease(i2c_mutex);

        // Cập nhật roll / pitch / yaw + quaternion
        updateRPY();

        osDelay(10); // ~100 Hz
    }
}

void MotorControlTask(void *argument)
{
    for(;;)
    {
        PID_Calculate1();
        PID_Calculate2();

        PWM_Calculate1();
        PWM_Calculate2();

        osDelay(10);	// 100Hz = 10ms
    }
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(PWM_TIM, TIM_CHANNEL_3);

  /* IMU */
  MPU6050_init();

  set_last_read_angle_data(0, 0, 0, 0, 0, 0, 0);

  /* Motor Speed */
  setpoint_left = 0.0;   //(m/s)
  setpoint_right = 0.0;	 //(m/s)
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  const osMutexAttr_t i2c_mutex_attr = {
    .name = "i2cMutex"
  };
  i2c_mutex = osMutexNew(&i2c_mutex_attr);
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadId_t imuTaskHandle;

  const osThreadAttr_t imuCalibTask_attributes = {
    .name = "imuCalibTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t) osPriorityHigh
  };
  osThreadNew(ImuCalibTask, NULL, &imuCalibTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  const osThreadAttr_t imuTask_attributes = {
    .name = "imuTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

  const osThreadAttr_t motorTask_attributes = {
      .name = "motorTask",
      .stack_size = 2048 * 4,
      .priority = (osPriority_t) osPriorityHigh,   // ƯU TIÊN CAO
  };

  osThreadNew(MotorControlTask, NULL, &motorTask_attributes);

  //osThreadNew(ImuTask, NULL, &imuTask_attributes);

  imuTaskHandle = osThreadNew(ImuTask, NULL, &imuTask_attributes);
  if (imuTaskHandle == NULL)
  {
      // ❌ task tạo thất bại
      HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);  // sáng đỏ
  }
  else
  {
      // ✅ task tạo thành công
      HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);  // sáng xanh
  }



  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Function for microros */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* Function for PID */

// v = w.r
//(delta_left/PPR)/Ts: Vận tốc góc trung bình của bánh xe tính bằng vòng/giây
//((delta_left/PPR)/Ts)*2*pi: Vận tốc góc rad/s
//2*pi*wheel_radius: chu vi bánh xe
void PID_Calculate1(){ // left
	enc_left = __HAL_TIM_GET_COUNTER(&htim1);
	delta_left = enc_left - pre_enc_left;
	if(delta_left > 40000){
		delta_left -= 65536;
	}else if(delta_left < -40000){
		delta_left += 65536;
	}
	speed_left= ((delta_left/PPR)/Ts)*(2*pi*wheel_radius);
	pre_enc_left = enc_left;
	Error1 = setpoint_left - speed_left;

	if (Error1 == 0) {
			P_part1 = 0;
			I_part1 = 0;
			D_part1 = 0;
			duty_left = 0;
			pre_pre_Error1 = 0;
			pre_Error1 = 0;
			pre_duty_left = 0;
			return;
		}
		if(Kp_L!=0||Ki_L!=0||Kd_L!=0){
			P_part1 = Kp_L*(Error1-pre_Error1 );
			I_part1 = Ki_L*0.5*Ts*(Error1 + pre_Error1);
			D_part1 = Kd_L/Ts*( Error1 - 2*pre_Error1 + pre_pre_Error1);
			duty_left = pre_duty_left + P_part1 + I_part1 + D_part1;
			//Velocity is limited
			if(duty_left > 699){
				duty_left = 699;
			}else if (duty_left < -699){
				duty_left = -699;
			}
			pre_pre_Error1 = pre_Error1;
			pre_Error1 = Error1;
			pre_duty_left = duty_left;
		}
}

void PID_Calculate2(){ //right
	enc_right = __HAL_TIM_GET_COUNTER(&htim4);
	delta_right = enc_right-pre_enc_right;
	if(delta_right > 40000){
		delta_right -= 65536;
	}else if(delta_right <- 40000){
		delta_right += 65536;
	}
	speed_right = ((delta_right/PPR)/Ts)*(2*pi*wheel_radius);
	pre_enc_right = enc_right;
	Error2 = setpoint_right - speed_right;

	if (Error2 == 0) {
			P_part2 = 0;
			I_part2 = 0;
			D_part2 = 0;
			duty_right = 0;
			pre_pre_Error2 = 0;
			pre_Error2 = 0;
			pre_duty_right = 0;
			return;
		}
		if(Kp_R!=0||Ki_R!=0||Kd_R!=0){
			P_part2 = Kp_R*(Error2-pre_Error2 );
			I_part2 = Ki_R*0.5*Ts*(Error2 + pre_Error2);
			D_part2 = Kd_R/Ts*( Error2 - 2*pre_Error2+ pre_pre_Error2);
			duty_right = pre_duty_right + P_part2 + I_part2 + D_part2;
			//Velocity is limited
			if(duty_right > 699){
				duty_right = 699;
			}else if (duty_right < -699){
				duty_right = -699;
			}
			pre_pre_Error2 = pre_Error2;
			pre_Error2 = Error2;
			pre_duty_right = duty_right;
			}
}

void PWM_Calculate1(){ //left
	if(duty_left == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		}else if(duty_left > 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			duty_cycle_left = duty_left;
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			duty_cycle_left = -duty_left;

		}__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,duty_cycle_left);
}

void PWM_Calculate2(){ //right
	if(duty_right == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	}else if(duty_right > 0){
		duty_cycle_right = duty_right;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

	}else{
		duty_cycle_right = -duty_right;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

	}__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,duty_cycle_right);
}

void updateRPY()
{
    // ==== 1. Cập nhật thời gian ====
	t_now = osKernelGetTickCount();
	dt = (t_now - t_last) * 0.001f;
	t_last = t_now;

    // Chống lỗi khi dt quá nhỏ hoặc quá lớn
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    // ==== 2. Lấy gia tốc (m/s^2) ====
    float ax = g_accel_angle_x;
    float ay = g_accel_angle_y;
    float az = g_accel_angle_z;

    // Góc từ accelerometer (rad)
    float accel_roll  = atan2f(ay, az);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // ==== 3. Lấy gyro (deg/s) -> rad/s, trừ bias ====
    float gx = (g_gyro_angle_x - base_x_gyro) * (M_PI / 180.0f);
    float gy = (g_gyro_angle_y - base_y_gyro) * (M_PI / 180.0f);
    float gz = (g_gyro_angle_z - base_z_gyro) * (M_PI / 180.0f);

    // ==== CHỐNG DRIFT YAW: Thêm deadzone cho gyro Z ====
    const float GYRO_Z_DEADZONE = 0.01f; // rad/s (~0.57 deg/s)
    if (fabsf(gz) < GYRO_Z_DEADZONE)
    {
        gz = 0.0f; // Bỏ qua gyro noise khi robot đứng yên
    }

    // Tích phân gyro để ra góc (rad)
    float gyro_roll  = get_last_x_angle() + gx * dt;
    float gyro_pitch = get_last_y_angle() + gy * dt;
    float gyro_yaw   = get_last_z_angle() + gz * dt;

    // ==== 4. Complementary filter ====
    float angle_x = ALPHA * gyro_roll  + (1.0f - ALPHA) * accel_roll;   // roll
    float angle_y = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;  // pitch
    float angle_z = gyro_yaw;                                           // yaw (chỉ từ gyro)

    // Giữ yaw trong [-pi, pi]
    if (angle_z > M_PI)
        angle_z -= 2.0f * M_PI;
    else if (angle_z < -M_PI)
        angle_z += 2.0f * M_PI;

    // Lưu lại để debug nếu cần
    roll  = angle_x;
    pitch = angle_y;
    yaw   = angle_z;

    // ==== 5. Tính quaternion từ roll (x), pitch (y), yaw (z) ====
    float cy = cosf(angle_z * 0.5f);
    float sy = sinf(angle_z * 0.5f);
    float cp = cosf(angle_y * 0.5f);
    float sp = sinf(angle_y * 0.5f);
    float cr = cosf(angle_x * 0.5f);
    float sr = sinf(angle_x * 0.5f);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // Lưu lại góc cho lần gọi sau
    set_last_read_angle_data(t_now,
                             angle_x, angle_y, angle_z,
                             gyro_roll, gyro_pitch, gyro_yaw);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// micro-ROS configuration

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart6,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "stm32_node", "", &support);

	  // ---------- Publishers ----------
	      rclc_publisher_init_default(
	          &imu_pub,
	          &node,
	          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	          "imu");

	      rclc_publisher_init_default(
	          &motor_feedback_pub,
	          &node,
	          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	          "motor_feedback");

	  // ---------- Subscriber ----------
	      rclc_subscription_init_default(
	          &cmd_vel_sub,
	          &node,
	          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
	          "/diff_cont/cmd_vel");


	  // ---------- Executor ----------
		  rclc_executor_init(&executor, &support.context, 1, &allocator);
		  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);


	  // ---------- Main loop ----------

		  // ---------- Init message structures ----------
		  sensor_msgs__msg__Imu__init(&imu_msg);
		  std_msgs__msg__Float32MultiArray__init(&motor_feedback_msg);

		  // Layout cho motor_feedback
		  motor_feedback_msg.layout.dim.data = NULL;
		  motor_feedback_msg.layout.dim.size = 0;
		  motor_feedback_msg.layout.dim.capacity = 0;
		  motor_feedback_msg.layout.data_offset = 0;

		  // Gán frame_id (chỉ làm 1 lần)
		  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

		  // Tùy chọn: ping agent để đảm bảo micro-ROS kết nối trước khi publish
		  (void)rmw_uros_ping_agent(1000, 5);

		  while (1)
		  {
			  // Publish IMU
			  imu_msg.header.stamp.sec = HAL_GetTick() / 1000;
			  imu_msg.header.stamp.nanosec = (HAL_GetTick() % 1000) * 1000000;
			  osMutexAcquire(i2c_mutex, osWaitForever);

			  imu_msg.angular_velocity.x = g_gyro_angle_x * M_PI / 180.0f;
			  imu_msg.angular_velocity.y = g_gyro_angle_y * M_PI / 180.0f;
			  imu_msg.angular_velocity.z = g_gyro_angle_z * M_PI / 180.0f;

			  imu_msg.linear_acceleration.x = g_accel_angle_x;
			  imu_msg.linear_acceleration.y = g_accel_angle_y;
			  imu_msg.linear_acceleration.z = g_accel_angle_z;

			  (void)rcl_publish(&imu_pub, &imu_msg, NULL);
			  osMutexRelease(i2c_mutex);

			  // Publish motor feedback
			  motor_fb_buf[0] = -speed_left;
			  motor_fb_buf[1] = -speed_right;

			  motor_feedback_msg.data.data = motor_fb_buf;
			  motor_feedback_msg.data.size = 2;
			  motor_feedback_msg.data.capacity = 2;

			  (void)rcl_publish(&motor_feedback_pub, &motor_feedback_msg, NULL);

			  // Spin executor để nhận cmd_vel
			  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

			  osDelay(20); // ~50Hz
		  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
