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
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for imuCalibTask */
osThreadId_t imuCalibTaskHandle;
const osThreadAttr_t imuCalibTask_attributes = {
  .name = "imuCalibTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osMutexId_t i2c_mutex;

rcl_publisher_t sensor_data_pub;  // Gộp IMU + motor vào 1 topic
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray sensor_data_msg;  // [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
geometry_msgs__msg__TwistStamped cmd_vel_msg;

//Parameters Robot
const float pi = 3.14159265359;
const float wheel_radius = 0.05;//0.143m
const float wheel_base = 0.455;
static float sensor_data_buf[8] = {0};  // [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]

volatile uint8_t calibration_done = 0;
extern float speed_left, speed_right;

// Debug counters
volatile uint32_t imu_read_counter = 0;
volatile int16_t last_gyro_z_raw = 0;

//Parameters PID & PWM
int PPR = 998;//=250x4/pulse per revolution
const float Ts = 0.01f;
float Kp = 1200;//2000
float Ki = 1400;//5500
float Kd = 2;//1.55


//Parameters PID for Wheel's Motors
float Kp_L = 880, Ki_L = 4000, Kd_L = 3.5;   // <-- Tinh chỉnh lại motor TRÁI
float Kp_R = 1000, Ki_R = 3500, Kd_R = 3.7;   // <-- Tinh chỉnh lại motor PHẢI

float setpoint = 0;

// Debug flags
volatile uint8_t pid_running = 0;
volatile uint8_t pwm_running = 0;

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
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */
void PID_Calculate1(void);
void PID_Calculate2(void);
void PWM_Calculate1(void);
void PWM_Calculate2(void);
void updateRPY(void);
void updateRPY(void);

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__TwistStamped *msg = (const geometry_msgs__msg__TwistStamped *)msgin;

    // Giải mã vận tốc tuyến tính & góc từ twist.linear và twist.angular
    float v = msg->twist.linear.x;   // m/s
    float w = msg->twist.angular.z;  // rad/s

    // Tính vận tốc từng bánh - đảo cả v và w để sửa chiều tiến và góc quay
    // ⚠️ Atomic update: disable interrupts để cập nhật đồng thời
    __disable_irq();
    setpoint_left  = ((-v) - (wheel_base / 2.0f) * (w));
    setpoint_right = ((-v) + (wheel_base / 2.0f) * (w));
    __enable_irq();
}

void ImuCalibTask(void *argument)
{
    // Calibrate IMU - không dùng LED để tránh conflict với MotorControlTask
    MPU6050_Calibrate(&MPU6050, &base_x_gyro, &base_y_gyro, &base_z_gyro);
    calibration_done = 1;
    osThreadExit();  // kết thúc task sau khi calibrate xong
}

void ImuTask(void *argument)
{
    // Đợi calibration hoàn thành trước khi xử lý
    while (!calibration_done) {
        osDelay(10);
    }

    // Khởi tạo mốc thời gian ban đầu
	t_last = osKernelGetTickCount();

    for (;;)
    {
        // Đọc IMU (đã convert sang m/s2 và deg/s trong MPU6050.c)
        osMutexAcquire(i2c_mutex, osWaitForever);
        MPU6050_ProcessData(&MPU6050);

        // Debug: increment counter và lưu raw gyro Z
        imu_read_counter++;
        last_gyro_z_raw = MPU6050.gyro_z_raw;

        // Lưu gia tốc (m/s^2) vào biến global
        g_accel_angle_x = MPU6050.acc_x;
        g_accel_angle_y = MPU6050.acc_y;
        g_accel_angle_z = MPU6050.acc_z;

        // Lưu vận tốc góc (deg/s) vào biến global
        g_gyro_angle_x  = MPU6050.gyro_x;
        g_gyro_angle_y  = MPU6050.gyro_y;
        g_gyro_angle_z  = MPU6050.gyro_z;
        osMutexRelease(i2c_mutex);

        // Không cần tính quaternion - IPC sẽ xử lý
        // updateRPY(); // Bỏ để giảm CPU overhead

        osDelay(10); // ~100 Hz
    }
}

void MotorControlTask(void *argument)
{
    // Reset debug flags
    pid_running = 0;
    pwm_running = 0;

  for(;;)
  {
      // BƯỚC 1: Tính cả 2 PID trước (đồng bộ tính toán)
      PID_Calculate2();  // Right
      PID_Calculate1();  // Left        // BƯỚC 2: Disable interrupts để xuất PWM đồng thời
        __disable_irq();
        PWM_Calculate2();  // Right
        PWM_Calculate1();  // Left
        __enable_irq();

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

  /* IMU - Try init but don't block if failed */
  uint8_t imu_ok = MPU6050_init();
  
  if (!imu_ok) {
      // ⚠️ MPU6050 không kết nối - chỉ báo lỗi, KHÔNG block hệ thống
      HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
      // Hệ thống vẫn chạy để test motor
  } else {
      set_last_read_angle_data(0, 0, 0, 0, 0, 0, 0);
  }

  /* Reset tất cả biến encoder và PID về 0 khi reset STM32 */
  // Reset encoder counters
  __HAL_TIM_SET_COUNTER(&htim1, 0);  // Left encoder
  __HAL_TIM_SET_COUNTER(&htim4, 0);  // Right encoder
  
  // Reset encoder variables
  enc_right = 0;
  pre_enc_right = 0;
  delta_right = 0;
  enc_left = 0;
  pre_enc_left = 0;
  delta_left = 0;
  
  // Reset speed variables
  speed_left = 0.0f;
  speed_right = 0.0f;
  setpoint_left = 0.0f;
  setpoint_right = 0.0f;
  
  // Reset PID variables - Right motor
  Error1 = 0;
  pre_Error1 = 0;
  pre_pre_Error1 = 0;
  P_part1 = 0;
  I_part1 = 0;
  D_part1 = 0;
  duty_cycle_right = 0;
  duty_right = 0;
  pre_duty_right = 0;
  
  // Reset PID variables - Left motor
  Error2 = 0;
  pre_Error2 = 0;
  pre_pre_Error2 = 0;
  P_part2 = 0;
  I_part2 = 0;
  D_part2 = 0;
  duty_cycle_left = 0;
  duty_left = 0;
  pre_duty_left = 0;
  
  // Reset IMU angles
  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;
  
  // Force calibration_done = 1 để MotorControlTask có thể chạy ngay
  calibration_done = 1;
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

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartTask02, NULL, &imuTask_attributes);

  /* creation of imuCalibTask */
  imuCalibTaskHandle = osThreadNew(StartTask03, NULL, &imuCalibTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartTask04, NULL, &motorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  huart6.Init.BaudRate = 921600;
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
	pid_running = 1;
	enc_left = __HAL_TIM_GET_COUNTER(&htim1);
	delta_left = enc_left - pre_enc_left;
	if(delta_left > 40000){
		delta_left -= 65536;
	}else if(delta_left < -40000){
		delta_left += 65536;
	}
	
	// Lọc nhiễu: Bỏ qua nếu delta quá lớn (encoder glitch)
	if(fabsf(delta_left) > 5000) {
		delta_left = 0;  // Giữ nguyên giá trị trước
	}
	
	speed_left= ((delta_left/PPR)/Ts)*(2*pi*wheel_radius);
	
	// Deadzone: loại bỏ nhiễu nhỏ khi về 0
	if(fabsf(speed_left) < 0.08f) {
		speed_left = 0.0f;
	}
	
	pre_enc_left = enc_left;
	
	// Đọc setpoint atomically
	__disable_irq();
	float sp_left = setpoint_left;
	__enable_irq();
	
	Error1 = sp_left - speed_left;

	if (sp_left == 0.0f) {
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
	}else if(delta_right < -40000){
		delta_right += 65536;
	}
	
	// Lọc nhiễu: Bỏ qua nếu delta quá lớn (encoder glitch)
	if(fabsf(delta_right) > 5000) {
		delta_right = 0;  // Giữ nguyên giá trị trước
	}
	
	speed_right = ((delta_right/PPR)/Ts)*(2*pi*wheel_radius);
	
	// Deadzone: loại bỏ nhiễu nhỏ khi về 0
	if(fabsf(speed_right) < 0.08f) {
		speed_right = 0.0f;
	}
	
	pre_enc_right = enc_right;
	
	// Đọc setpoint atomically
	__disable_irq();
	float sp_right = setpoint_right;
	__enable_irq();
	
	Error2 = sp_right - speed_right;

	if (sp_right == 0.0f) {
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
	pwm_running = 1;
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

    // ==== Gyro Z Deadzone để chống drift ====
    #define GYRO_Z_DEADZONE 0.5f  // 0.5 deg/s
    if (fabsf(g_gyro_angle_z - base_z_gyro) < GYRO_Z_DEADZONE) {
        gz = 0.0f;
    }

    // ==== Stationary detection - ngừng tích phân yaw khi robot đứng yên ====
    static int stationary_count = 0;
    #define STATIONARY_THRESHOLD 0.01f  // m/s
    #define STATIONARY_SAMPLES 50       // 0.5 giây @ 100Hz
    
    if (fabsf(speed_left) < STATIONARY_THRESHOLD && fabsf(speed_right) < STATIONARY_THRESHOLD) {
        stationary_count++;
        if (stationary_count > STATIONARY_SAMPLES) {
            gz = 0.0f;  // Dừng tích phân yaw khi robot đứng yên
        }
    } else {
        stationary_count = 0;
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

	  // ---------- Publisher - gộp tất cả sensor data vào 1 topic ----------
	      rclc_publisher_init_default(
	          &sensor_data_pub,
	          &node,
	          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	          "sensor_data");

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

		  // ---------- Init message structure ----------
		  std_msgs__msg__Float32MultiArray__init(&sensor_data_msg);

		  // Gán data pointer một lần - buffer chứa [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
		  sensor_data_msg.data.data = sensor_data_buf;
		  sensor_data_msg.data.size = 8;
		  sensor_data_msg.data.capacity = 8;

		  uint32_t loop_count = 0;
		  const float deg2rad = 0.0174532925f; // M_PI / 180.0f

		  while (1)
		  {
			  // Đọc dữ liệu IMU với mutex ngắn nhất có thể
			  osMutexAcquire(i2c_mutex, osWaitForever);
			  float gyro_x = g_gyro_angle_x;
			  float gyro_y = g_gyro_angle_y;
			  float gyro_z = g_gyro_angle_z;
			  float accel_x = g_accel_angle_x;
			  float accel_y = g_accel_angle_y;
			  float accel_z = g_accel_angle_z;
			  osMutexRelease(i2c_mutex);

			  // Đọc motor speed atomically
			  __disable_irq();
			  float s_right = -speed_right;
			  float s_left = -speed_left;
			  __enable_irq();

			  // Đóng gói tất cả data vào 1 array - [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, speed_left, speed_right]
			  sensor_data_buf[0] = gyro_x * deg2rad;  // rad/s
			  sensor_data_buf[1] = gyro_y * deg2rad;
			  sensor_data_buf[2] = gyro_z * deg2rad;
			  sensor_data_buf[3] = accel_x;  // m/s²
			  sensor_data_buf[4] = accel_y;
			  sensor_data_buf[5] = accel_z;
			  sensor_data_buf[6] = s_left;   // m/s
			  sensor_data_buf[7] = s_right;

			  // Publish 1 topic duy nhất
			  (void)rcl_publish(&sensor_data_pub, &sensor_data_msg, NULL);

			  // Check cmd_vel mỗi 2 lần publish (~40-50Hz cmd_vel response)
			  if (++loop_count >= 2) {
				  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
				  loop_count = 0;
			  }

			  osDelay(5); // ~200Hz target -> ~80-100Hz actual
		  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  // Đợi calibration hoàn thành trước khi xử lý
  while (!calibration_done) {
      osDelay(10);
  }

  // Khởi tạo mốc thời gian ban đầu
  t_last = osKernelGetTickCount();

  for (;;)
  {
      // Đọc IMU (đã convert sang m/s2 và deg/s trong MPU6050.c)
      osMutexAcquire(i2c_mutex, osWaitForever);
      MPU6050_ProcessData(&MPU6050);

      // Debug: increment counter và lưu raw gyro Z
      imu_read_counter++;
      last_gyro_z_raw = MPU6050.gyro_z_raw;

      // Lưu gia tốc (m/s^2) vào biến global
      g_accel_angle_x = MPU6050.acc_x;
      g_accel_angle_y = MPU6050.acc_y;
      g_accel_angle_z = MPU6050.acc_z;

      // Lưu vận tốc góc (deg/s) vào biến global
      g_gyro_angle_x  = MPU6050.gyro_x;
      g_gyro_angle_y  = MPU6050.gyro_y;
      g_gyro_angle_z  = MPU6050.gyro_z;
      osMutexRelease(i2c_mutex);

      // Không cần tính toán phức tạp - để IPC xử lý
      // updateRPY(); // Bỏ để giảm overhead

      osDelay(10); // ~100 Hz
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the imuCalibTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  // Calibrate IMU - không dùng LED để tránh conflict với MotorControlTask
  MPU6050_Calibrate(&MPU6050, &base_x_gyro, &base_y_gyro, &base_z_gyro);
  calibration_done = 1;
  osThreadExit();  // kết thúc task sau khi calibrate xong
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // Sáng LD6 tạm thời để báo task đã khởi động
  HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
  osDelay(500);
  HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
  
  // Reset debug flags
  pid_running = 0;
  pwm_running = 0;

  for(;;)
  {
      // BƯỚC 1: Tính cả 2 PID trước (đồng bộ tính toán)
      PID_Calculate2();  // Right
      PID_Calculate1();  // Left

      // BƯỚC 2: Disable interrupts để xuất PWM đồng thời
      __disable_irq();
      PWM_Calculate2();  // Right
      PWM_Calculate1();  // Left
      __enable_irq();

      osDelay(10);	// 100Hz = 10ms
  }
  /* USER CODE END StartTask04 */
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
