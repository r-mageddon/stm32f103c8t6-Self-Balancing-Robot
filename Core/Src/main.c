/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI 3.14159265359
#define MAX_PWM 80
#define MIN_PWM -MAX_PWM;

// MPU6050 Address
#define MPU6050_ADDR 		0xD0

// MPU6050 Registers
#define ACCEL_CONFIG 		0x1C
#define GYRO_CONFIG 		0x1B
#define ACCEL_XOUT_H_REG 	0x3B
#define GYRO_XOUT_H_REG 	0x43
#define SMPLRT_DIV_REG		0x19
#define PWR_MNGMT_1_REG		0x6B
#define WHO_AM_I_REG		0x75

// Gyro range
#define GYRO_250			0
#define GYRO_500			8
#define GYRO_1000			9
#define GYRO_2000			10

// Accel range
#define ACCEL_2				0
#define ACCEL_4				8
#define ACCEL_8				9
#define ACCEL_10			10

// MPU6050 Reads
int16_t ax_raw = 0, ay_raw = 0, az_raw = 0; // accelerometer raw values
int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0; // gyroscope raw values
float ax, ay, az, gx, gy, gz; // accurate mpu6050 values (after sensitivity)
float a_roll, a_pitch, g_roll, g_pitch, g_yaw; // No reliability in 'a_yaw'
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

double roll, pitch, yaw; // final values values with filter for noise (roll = input)
static float alpha = 0.995; // prepare angle value before using complimentary filter (lowering this applies more horizontal acceleration)

// PID Controller
double output = 0, LastError = 0;
double error, integral, derivative;
double Kp = 4.3, Ki = 0.06, Kd = 0.135;
const int SetPoint = 1.5;
static int dt = 2000; // Sampling frequency of error = HSE/(pre-scaler * counter_period) = (72*10^6)/(72 * 50)
float current_time = 0, delta_t = 0, last_time = 0;


/* MPU6050 INITIALISE */
void MPU6050_Init(void)
{
	uint8_t check, data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	if (check == 0x68)
	{
		// Wake up Device
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MNGMT_1_REG, 1, &data, 1, 1000);

		// Set data rate
		data = 0x07; // 1kHz
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

		// Configure Gyroscope
		data = GYRO_250; // range = +/- 250 deg
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 1000);

		// Configure Accelerometer
		data = ACCEL_2; // range = +/- 2g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000);
	}
}
/* END OF MPU6050 INITIALIZATION */

/* MPU6050 ACCELEROMETERR READ */
void MPU6050_ACCEL_READ()
{
	uint8_t data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 1000);

	ax_raw = (int16_t)(data[0] << 8 | data[1]);
	ay_raw = (int16_t)(data[2] << 8 | data[3]);
	az_raw = (int16_t)(data[4] << 8 | data[5]);

	ax = (float)ax_raw/16384.0;
	ay = (float)ay_raw/16834.0;
	az = (float)az_raw/16384.0;
}
/* END OF ACCELEROMETER READ */


/* GYROSCOPE READ */
void MPU6050_GYRO_READ()
{
	uint8_t data[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, 1000);

	int16_t gx_raw = (int16_t)(data[0] << 8 | data[1]);
	int16_t gy_raw = (int16_t)(data[2] << 8 | data[3]);
	int16_t gz_raw = (int16_t)(data[4] << 8 | data[5]);

	gx = (float)gx_raw/131.0;
	gy = (float)gy_raw/131.0;
	gz = (float)gz_raw/131.0;
}
/* END OF GYROSCOPE READ */

/* Calibration for MPU6050 */
void CalGyro()
{
	int numReadings = 50;
	int16_t Xsum = 0, Ysum = 0, Zsum = 0;

	for (int i = 0; i < numReadings; i++)
	{
		MPU6050_GYRO_READ();
		Xsum += gx;
		Ysum += gy;
		Zsum += gz;
		HAL_Delay(10);
	}
	gyroXoffset = Xsum/numReadings;
	gyroYoffset = Ysum/numReadings;
	gyroZoffset = Zsum/numReadings;
}
/* END OF GYROSCOPE CALIBRATION */

/* ACCCELAROMETER CLAIBRATION */
void CalAccel()
{
	int numReadings = 50;
	int16_t Xsum = 0, Ysum = 0, Zsum = 0;

	for (int i = 0; i < numReadings; i++)
	{
		MPU6050_ACCEL_READ();
		Xsum += ax;
		Ysum += ay;
		Zsum += az;
		HAL_Delay(10);
	}
	accelXoffset = Xsum/numReadings;
	accelYoffset = Ysum/numReadings;
	accelZoffset = Zsum/numReadings;
}
/* END OF ACCELEROMETER CLIBRATION */

/* DIRECTIONAL MOTOR CONTROL */
void moveForward(int A, int B)
{
	 HAL_GPIO_WritePin(ForwardA_GPIO_Port, ForwardA_Pin, A);
	 HAL_GPIO_WritePin(ForwardB_GPIO_Port, ForwardB_Pin, B);
}

void moveBackward(int A, int B)
{
	HAL_GPIO_WritePin(BackwardA_GPIO_Port, BackwardA_Pin, A);
	HAL_GPIO_WritePin(BackwardB_GPIO_Port, BackwardB_Pin, B);
}
/* END OF MOTOR DIRECTIONAL CONTROL */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim -> Instance == TIM4)
	{

		  // Accelerometer
		  MPU6050_ACCEL_READ();
		  ax -= accelXoffset;
		  ay -= accelYoffset;
		  az -= accelZoffset;
		  //a_roll = atan(-ax/sqrt(pow(ay,2) + pow(az,2))) * 180/PI;
		  a_pitch = atan(-ay/sqrt(pow(ax,2) + pow(az,2))) * 180/PI;
		  // no yaw value due to reliability

		  /* GET TIME FOR GYROSCOPE READINGS AND PID CONTROLLER */
		  last_time = current_time;
		  current_time = HAL_GetTick();
		  delta_t = (current_time - last_time)/1000; // get it in seconds
		  /* END GET TIME */

		  // Gyroscope
		  MPU6050_GYRO_READ();
		  gx -= gyroXoffset;
		  //gy -= gyroYoffset;
		  //gz -= gyroZoffset;
		  //g_roll += gy*delta_t;
		  g_pitch += gx*delta_t;
		  //g_yaw += gz*delta_t;

		  /* ANGLES CALCULATION */

		  // filter
		  //roll = alpha*(gy*delta_t + roll) + (1-alpha)*a_roll;
		  pitch = alpha*(gx*delta_t + pitch) + (1-alpha)*a_pitch;

		  /* END OF ANGLES CALCULATIONS */

		  /* PID IMPLEMENTATIONS */

		  error = SetPoint - pitch;
		  integral += error/dt; // product of error and sampling frequency
		  // Prevent Anti-Winndup (max rpm of motors)
		  if (integral > 80)
			  integral = 80;
		  else if(integral < -80)
			  integral = -80;

		  derivative = (error - LastError)*dt; // division of change in error and sampling frequency

		  output = Kp*error + Ki*integral + Kd*derivative;

		  LastError = error;

		  if (output > 80)
			  output = 80;
		  else if (output < -80)
			  output = -80;

		  /* END OF PID IMPLEMENTATION */

		  /* PWM CONDITION SETUP */

		  // if the error (roll angle) is positive move the motors forward
		  if (error < 0)
		  {
			  // Move motors forward with a speed relative to output value
			  moveForward(1,1);
			  moveBackward(0,0);
			  //Update PWM
			  TIM1 -> CCR1 = abs(output)*0.9;
			  TIM1 -> CCR4 = abs(output)*0.9;
		  }
		  // i	f the error (roll angle) is negative move motors backward
		  else if (error > 0)
		  {
			  // Move motors backward with a speed relative to output value
			  moveForward(0,0);
			  moveBackward(1,1);
			  //Update PWM
			  TIM1 -> CCR1 = abs(output);
			  TIM1 -> CCR4 = abs(output);
		  }
		  // if the error = 0 then do nothing
		  else
		  {
			  moveForward(1,1);
			  moveBackward(1,1);
			  //Update PWM
			  TIM1 -> CCR1 = 0;
			  TIM1 -> CCR4 = 0;
		  }
		  /* END OF CONDIITON SETUP */
	}
}

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM Channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Start MPU6050
  MPU6050_Init();
  HAL_Delay(1000);
  CalGyro(); // Calibrat Gyroscope for accurate reading
  CalAccel(); // Calibrate accelerometer for accurate reading

  // Start TIM4 interrupt
  HAL_TIM_Base_Start_IT(&htim4);

  //HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BackwardB_Pin|ForwardB_Pin|ForwardA_Pin|BackwardA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BackwardB_Pin ForwardB_Pin ForwardA_Pin BackwardA_Pin */
  GPIO_InitStruct.Pin = BackwardB_Pin|ForwardB_Pin|ForwardA_Pin|BackwardA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
