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
#include <math.h>
#include "stm32f1xx_hal.h"

#define MAX_INTEGRAL 1000.0

I2C_HandleTypeDef hi2c1;


volatile float encoder_value = 0.0;  // Encoder value variable
float setpoint = 1000.0;  // Desired encoder value (adjust as needed)
float kp = 0.1;  // Proportional gain
float ki = 0.01;  // Integral gain
float integral = 0.0;  // Integral term
float error = 0.0;  // Error term
float control_signal = 0.0;  // Control signal
int direction;
int desired_position;
int enable;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void PI_Controller(void);
void readEncoderData(void);
void readInputs(void);
void StepMotor(uint8_t);

const uint8_t step_sequence[4][4] = {
    {1, 0, 1, 0}, // Step 1: A+ B+
    {0, 1, 1, 0}, // Step 2: A- B+
    {0, 1, 0, 1}, // Step 3: A- B-
    {1, 0, 0, 1}  // Step 4: A+ B-
};

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  uint8_t step = 0;
  while (1)
  {
	if (enable)
	{
		readInputs();
		readEncoderData();  // Read encoder data
		PI_Controller();  // Compute PI control signal


		if (error>0)
		{
		  StepMotor(step);
		  HAL_Delay((int)(1000 / fabs(control_signal))); // Delay inversely proportional to control signal
		  step = (step + 1) % 4; // Increment step for forward direction
		}
		else if (error<0)
		{
		  StepMotor(step);
		  HAL_Delay((int)(1000 / fabs(control_signal))); // Delay inversely proportional to control signal
		  step = (step + 3) % 4; // Decrement step for reverse direction
		}
	}

  }
}


void SetHBridge1Pins(uint8_t in1, uint8_t in2) {
    HAL_GPIO_WritePin(IN_AM_GPIO_Port, IN_AP_Pin, in1);
    HAL_GPIO_WritePin(IN_AM_GPIO_Port, IN_AM_Pin, in2);
}

// Function to set GPIO pins for H-Bridge 2
void SetHBridge2Pins(uint8_t in1, uint8_t in2) {
    HAL_GPIO_WritePin(IN_BP_GPIO_Port, IN_BP_Pin, in1);
    HAL_GPIO_WritePin(IN_BP_GPIO_Port, IN_BM_Pin, in2);
}

// Function to step the motor
void StepMotor(uint8_t step) {
    // Set H-Bridge 1 pins
    SetHBridge1Pins(step_sequence[step][0], step_sequence[step][1]);
    // Set H-Bridge 2 pins
    SetHBridge2Pins(step_sequence[step][2], step_sequence[step][3]);
}

void PI_Controller(void)
{
	error = desired_position - encoder_value;  // Calculate error

	integral += error;  // Update integral term
	if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;  // Limit integral term
	if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;  // Limit integral term

	control_signal = kp * error + ki * integral;  // Calculate control signal
}


void readEncoderData(void)
{
  // Placeholder - Implement based on hardware
  uint8_t data[2];  // Data buffer
  if (HAL_I2C_Master_Receive(&hi2c1, 0x36 << 1, data, 2, HAL_MAX_DELAY) == HAL_OK)
  {
    encoder_value = ((data[0] << 8) | data[1]) / 256.0 * 360.0;  // Convert to degrees
  }
  else
  {
    Error_Handler();  // Error handler
  }
}

void readInputs()
{
	direction = HAL_GPIO_ReadPin(ID_0_GPIO_Port,ID_0_Pin);
	desired_position = HAL_GPIO_ReadPin(ID_1_GPIO_Port,ID_1_Pin);
	enable = HAL_GPIO_ReadPin(ID_2_GPIO_Port,ID_2_Pin);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_I2C1_Init(void)
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D1_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN_BM_Pin|IN_BP_Pin|IN_AM_Pin|IN_AP_Pin
                          |DIR_Pin|PGO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN_APB10_Pin|IN_AMB11_Pin|CAN_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Temp_Pin */
  GPIO_InitStruct.Pin = Temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(Temp_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_BM_Pin IN_BP_Pin IN_AM_Pin IN_AP_Pin
                           DIR_Pin PGO_Pin */
  GPIO_InitStruct.Pin = IN_BM_Pin|IN_BP_Pin|IN_AM_Pin|IN_AP_Pin
                          |DIR_Pin|PGO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin CAN_RX_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|CAN_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_APB10_Pin IN_AMB11_Pin CAN_TX_Pin */
  GPIO_InitStruct.Pin = IN_APB10_Pin|IN_AMB11_Pin|CAN_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ID_0_Pin ID_1_Pin ID_2_Pin SW_DIO_Pin
                           SW_CLK_Pin */
  GPIO_InitStruct.Pin = ID_0_Pin|ID_1_Pin|ID_2_Pin|SW_DIO_Pin
                          |SW_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_ENC_Pin */
  GPIO_InitStruct.Pin = OUT_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(OUT_ENC_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
