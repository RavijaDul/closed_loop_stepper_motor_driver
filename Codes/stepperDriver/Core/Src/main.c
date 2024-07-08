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
#define MAX_ANGLE 360.0

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

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

int full_rotations = 0;
float remaining_degrees = 0.0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void PI_Controller(void);
void readEncoderData(void);
void readInputs(void);
void StepMotor(uint8_t);
void readDesiredPositionSPI(void);

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
  MX_SPI1_Init();
  uint8_t step = 0;
  while (1)
  {
	if (enable)
	{
		readInputs();
		readDesiredPositionSPI();  // Read desired position via SPI
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
	  //Normalize the encoder value within 0 to 360 degrees
	float normalized_encoder_value = fmod(encoder_value, MAX_ANGLE);
	if (normalized_encoder_value < 0) normalized_encoder_value += MAX_ANGLE;

	  // Calculate the full rotations for the encoder
	int encoder_rotations = encoder_value / MAX_ANGLE;

	  // Normalize the desired position within 0 to 360 degrees
	float normalized_position = fmod(desired_position, MAX_ANGLE);
	if (normalized_position < 0) normalized_position += MAX_ANGLE;

	  // Calculate the full rotations for the desired position
	int desired_rotations = desired_position / MAX_ANGLE;

	  // Calculate the total error in terms of rotations and remaining degrees
	int rotation_error = desired_rotations - encoder_rotations;
	float degree_error = normalized_position - normalized_encoder_value;


	  // Combine the rotation and degree errors
	error = rotation_error * MAX_ANGLE + degree_error;

	error = desired_position - encoder_value;  // Calculate error

	if (direction == GPIO_PIN_RESET)  // Check if direction pin is reset (logic low)
	  {
		error = -error;  // Invert error for reverse direction
	  }

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
    direction = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6); // PA6 for DIR
    enable = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // PA15 for Enable
}


void readDesiredPositionSPI(void)
{
    uint8_t spi_rx_buffer[2];
    if (HAL_SPI_Receive(&hspi1, spi_rx_buffer, 2, HAL_MAX_DELAY) == HAL_OK)
    {
        desired_position = (spi_rx_buffer[0] << 8) | spi_rx_buffer[1];
    }
    else
    {
        Error_Handler();
    }
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


}



static void MX_SPI1_Init(void)
{


  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, IN_BM_Pin|IN_BP_Pin|IN_AM_Pin|IN_AP_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : IN_BM_Pin IN_BP_Pin IN_AM_Pin IN_AP_Pin */
  GPIO_InitStruct.Pin = IN_BM_Pin|IN_BP_Pin|IN_AM_Pin|IN_AP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin ID_0_Pin ID_1_Pin ID_2_Pin
                           SW_DIO_Pin SW_CLK_Pin Enable_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|ID_0_Pin|ID_1_Pin|ID_2_Pin
                          |SW_DIO_Pin|SW_CLK_Pin|Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin PB5 CAN_RX_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|GPIO_PIN_5|CAN_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_APB10_Pin IN_AMB11_Pin CAN_TX_Pin */
  GPIO_InitStruct.Pin = IN_APB10_Pin|IN_AMB11_Pin|CAN_TX_Pin;
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
