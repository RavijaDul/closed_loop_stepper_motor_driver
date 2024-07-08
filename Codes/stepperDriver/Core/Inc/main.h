/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f1xx_hal_tim.h"
//#include "stm32f1xx_hal_i2c.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_Pin GPIO_PIN_13
#define D1_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOD
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOD
#define Temp_Pin GPIO_PIN_1
#define Temp_GPIO_Port GPIOA
#define IN_BM_Pin GPIO_PIN_2
#define IN_BM_GPIO_Port GPIOA
#define IN_BP_Pin GPIO_PIN_3
#define IN_BP_GPIO_Port GPIOA
#define IN_AM_Pin GPIO_PIN_4
#define IN_AM_GPIO_Port GPIOA
#define IN_AP_Pin GPIO_PIN_5
#define IN_AP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_6
#define DIR_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_2
#define BUTTON_1_GPIO_Port GPIOB
#define IN_APB10_Pin GPIO_PIN_10
#define IN_APB10_GPIO_Port GPIOB
#define IN_AMB11_Pin GPIO_PIN_11
#define IN_AMB11_GPIO_Port GPIOB
#define BUTTON_2_Pin GPIO_PIN_12
#define BUTTON_2_GPIO_Port GPIOB
#define ID_0_Pin GPIO_PIN_8
#define ID_0_GPIO_Port GPIOA
#define ID_1_Pin GPIO_PIN_9
#define ID_1_GPIO_Port GPIOA
#define ID_2_Pin GPIO_PIN_10
#define ID_2_GPIO_Port GPIOA
#define SW_DIO_Pin GPIO_PIN_13
#define SW_DIO_GPIO_Port GPIOA
#define SW_CLK_Pin GPIO_PIN_14
#define SW_CLK_GPIO_Port GPIOA
#define Enable_Pin GPIO_PIN_15
#define Enable_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
