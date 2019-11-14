/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void BaseTimer(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Disp_backlight_Pin GPIO_PIN_0
#define Disp_backlight_GPIO_Port GPIOF
#define Backlight_Pin GPIO_PIN_1
#define Backlight_GPIO_Port GPIOF
#define A_U12V_Pin GPIO_PIN_1
#define A_U12V_GPIO_Port GPIOC
#define A_LUX_Pin GPIO_PIN_2
#define A_LUX_GPIO_Port GPIOC
#define D5_Y_Pin GPIO_PIN_1
#define D5_Y_GPIO_Port GPIOA
#define D3_R_Pin GPIO_PIN_2
#define D3_R_GPIO_Port GPIOA
#define D2_R_Pin GPIO_PIN_3
#define D2_R_GPIO_Port GPIOA
#define DISP_CS_Pin GPIO_PIN_4
#define DISP_CS_GPIO_Port GPIOA
#define SM2__Pin GPIO_PIN_7
#define SM2__GPIO_Port GPIOA
#define DISP_RST_Pin GPIO_PIN_4
#define DISP_RST_GPIO_Port GPIOC
#define DISP_CD_Pin GPIO_PIN_5
#define DISP_CD_GPIO_Port GPIOC
#define SM1B__Pin GPIO_PIN_0
#define SM1B__GPIO_Port GPIOB
#define SM2A__Pin GPIO_PIN_1
#define SM2A__GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_14
#define SW1_GPIO_Port GPIOF
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOF
#define SM1A__Pin GPIO_PIN_8
#define SM1A__GPIO_Port GPIOE
#define SM1A_Pin GPIO_PIN_9
#define SM1A_GPIO_Port GPIOE
#define SM1B_Pin GPIO_PIN_11
#define SM1B_GPIO_Port GPIOE
#define SM2A_Pin GPIO_PIN_13
#define SM2A_GPIO_Port GPIOE
#define D4_Y_Pin GPIO_PIN_12
#define D4_Y_GPIO_Port GPIOB
#define SM3A__Pin GPIO_PIN_14
#define SM3A__GPIO_Port GPIOB
#define SM3B__Pin GPIO_PIN_15
#define SM3B__GPIO_Port GPIOB
#define D1_R_Pin GPIO_PIN_2
#define D1_R_GPIO_Port GPIOG
#define D6_G_Pin GPIO_PIN_7
#define D6_G_GPIO_Port GPIOG
#define SM2B_Pin GPIO_PIN_6
#define SM2B_GPIO_Port GPIOC
#define SM3A_Pin GPIO_PIN_7
#define SM3A_GPIO_Port GPIOC
#define SM3B_Pin GPIO_PIN_8
#define SM3B_GPIO_Port GPIOC
#define OUT6_Pin GPIO_PIN_0
#define OUT6_GPIO_Port GPIOD
#define OUT5_Pin GPIO_PIN_1
#define OUT5_GPIO_Port GPIOD
#define OUT4_Pin GPIO_PIN_2
#define OUT4_GPIO_Port GPIOD
#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOD
#define OUT2_Pin GPIO_PIN_4
#define OUT2_GPIO_Port GPIOD
#define OUT1_Pin GPIO_PIN_5
#define OUT1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
