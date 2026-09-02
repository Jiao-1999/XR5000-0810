/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
	

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
#define ZBDQH_Pin GPIO_PIN_4
#define ZBDQH_GPIO_Port GPIOE
#define BATCD_Pin GPIO_PIN_5
#define BATCD_GPIO_Port GPIOE
#define PWLED_G_Pin GPIO_PIN_13
#define PWLED_G_GPIO_Port GPIOC
#define PWLED_R_Pin GPIO_PIN_14
#define PWLED_R_GPIO_Port GPIOC
#define SXLED_G_Pin GPIO_PIN_15
#define SXLED_G_GPIO_Port GPIOC
#define SXLED_R_Pin GPIO_PIN_0
#define SXLED_R_GPIO_Port GPIOF
#define XXLED_G_Pin GPIO_PIN_1
#define XXLED_G_GPIO_Port GPIOF
#define XXLED_R_Pin GPIO_PIN_2
#define XXLED_R_GPIO_Port GPIOF
#define PMLED_G_Pin GPIO_PIN_3
#define PMLED_G_GPIO_Port GPIOF
#define PMLED_R_Pin GPIO_PIN_4
#define PMLED_R_GPIO_Port GPIOF
#define RTCSCL_Pin GPIO_PIN_8
#define RTCSCL_GPIO_Port GPIOF
#define RTCSDA_Pin GPIO_PIN_9
#define RTCSDA_GPIO_Port GPIOF
#define JDQ4P_Pin GPIO_PIN_2
#define JDQ4P_GPIO_Port GPIOB
#define JDQ4N_Pin GPIO_PIN_14
#define JDQ4N_GPIO_Port GPIOF
#define JDQ5P_Pin GPIO_PIN_15
#define JDQ5P_GPIO_Port GPIOF
#define JDQ5N_Pin GPIO_PIN_7
#define JDQ5N_GPIO_Port GPIOE
#define JDQ6P_Pin GPIO_PIN_8
#define JDQ6P_GPIO_Port GPIOE
#define JDQ6N_Pin GPIO_PIN_9
#define JDQ6N_GPIO_Port GPIOE
#define JDQ7P_Pin GPIO_PIN_10
#define JDQ7P_GPIO_Port GPIOE
#define JDQ7N_Pin GPIO_PIN_11
#define JDQ7N_GPIO_Port GPIOE
#define JDQ8P_Pin GPIO_PIN_12
#define JDQ8P_GPIO_Port GPIOE
#define JDQ8N_Pin GPIO_PIN_13
#define JDQ8N_GPIO_Port GPIOE
#define JDQ9P_Pin GPIO_PIN_14
#define JDQ9P_GPIO_Port GPIOE
#define JDQ9N_Pin GPIO_PIN_15
#define JDQ9N_GPIO_Port GPIOE
#define JDQ1P_Pin GPIO_PIN_8
#define JDQ1P_GPIO_Port GPIOD
#define AHTSDA_Pin GPIO_PIN_9
#define AHTSDA_GPIO_Port GPIOD
#define AHTSCL_Pin GPIO_PIN_10
#define AHTSCL_GPIO_Port GPIOD
#define JDQ2N_Pin GPIO_PIN_11
#define JDQ2N_GPIO_Port GPIOD
#define JDQ3P_Pin GPIO_PIN_12
#define JDQ3P_GPIO_Port GPIOD
#define JDQ3N_Pin GPIO_PIN_13
#define JDQ3N_GPIO_Port GPIOD
#define IN1C_Pin GPIO_PIN_14
#define IN1C_GPIO_Port GPIOD
#define IN1B_Pin GPIO_PIN_15
#define IN1B_GPIO_Port GPIOD
#define IN1A_Pin GPIO_PIN_2
#define IN1A_GPIO_Port GPIOG
#define IN2C_Pin GPIO_PIN_3
#define IN2C_GPIO_Port GPIOG
#define IN2B_Pin GPIO_PIN_4
#define IN2B_GPIO_Port GPIOG
#define IN2A_Pin GPIO_PIN_5
#define IN2A_GPIO_Port GPIOG
#define IN3C_Pin GPIO_PIN_6
#define IN3C_GPIO_Port GPIOG
#define IN3B_Pin GPIO_PIN_7
#define IN3B_GPIO_Port GPIOG
#define IN3A_Pin GPIO_PIN_8
#define IN3A_GPIO_Port GPIOG
#define EVCEN1_Pin GPIO_PIN_8
#define EVCEN1_GPIO_Port GPIOC
#define EVCBRK1_Pin GPIO_PIN_9
#define EVCBRK1_GPIO_Port GPIOC
#define EVCEN2_Pin GPIO_PIN_8
#define EVCEN2_GPIO_Port GPIOA
#define W25Q128_CS_Pin GPIO_PIN_10
#define W25Q128_CS_GPIO_Port GPIOA
#define EVCBRK2_Pin GPIO_PIN_15
#define EVCBRK2_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
