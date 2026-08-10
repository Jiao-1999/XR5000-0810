/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ZBDQH_Pin|BATCD_Pin|JDQ5N_Pin|JDQ6N_Pin
                          |JDQ7N_Pin|JDQ8N_Pin|JDQ9N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWLED_G_Pin|PWLED_R_Pin|SXLED_G_Pin|EVCEN1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SXLED_R_Pin|XXLED_G_Pin|XXLED_R_Pin|PMLED_G_Pin
                          |PMLED_R_Pin|RTCSCL_Pin|RTCSDA_Pin|JDQ5P_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, JDQ4P_Pin|SCL_Pin|SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JDQ4N_GPIO_Port, JDQ4N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, JDQ6P_Pin|JDQ7P_Pin|JDQ8P_Pin|JDQ9P_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, JDQ1P_Pin|JDQ3P_Pin|AHTSCL_Pin|AHTSDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, JDQ2N_Pin|JDQ3N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EVCEN2_Pin|W25Q128_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ZBDQH_Pin BATCD_Pin JDQ5N_Pin JDQ6P_Pin
                           JDQ6N_Pin JDQ7P_Pin JDQ7N_Pin JDQ8P_Pin
                           JDQ8N_Pin JDQ9P_Pin JDQ9N_Pin */
  GPIO_InitStruct.Pin = ZBDQH_Pin|BATCD_Pin|JDQ5N_Pin|JDQ6P_Pin
                          |JDQ6N_Pin|JDQ7P_Pin|JDQ7N_Pin|JDQ8P_Pin
                          |JDQ8N_Pin|JDQ9P_Pin|JDQ9N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PWLED_G_Pin PWLED_R_Pin SXLED_G_Pin EVCEN1_Pin */
  GPIO_InitStruct.Pin = PWLED_G_Pin|PWLED_R_Pin|SXLED_G_Pin|EVCEN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SXLED_R_Pin XXLED_G_Pin XXLED_R_Pin PMLED_G_Pin
                           PMLED_R_Pin JDQ4N_Pin JDQ5P_Pin */
  GPIO_InitStruct.Pin = SXLED_R_Pin|XXLED_G_Pin|XXLED_R_Pin|PMLED_G_Pin
                          |PMLED_R_Pin|JDQ4N_Pin|JDQ5P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RTCSCL_Pin RTCSDA_Pin */
  GPIO_InitStruct.Pin = RTCSCL_Pin|RTCSDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : JDQ4P_Pin */
  GPIO_InitStruct.Pin = JDQ4P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JDQ4P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JDQ1P_Pin JDQ2N_Pin JDQ3P_Pin JDQ3N_Pin */
  GPIO_InitStruct.Pin = JDQ1P_Pin|JDQ2N_Pin|JDQ3P_Pin|JDQ3N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : AHTSCL_Pin AHTSDA_Pin */
  GPIO_InitStruct.Pin = AHTSCL_Pin|AHTSDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1C_Pin IN1B_Pin */
  GPIO_InitStruct.Pin = IN1C_Pin|IN1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1A_Pin IN2C_Pin IN2B_Pin IN2A_Pin
                           IN3C_Pin IN3B_Pin IN3A_Pin */
  GPIO_InitStruct.Pin = IN1A_Pin|IN2C_Pin|IN2B_Pin|IN2A_Pin
                          |IN3C_Pin|IN3B_Pin|IN3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : EVCBRK1_Pin */
  GPIO_InitStruct.Pin = EVCBRK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EVCBRK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EVCEN2_Pin */
  GPIO_InitStruct.Pin = EVCEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EVCEN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q128_CS_Pin */
  GPIO_InitStruct.Pin = W25Q128_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(W25Q128_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EVCBRK2_Pin */
  GPIO_InitStruct.Pin = EVCBRK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EVCBRK2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
