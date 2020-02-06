/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <math.h>
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
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define dir2_Pin GPIO_PIN_2
#define dir2_GPIO_Port GPIOE
#define L298_ENA1_Pin GPIO_PIN_3
#define L298_ENA1_GPIO_Port GPIOE
#define L298_ENA2_Pin GPIO_PIN_4
#define L298_ENA2_GPIO_Port GPIOE
#define out2_Pin GPIO_PIN_8
#define out2_GPIO_Port GPIOC
#define int1_M_cpt_t_Pin GPIO_PIN_9
#define int1_M_cpt_t_GPIO_Port GPIOC
#define h1_inter_Pin GPIO_PIN_1
#define h1_inter_GPIO_Port GPIOD
#define h1_inter_EXTI_IRQn EXTI1_IRQn
#define h2_inter_Pin GPIO_PIN_2
#define h2_inter_GPIO_Port GPIOD
#define h2_inter_EXTI_IRQn EXTI2_IRQn
#define pin_error_Pin GPIO_PIN_3
#define pin_error_GPIO_Port GPIOD
#define pin_error_EXTI_IRQn EXTI3_IRQn
#define dir1_Pin GPIO_PIN_1
#define dir1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define TIEMPO_SAMP 0.0002
#define VEL_MAX 11.7705
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
