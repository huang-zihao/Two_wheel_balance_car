/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include "../../../Hardware/MPU6050/mpu6050.h"
#include "../../../Hardware/MPU6050/delay.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu_dmp_motion_driver.h" 

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
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_6
#define PWMB_GPIO_Port GPIOA
#define AOUT1_Pin GPIO_PIN_12
#define AOUT1_GPIO_Port GPIOB
#define BOUT1_Pin GPIO_PIN_13
#define BOUT1_GPIO_Port GPIOB
#define AOUT2_Pin GPIO_PIN_14
#define AOUT2_GPIO_Port GPIOB
#define BOUT2_Pin GPIO_PIN_15
#define BOUT2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void rotation(int tmp);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */