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
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"

#include "stm32g0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "foc_utils.h" 
#include "FOCMotor.h" 
#include "BLDCmotor.h" 
	
#include "lowpass_filter.h" 
#include "pid.h"
	
#include "MagneticSensor.h" 
#include "LowsideCurrentSense.h"
#include "CurrentSense.h"

#include "flash_WR.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t running_state, error_code;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define M1_Enable    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);          //高电平使能
#define M1_Disable   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);        //低电平失能
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_Period 1280
#define I2C_ADD 0x10
#define SOC_Pin GPIO_PIN_3
#define SOC_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_4
#define SOB_GPIO_Port GPIOA
#define SOA_Pin GPIO_PIN_5
#define SOA_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_6
#define EN_GPIO_Port GPIOC
#define nSLEEP_Pin GPIO_PIN_11
#define nSLEEP_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_12
#define nFAULT_GPIO_Port GPIOA
#define TLE_CS_Pin GPIO_PIN_15
#define TLE_CS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
