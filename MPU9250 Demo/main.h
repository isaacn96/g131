/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESC_PWM_1_Pin GPIO_PIN_1
#define ESC_PWM_1_GPIO_Port GPIOA
#define ESC_PWM_2_Pin GPIO_PIN_2
#define ESC_PWM_2_GPIO_Port GPIOA
#define ESC_PWM_3_Pin GPIO_PIN_3
#define ESC_PWM_3_GPIO_Port GPIOA
#define DIR_3_Pin GPIO_PIN_4
#define DIR_3_GPIO_Port GPIOA
#define DIR_2_Pin GPIO_PIN_5
#define DIR_2_GPIO_Port GPIOA
#define SERVO_1_Pin GPIO_PIN_6
#define SERVO_1_GPIO_Port GPIOA
#define SERVO_2_Pin GPIO_PIN_7
#define SERVO_2_GPIO_Port GPIOA
#define SERVO_3_Pin GPIO_PIN_0
#define SERVO_3_GPIO_Port GPIOB
#define DIR_1_Pin GPIO_PIN_1
#define DIR_1_GPIO_Port GPIOB
#define BOOT_1_Pin GPIO_PIN_2
#define BOOT_1_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_13
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_14
#define LED_3_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_15
#define LED_4_GPIO_Port GPIOB
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define TACH_1_Pin GPIO_PIN_3
#define TACH_1_GPIO_Port GPIOB
#define TACH_2_Pin GPIO_PIN_4
#define TACH_2_GPIO_Port GPIOB
#define TACH_3_Pin GPIO_PIN_5
#define TACH_3_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
