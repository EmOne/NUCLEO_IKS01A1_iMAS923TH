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
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LIS3MDL_DRDY_Pin GPIO_PIN_0
#define LIS3MDL_DRDY_GPIO_Port GPIOC
#define LIS3MDL_DRDY_EXTI_IRQn EXTI0_1_IRQn
#define LIS3MDL_INT1_Pin GPIO_PIN_1
#define LIS3MDL_INT1_GPIO_Port GPIOC
#define LIS3MDL_INT1_EXTI_IRQn EXTI0_1_IRQn
#define LORA_TX_Pin GPIO_PIN_2
#define LORA_TX_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_3
#define LORA_RX_GPIO_Port GPIOA
#define DIL24_INT1_Pin GPIO_PIN_4
#define DIL24_INT1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DIL24_INT2_Pin GPIO_PIN_0
#define DIL24_INT2_GPIO_Port GPIOB
#define HTS221_DRDY_Pin GPIO_PIN_10
#define HTS221_DRDY_GPIO_Port GPIOB
#define HTS221_DRDY_EXTI_IRQn EXTI4_15_IRQn
#define USER_INT_Pin GPIO_PIN_10
#define USER_INT_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LPS25HB_INT1_Pin GPIO_PIN_4
#define LPS25HB_INT1_GPIO_Port GPIOB
#define LPS25HB_INT1_EXTI_IRQn EXTI4_15_IRQn
#define LSM6DS0_INT1_Pin GPIO_PIN_5
#define LSM6DS0_INT1_GPIO_Port GPIOB
#define LSM6DS0_INT1_EXTI_IRQn EXTI4_15_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
typedef struct {
	float temperture;
	float pressure;
	float humidity;
	int32_t gyro_x;
	int32_t gyro_y;
	int32_t gyro_z;
	int32_t accel_x;
	int32_t accel_y;
	int32_t accel_z;
	int32_t magnet_x;
	int32_t magnet_y;
	int32_t magnet_z;
	float latitude;
	float longitude;
	float altitudeGps;
	uint8_t di;
} sensor_t;

typedef struct {
	bool activation_mode; //0:ABP 1:OTAA
	uint8_t	dev_eui [8];
	uint8_t	app_eui [8];
	uint8_t dev_addr [4];
	uint8_t	nkw_key [16];
	uint8_t	app_key [16];
} user_setting_t;

extern sensor_t dSersor;
extern user_setting_t uSetting;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
