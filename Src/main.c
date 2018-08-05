
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l0xx_nucleo.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "WiMOD_LoRaWAN_API.h"
#include "eeprom.h"
#include "cayenne_lpp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
sensor_t dSersor;
user_setting_t uSetting;
uint8_t data[255];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint8_t Cayenne_LPP_parse(sensor_t *data, uint8_t * payload, size_t size);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void *hTemperatur;
void *hHumidity;
void *hPressure;
void *hAccel;
void *hGyro;
void *hMagneto;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	memset(&dSersor, 0x0, sizeof(sensor_t));
	memset(&uSetting, 0x0,sizeof(user_setting_t));

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  EEPROM_GetUserSetting(0, (uint32_t*) &uSetting, sizeof(user_setting_t));
  //Initialize BSP sensor
//  Sensor_IO_Init();
	if (BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &hTemperatur)
			!= COMPONENT_OK)
		printf("Initialize temperature missing!!!\r\n");
	if (BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &hHumidity) != COMPONENT_OK)
		printf("Initialize humidity missing!!!\r\n");
	if (BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &hPressure) != COMPONENT_OK)
		printf("Initialize pressure missing!!!\r\n");
	if (BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &hAccel) != COMPONENT_OK)
		printf("Initialize accelero missing!!!\r\n");
	if (BSP_GYRO_Init(GYRO_SENSORS_AUTO, &hGyro) != COMPONENT_OK)
		printf("Initialize gyro missing!!!\r\n");
	if (BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &hMagneto) != COMPONENT_OK)
		printf("Initialize magneto missing!!!\r\n");

	  BSP_HUMIDITY_Sensor_Enable( hHumidity );
	  BSP_TEMPERATURE_Sensor_Enable( hTemperatur );
	  BSP_PRESSURE_Sensor_Enable( hPressure );
	  BSP_ACCELERO_Sensor_Enable( hAccel );
	  BSP_GYRO_Sensor_Enable( hGyro );
	  BSP_MAGNETO_Sensor_Enable( hMagneto );

  //Initialize BSP network
  WiMOD_LoRaWAN_Init(&huart2);
  WiMOD_LoRaWAN_Reset();
  HAL_Delay(200);
  WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_ACTIVATE_DEVICE_REQ,
			(uint8_t *) &uSetting.dev_addr, 36);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //Read sensor
	  //Parse Cayenne
//	  Cayenne_LPP_parse(&sensor, AppData->Buff, 255);
	  //Send data
	  WiMOD_LoRaWAN_Process();
	  HAL_Delay(1);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIS3MDL_DRDY_Pin LIS3MDL_INT1_Pin */
  GPIO_InitStruct.Pin = LIS3MDL_DRDY_Pin|LIS3MDL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIL24_INT1_Pin USER_INT_Pin */
  GPIO_InitStruct.Pin = DIL24_INT1_Pin|USER_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIL24_INT2_Pin */
  GPIO_InitStruct.Pin = DIL24_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIL24_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HTS221_DRDY_Pin LPS25HB_INT1_Pin LSM6DS0_INT1_Pin */
  GPIO_InitStruct.Pin = HTS221_DRDY_Pin|LPS25HB_INT1_Pin|LSM6DS0_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
static uint8_t Cayenne_LPP_parse(sensor_t *data, uint8_t * payload, size_t size) {
  uint8_t cursor = 0;
  CayenneLPP_Init(size);

//  cayenne_payload_addDigitalInput(uint8_t channel, uint8_t value);
//  cayenne_payload_addDigitalOutput(uint8_t channel, uint8_t value);
//
//  cayenne_payload_addAnalogInput(uint8_t channel, float value);
//  cayenne_payload_addAnalogOutput(uint8_t channel, float value);

//  cayenne_payload_addLuminosity(LPP_CHANNEL_LUX, data->luminosity);
  cayenne_payload_addTemperature(LPP_CHANNEL_OUTER_TEMPERATURE, data->temperture);
  cayenne_payload_addRelativeHumidity(LPP_CHANNEL_HUMIDITY, data->humidity);
  cayenne_payload_addAccelerometer(LPP_CHANNEL_ACCELERO, data->accel_x, data->accel_y, data->accel_z);
  cayenne_payload_addBarometricPressure(LPP_CHANNEL_BAR_PRESSURE, data->pressure);
  cayenne_payload_addGyrometer(LPP_CHANNEL_GYRO, data->gyro_x, data->gyro_y, data->gyro_z);
  cayenne_payload_addMagnetometer(LPP_CHANNEL_MAGNETO, data->magnet_x, data->magnet_y, data->magnet_z);
//  cayenne_payload_addGPS(LPP_CHANNEL_GPS, data->latitude, data->longitude, data->altitudeGps);
//  cayenne_payload_addVoltage(LPP_CHANNEL_VOLTAGE, data->power_meter.vin);
//  cayenne_payload_addCurrent(LPP_CHANNEL_CURRENT, data->power_meter.iin);
//  cayenne_payload_addPower(LPP_CHANNEL_POWER_CONSUMP, data->power_meter.power);
//  cayenne_payload_addPercentage(LPP_CHANNEL_BATTERY_LVL, data->batteryLevel);
//  cayenne_payload_addPresence(LPP_CHANNEL_RAIN, data->rain.detect);
//  cayenne_payload_addPercentage(LPP_CHANNEL_RAIN_LVL, data->rain.lvl);
//  cayenne_payload_addEnergy(LPP_CHANNEL_ENERGY, data->power_meter.energy);
//  cayenne_payload_addPower(LPP_CHANNEL_POWER_PANEL, data->power_meter.vout * data->power_meter.ipanel);
//  cayenne_payload_addTemperature(LPP_CHANNEL_INNER_TEMPERATURE, data->weather_data.internal_temp);

  cursor = cayenne_payload_copy(payload);

  CayenneLPP_Deinit();
  return cursor;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SensorAxes_t axes;
  /* Prevent unused argument(s) compilation warning */
  switch (GPIO_Pin) {
	case GPIO_PIN_0:	//LIS3MDL_DRDY	3-axis magnetometer data ready
		BSP_MAGNETO_Get_Axes(&hMagneto, &axes);
		dSersor.magnet_x = axes.AXIS_X;
		dSersor.magnet_y = axes.AXIS_Y;
		dSersor.magnet_z = axes.AXIS_Z;
		break;
	case GPIO_PIN_1:	//LIS3MDL_INT1	3-axis magnetometer interrupt
		//P-N Threshold, range overflow
			break;
	case GPIO_PIN_4:	//LPS25HB_INT1	pressure interrupt
		BSP_PRESSURE_Get_Press(&hPressure, &dSersor.pressure);
			break;
	case GPIO_PIN_5:	//LSM6DS0_INT1	3-axis accelerometer plus 3-axis gyroscope
		BSP_ACCELERO_Get_Axes(&hAccel, &axes);
		dSersor.accel_x = axes.AXIS_X;
		dSersor.accel_y = axes.AXIS_Y;
		dSersor.accel_z = axes.AXIS_Z;

		BSP_GYRO_Get_Axes(&hGyro, &axes);
		dSersor.gyro_x = axes.AXIS_X;
		dSersor.gyro_y = axes.AXIS_Y;
		dSersor.gyro_z = axes.AXIS_Z;
			break;
	case GPIO_PIN_10:	//HTS221_DRDY	humidity data ready
		BSP_HUMIDITY_Get_Hum(&hHumidity, &dSersor.humidity);
		BSP_TEMPERATURE_Get_Temp(&hTemperatur, &dSersor.temperture);
			break;
	case GPIO_PIN_13:	//B1 [Blue PushButton]
		BSP_LED_On(LED2);

//		HAL_Delay(100);
		//LoRaWAN Data send
		WiMOD_LoRaWAN_SendURadioData(10, data, 35);
		BSP_LED_Off(LED2);

			break;
	default:
		break;
  }

  //Cayenne LPP payload parse
  Cayenne_LPP_parse(&dSersor, data, sizeof(data));
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
