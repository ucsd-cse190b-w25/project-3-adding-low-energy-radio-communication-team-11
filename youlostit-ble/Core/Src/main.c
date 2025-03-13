/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include <rtc.h>
#include "ble.h"
#include "timer.h"
#include "i2c.h"
#include "lsm6dsl.h"
#include <stdlib.h>
#include <string.h>

#define TIMER_PERIOD 2000
#define TIM_UIF 0x1

//movement threshold = 0.1g
#define THRESHOLD 1638

//maximum wait_cnt before led lights up
#define MAX_WAIT_CNT 5

#define LOST_MODE_MSG_INTERVAL 5

/* counts the amount of time the tag is stationary */
volatile uint32_t stationary_cnt;

/*counts the amount of time the tag has been lost(in lost mode)*/
volatile uint32_t lost_cnt;

volatile uint8_t signal_read;
volatile uint8_t signal_lost_msg;

/*boolean that indicate the current operating mode*/
volatile uint8_t isLost;

/*RTC handle*/
RTC_HandleTypeDef hrtc;

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);


void OnEnterSleepMode()
{
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_SPI3_CLK_DISABLE();

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOE_CLK_DISABLE();

}

void OnEnterRunMode()
{
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_SPI3_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

}

void SwitchToLowFrequency()
{
  // Switch to MSI range 0 = 100 kHz to save power.
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0); // Set MSI to 100KHz
  SystemCoreClockUpdate();
}

void SwitchToHighFrequency()
{
  // Switch to MSI range 7 = 8 MHz for BLE operations.
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7); // Set MSI to 8MHz
  SystemCoreClockUpdate();
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{

	//printf("Entering Event Callback\n");

	/*stationary counter logic*/
	if (stationary_cnt >= MAX_WAIT_CNT)
	{

		lost_cnt++;
		isLost = 0x1;
	}
	else
	{
		lost_cnt = 0;
		isLost = 0x0;
	}

	/* 10 second interval for message */
	if (lost_cnt % LOST_MODE_MSG_INTERVAL == 1){
		signal_lost_msg = 1;
	}

	/* signal the accelerometer to read*/
	signal_read = 0x1;


}

void RTC_WKUP_IRQHandler()
{

	HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);

}

int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

uint8_t DectectMotion(int16_t AccelX, int16_t AccelY, int16_t AccelZ, int16_t prevX, int16_t prevY, int16_t prevZ)
{
	if (abs(prevX -AccelX) > THRESHOLD || abs(prevY -AccelY) > THRESHOLD || abs(prevZ -AccelZ) > THRESHOLD){
		return 1;
	}
	else
		return 0;

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();

	RTC_Init(&hrtc);

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI3_Init();

	//RESET BLE MODULE
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

	ble_init();

	HAL_Delay(10);

	uint8_t nonDiscoverable = 0;
	uint8_t disconnected = 0;


	stationary_cnt = 0x0;
	signal_read = 0x0;
	isLost = 0x0;

	i2c_init();

	lsm6dsl_init();

	int init = 1;

	int16_t AccelX, AccelY, AccelZ;
	int16_t prevX, prevY, prevZ;

	uint8_t motionDetected = 0;


	SwitchToLowFrequency();
	while (1)
	{
		if(signal_read){

			prevX = AccelX;
			prevY = AccelY;
			prevZ = AccelZ;

			lsm6dsl_read_xyz(&AccelX, &AccelY, &AccelZ);

			motionDetected = DectectMotion(AccelX, AccelY, AccelZ, prevX, prevY, prevZ);
			if (motionDetected){
				stationary_cnt = 0x0;

				if (!init){
					SwitchToHighFrequency();
					disconnectBLE();
					disconnected = 1;
					SwitchToLowFrequency();
				}
				if (disconnected)
				{
					SwitchToHighFrequency();
					setDiscoverability(0);
					standbyBle();
					nonDiscoverable = 1;
					SwitchToLowFrequency();
				}

				printf("prevXYZ:(%d,%d,%d) ; currentXYZ:(%d,%d,%d)", prevX, prevY, prevZ, AccelX, AccelY, AccelZ);
			}
			else {
				stationary_cnt++;
			}

			if (isLost){

				if (init){
					init = 0;
				}
				else if (nonDiscoverable){
					SwitchToHighFrequency();
					setDiscoverability(1);
					disconnected = 0;
					nonDiscoverable = 0;
					SwitchToLowFrequency();
				}
			}

			signal_read = 0x0;
		}

		if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
			SwitchToHighFrequency();
			catchBLE();
			SwitchToLowFrequency();
		}else{

			if (signal_lost_msg){
				SwitchToHighFrequency();
				unsigned char lost_str1[] = "Peartag: lost for ";
				unsigned char lost_str2[] = "%08lu seconds";
				char buffer[17];

				snprintf(buffer, sizeof(buffer), lost_str2, lost_cnt*2);

				updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(lost_str1)-1, lost_str1);

				updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(buffer)-1, buffer);


				printf("lost sec=%d\n", lost_cnt*2);
				signal_lost_msg = 0;
				SwitchToLowFrequency();
			}
		}



		//Go to STOP0 mode (DEEPSLEEP with MR on)

		//PWR->CR1 |= PWR_CR1_LPR;

		//SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

/*		HAL_SuspendTick();

		OnEnterSleepMode();

		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		OnEnterRunMode();

		HAL_ResumeTick();*/

		//SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	}
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_CR_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
