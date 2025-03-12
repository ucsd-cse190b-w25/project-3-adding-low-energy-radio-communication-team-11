/*
 * rtc_lsi.c
 *
 *  Created on: Mar 10, 2025
 *      Author: Kaiji
 */

#include <rtc.h>


#define LSI_FREQ 32000
#define LSE_FREQ 32768


void RTC_Init(RTC_HandleTypeDef* hrtc) {

	// Enable write access to the backup domain
	__HAL_RCC_PWR_CLK_ENABLE(); // Enable the PWR clock
	HAL_PWR_EnableBkUpAccess(); // Enable backup domain access

	// Reset the backup domain (including the RTC registers)
	__HAL_RCC_BACKUPRESET_FORCE();
	__HAL_RCC_BACKUPRESET_RELEASE();


	hrtc->Instance = RTC;
	hrtc->Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc->Init.AsynchPrediv = 127;
	hrtc->Init.SynchPrediv = LSE_FREQ / (127+1) - 1;
	hrtc->Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc->Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
	Error_Handler();
	}


	__HAL_RCC_RTC_ENABLE();


	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0, 0);

	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    // Enable RTC wake-up interrupt at RTC level
    __HAL_RTC_WAKEUPTIMER_ENABLE_IT(hrtc, RTC_IT_WUT);

    // Enable EXTI line 22 for RTC wake-up
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_RISING_EDGE();


	if (HAL_RTCEx_SetWakeUpTimer_IT(hrtc, LSE_FREQ/16*2 - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
	{
		Error_Handler();
	}



}

