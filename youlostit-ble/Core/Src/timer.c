/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

#define _ARR 400
#define CLOCK_SCALEDOWN 1
#define CLOCK_FREQ_DEFAULT 8000000

void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this
	timer->CNT = 0;
	timer->ARR = _ARR;
	timer->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ((TIM2_IRQn));
	NVIC_SetPriority((TIM2_IRQn), 0x1);

	//100KHZ = 1000000 HZ = Downscale * freqency * ARR * psc  = 1 * 1 * 200 * 500
	//4MHZ = 4000000 Hz = Downscale * freqency * ARR * psc  = 1 * 20 * 200 * 1000
	//8MHZ = 8000000 Hz = Downscale * frequency * ARR * psc = 1 * 1 * 200 * 40000
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
	//APB scales

	timer->CR1 |= TIM_CR1_CEN;


}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this
	timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // TODO implement this

	uint16_t prescale = (CLOCK_FREQ_DEFAULT * period_ms) / (CLOCK_SCALEDOWN * _ARR * 1000);

	printf("Prescale:%d\n", prescale);

	timer->PSC  = prescale;

}
