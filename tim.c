/*
 * tim.c
 *
 *  Created on: 8 нояб. 2020 г.
 *      Author: еу
 */

#include "tim.h"

void tim14_init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	TIM14->ARR = 48000 -1;
	TIM14->PSC = 1000 - 1;

	TIM14->DIER |= TIM_DIER_UIE;

	NVIC_SetPriority(TIM14_IRQn, 5);
	NVIC_EnableIRQ(TIM14_IRQn);

	TIM14->CR1 |= TIM_CR1_CEN;
}

