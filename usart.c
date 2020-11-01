/*
 * usart.c
 *
 *  Created on: 1 нояб. 2020 г.
 *      Author: еу
 */


#include "usart.h"

uint8_t rx_data_buf[USART_RX_BUFFER];
cBuffer rx_buffer;

void USART1_IRQHandler() {
	if (USART1->ISR & USART_ISR_RXNE) {
		uint8_t data_rx = USART1->RDR;
		bufferAddToEnd(&rx_buffer, data_rx);
	}
}

void usart1_init() {
	bufferInit(&rx_buffer, &rx_data_buf[0], USART_RX_BUFFER);

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->AFR[1] |= (1 << 4)| (1 << 8);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;	//enable transmit and receive
	USART1->CR3 |= USART_CR3_OVRDIS;	//only for debug!!!

	USART1->BRR = 8000000/115200;

	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 3);
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1|= USART_CR1_UE;
}
