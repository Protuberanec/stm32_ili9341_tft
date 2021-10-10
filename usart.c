/*
 * usart.c
 *
 *  Created on: 1 нояб. 2020 г.
 *      Author: еу
 */


#include "usart.h"

uint8_t rx_data_buf[USART_RX_BUFFER];
cBuffer bufferRX;

void USART1_IRQHandler() {
	if (USART1->ISR & USART_ISR_RXNE) {
		uint8_t data_rx = USART1->RDR;
		bufferAddToEnd(&bufferRX, data_rx);
	}
}

void usart1_init() {
	bufferInit(&bufferRX, &rx_data_buf[0], USART_RX_BUFFER);

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->AFR[1] |= (1 << 4)| (1 << 8);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;	//enable transmit and receive
	USART1->CR3 |= USART_CR3_OVRDIS;	//only for debug!!!

	USART1->BRR = SystemCoreClock/115200;

	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 3);
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1|= USART_CR1_UE;
}

void usart1_SendTestData() {
	for (uint16_t data_for_usart = 0; data_for_usart < 255; data_for_usart++) {
		USART1->TDR = (uint8_t)data_for_usart;
		while(!(USART1->ISR & USART_ISR_TXE));
	}
}

void usart1_SlowSend1Byte(uint8_t byte) {
	while(!(USART1->ISR & USART_ISR_TXE));
	USART1->TDR = byte;
}

uint16_t usart1_buffer() {
	return bufferHaveData(&bufferRX);
}

uint8_t usart1_getByteFromBuffer() {
	return bufferGetFromFront(&bufferRX);
}

