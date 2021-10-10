/*
 * usart.h
 *
 *  Created on: 1 нояб. 2020 г.
 *      Author: еу
 */

#ifndef USART_H_
#define USART_H_

#include <stm32f0xx.h>
#include "buffer.h"

#define USART_RX_BUFFER	64

void usart1_init();
void usart1_SendTestData();
void usart1_SlowSend1Byte(uint8_t byte);

uint16_t usart1_buffer();
uint8_t usart1_getByteFromBuffer();


#endif /* USART_H_ */
