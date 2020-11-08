/*
 * spi.h
 *
 *  Created on: 11 окт. 2020 г.
 *      Author: Tugrik
 */

#ifndef SPI_H_
#define SPI_H_

#include <stm32f0xx.h>

#define BUFFER_SPI_SIZE	128

void gpio_spi1_init();
void spi1_master_init(uint8_t am_bits_send);
void spi1_putchar(uint16_t data);
uint8_t spi1_getchar(uint16_t *data);

void spi1_setDataSize(uint8_t size_data);


void spi1_SendGetDataDMA_1Byte(uint8_t *data, uint16_t count_byte);
void spi1_SendGetDataDMA_2byte(uint16_t *data, uint16_t count_repeat);
void spi1_spec_SendDataDMA_2byte(uint16_t data, uint16_t count_word);
uint8_t spi1_dma_end();


void gpio_spi2_init();
void spi2_slave_init();
void spi2_putchar(uint16_t data);
uint8_t spi2_getchar(uint16_t *data);


#endif /* SPI_H_ */
