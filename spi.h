/*
 * spi.h
 *
 *  Created on: 11 окт. 2020 г.
 *      Author: Tugrik
 */

#ifndef SPI_H_
#define SPI_H_

#include <stm32f0xx.h>

void gpio_spi1_init();
void spi1_master_init();
void spi1_putchar(uint16_t data);
uint8_t spi1_getchar(uint16_t *data);

void spi1_SendDataDMA(uint8_t *data, uint16_t write_size, uint16_t read_size);



void gpio_spi2_init();
void spi2_slave_init();
void spi2_putchar(uint16_t data);
uint8_t spi2_getchar(uint16_t *data);


#endif /* SPI_H_ */
