/*
 * spi.c
 *
 *  Created on: 11 окт. 2020 г.
 *      Author: Tugrik
 */

#include "spi.h"
//----------------SPI1-------------------------
/*
 *  pc4 - nss
 *  pa5 - sck
 *  pa6 - miso
 *  pa7 - mosi
 */

uint8_t spi_buffer_tx[1024];
uint8_t spi_buffer_rx[32];

void SPI1_IRQHandler() {

}

void DMA1_Channel2_3_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF3) {
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
	}

	if (DMA1->ISR & DMA_ISR_TCIF2) {
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
//		spi1_cs_set();
	}
}

void spi1_cs_set() {
	GPIOC->BSRR = GPIO_BSRR_BS_4;
}

void spi1_cs_clear() {
	GPIOC->BSRR = GPIO_BSRR_BR_4;
}

void gpio_spi1_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER4_0;	//cs
	spi1_cs_set();

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;	//as AF0
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
}

static void spi1_dma_tx_init() {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel3->CMAR = (uint32_t)(&spi_buffer_tx[0]);
	DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CCR |= DMA_CCR_DIR;
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	SPI1->CR2 |= SPI_CR2_TXDMAEN;
}

static void spi1_dma_rx_init() {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel2->CMAR = (uint32_t)(&spi_buffer_rx[0]);
	DMA1_Channel2->CPAR = (uint32_t)(&(SPI1->DR));
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 7);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	SPI1->CR2 |= SPI_CR2_RXDMAEN;
}

void spi1_master_init() {

	for (int i = 0; i < 1024; i++) {
		spi_buffer_tx[i] = i;
	}

	gpio_spi1_init();

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 |= SPI_CR1_BR_1;	//fcplk/8 = 1MHz
	SPI1->CR1 |= SPI_CR1_MSTR;	//master

	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;	//software control by nss

	SPI1->CR2 &= ~SPI_CR2_DS_3;	//maxima speed is equal 4Mhz

//	SPI1->CR2 |= SPI_CR2_SSOE;

	spi1_dma_tx_init();
	spi1_dma_rx_init();

	SPI1->CR1 |= SPI_CR1_SPE;
}

//here need to update the spi_buffer_tx
void spi1_SendDataDMA(uint8_t* data, uint16_t write_size, uint16_t read_size) {
//for read data init DMA
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CNDTR = read_size;
	if (read_size > 0) {
		DMA1_Channel2->CCR |= DMA_CCR_EN;
	}

//for write data init DMA
	for (int i = 0; i < write_size; i++) {
		spi_buffer_tx[i] = *(data + i);
	}

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CNDTR = write_size;
	spi1_cs_clear();
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}


void spi1_putchar(uint16_t data) {

	spi1_cs_set();

	*(uint16_t *)&(SPI1->DR) = data;
	while(SPI1->SR & SPI_SR_TXE != SPI_SR_TXE);

	for (int i = 0; i < 16; i++);

	spi1_cs_clear();
}

//----------------SPI2-------------------------
uint16_t data_spi2;
void SPI2_IRQHandler() {
	if (SPI2->SR | SPI_SR_RXNE) {
		data_spi2 = SPI2->DR;
	}
}
/*
 * pb12 - nss
 * pb13 - sck
 * pb14 - miso
 * pb15 - mosi
 */
void gpio_spi2_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER12_1;	//input as CS
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;	//as AF0
}

void spi2_slave_init() {
	gpio_spi2_init();

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPI2->CR1 |= SPI_CR1_BR_1;	//fcplk/8 = 1MHz
	SPI2->CR1 &= ~SPI_CR1_MSTR;	//slave

	SPI2->CR2 |= SPI_CR2_DS;	//16 bit

	SPI2->CR2 = SPI_CR2_RXNEIE | SPI_CR2_FRXTH;
	NVIC_SetPriority(SPI2_IRQn, 2);
	NVIC_EnableIRQ(SPI2_IRQn);

	SPI2->CR1 |= SPI_CR1_SPE;
}

