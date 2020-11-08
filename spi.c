/*
 * spi.c
 *
 *  Created on: 11 окт. 2020 г.
 *      Author: Tugrik
 *
 *      this library is not universal... but very fast!!! I hope
 */

#include "spi.h"
//----------------SPI1-------------------------
/*
 *  pc4 - nss
 *  pa5 - sck
 *  pa6 - miso
 *  pa7 - mosi
 */

uint8_t spi_buffer_tx[BUFFER_SPI_SIZE];
uint8_t spi_buffer_rx[BUFFER_SPI_SIZE];

static void spi1_cs_set();


void SPI1_IRQHandler() {
	if (SPI1->SR & SPI_SR_RXNE) {

	}
}

uint16_t status_dma_tx;

void DMA1_Channel2_3_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF3) {
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		status_dma_tx = 1;
	}

	if (DMA1->ISR & DMA_ISR_TCIF2) {
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
		spi1_cs_set();	//when transfer is end CS is up, если использовать по передаче, то прерывания срабатывает не когда
						//заканчивается передача данных, а когда опустошается буффер,
						//программный чип селект конечно не лучший вариант, но возможно на этот же spi повесить ещё карту памяти
						//или другую периферию...
		status_dma_tx = 1;
	}
}

uint8_t spi1_dma_end() {
	return status_dma_tx;
}

static void spi1_cs_set() {
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
	DMA1_Channel3->CCR &= ~DMA_CCR_HTIE;

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

void spi1_master_init(uint8_t am_bits_send) {
	gpio_spi1_init();

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 &= ~SPI_CR1_SPE;

//#define DEBUG
#ifdef DEBUG
	SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_2;
#else
	SPI1->CR1 &= ~SPI_CR1_BR;	// system_core_freq / 2
#endif


	SPI1->CR1 |= SPI_CR1_MSTR;	//master
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;	//software control by nss or SPI1->CR2 |= SPI_CR2_SSOE;

	uint16_t DataSize = SPI1->CR2 & ~SPI_CR2_DS;
	DataSize |= (am_bits_send - 1) << 8;
	SPI1->CR2 = DataSize;

	SPI1->CR2 |= SPI_CR2_FRXTH;

	spi1_dma_tx_init();
	spi1_dma_rx_init();

	SPI1->CR1 |= SPI_CR1_SPE;
}

void spi1_setDataSize(uint8_t am_bits_send) {
	SPI1->CR1 &= ~SPI_CR1_SPE;
	uint16_t DataSize = SPI1->CR2 & ~SPI_CR2_DS;
	DataSize |= (am_bits_send - 1) << 8;
	SPI1->CR2 = DataSize;
	SPI1->CR1 |= SPI_CR1_SPE;
}

//here need to update the spi_buffer_tx
void spi1_SendGetDataDMA_1Byte(uint8_t* data, uint16_t count_byte) {
	status_dma_tx = 0;
	spi1_setDataSize(8);

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	DMA1_Channel2->CNDTR = count_byte;
	DMA1_Channel2->CCR |= DMA_CCR_EN;


	//ooo may be copy data from local to global with DMA?? mem to mem
	for (int i = 0; i < count_byte; i++) {
		spi_buffer_tx[i] = data[i];
	}

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CNDTR = count_byte;
	DMA1_Channel3->CCR |= DMA_CCR_MINC;	//increment memory
	DMA1_Channel3->CCR &= ~DMA_CCR_MSIZE;
	DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE;	//size 8bits of data and peripheral

	spi1_cs_clear();
	DMA1_Channel3->CCR |= DMA_CCR_EN;	//start send data
}

void spi1_SendGetDataDMA_2byte(uint16_t *data, uint16_t count_word) {
	status_dma_tx = 0;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	if (count_word > 0) {
		DMA1_Channel2->CNDTR = count_word << 1;	//if not mult by 2, CS will up in the middle of sending data
		DMA1_Channel2->CCR |= DMA_CCR_EN;
	}

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)(&spi_buffer_tx[0]);
	DMA1_Channel3->CNDTR = count_word;

	spi1_setDataSize(16);

	count_word = count_word << 1;
	for (int i = 0; i < count_word; i += 2, data++) {
		spi_buffer_tx[i] =  0x00ff & (*data >> 8);
		spi_buffer_tx[i+1] = 0x00ff & (*data);
	}


	spi1_cs_clear();
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

//only send data....
void spi1_SendDataDMA_2byteNTimes(uint16_t data, uint16_t count_word) {
	status_dma_tx = 0;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR &= ~DMA_CCR_MINC;

	count_word = count_word << 1;	//на каждый пиксель по 2 байта, поэтому необходимо это число умножить на 2!!!

	if (count_word > 0) {
		DMA1_Channel2->CNDTR = count_word;	//if not mult by 2, CS will up in the middle of sending data
		DMA1_Channel2->CCR |= DMA_CCR_EN;
	}

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	DMA1_Channel3->CCR &= ~DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)(&spi_buffer_tx[0]);
	DMA1_Channel3->CNDTR = count_word;

	spi1_setDataSize(16);

//prepare data to send, because DMA don't work with local var, when function will finished local var will destroyed nad lost memory
	spi_buffer_tx[0] =  0x00ff & (data >> 8);
	spi_buffer_tx[1] = 0x00ff & (data);

	spi1_cs_clear();
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}
//don't use....
void spi1_putchar(uint16_t data) {

	spi1_cs_set();

	*(uint16_t *)&(SPI1->DR) = data;
	while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);

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

