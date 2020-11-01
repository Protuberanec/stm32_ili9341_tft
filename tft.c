/*
 * tft.c
 *
 *  Created on: 31 окт. 2020 г.
 *      Author: еу
 */

#include "tft.h"

extern uint8_t spi_buffer_tx[1024];

void tft_gpio_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER0_0;	//LED
	GPIOB->MODER |= GPIO_MODER_MODER1_0;	//DC/RS

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER5_0;	//reset
}

void tft_data() {
	GPIOB->BSRR = GPIO_BSRR_BS_1;
}

void tft_cmd() {
	GPIOB->BSRR = GPIO_BSRR_BR_1;
}

void tft_led_on() {
	GPIOB->BSRR = GPIO_BSRR_BS_0;
}

void tft_led_off() {
	GPIOB->BSRR = GPIO_BSRR_BR_0;
}

void tft_reset() {
	GPIOC->BSRR = GPIO_BSRR_BS_5;
}

void tft_init() {
	tft_gpio_init();

	spi1_master_init();
	tft_led_on();
	for (int i = 0; i < 100; i++);

	tft_reset();
	for (int i = 0; i < 20000; i++);

	tft_sleep_out();

	tft_display_normal_mode();
	tft_display_on();
	tft_pixel_format();

//	tft_read_display_id();
//	tft_read_display_status();

//	tft_write_cmd(0xEF, (uint8_t*){0x03, 0x80, 0x02}, 3);
//	tft_write_cmd(0xCF, (uint8_t*){0x00, 0xC1, 0x30}, 3);
//	tft_write_cmd(0xED, (uint8_t*){0x64, 0x03, 0x12, 0x81}, 4);
//	tft_write_cmd(0xE8, (uint8_t*){0x85, 0x00, 0x78}, 3);

	tft_fill_color(0,0,0);


}

void tft_write_cmd(uint8_t cmd, uint8_t *data, uint8_t size) {
	tft_cmd();
	spi1_SendDataDMA(&cmd, 1, 0);
	for (int i = 0; i < 10; i++);
	if (size == 0) {
		for (int i = 0; i < 10; i++);
		return;
	}
	tft_data();
	spi1_SendDataDMA(&data[0], size, size);
	for (int i = 0; i < 10; i++);
}

void tft_sleep_out() {
	tft_cmd();
	uint8_t data = 0x11;
	spi1_SendDataDMA(&data, 1, 0);
	for (int i = 0; i < 100000; i++);
}

void tft_display_off() {
	tft_write_cmd(0x28, 0, 0);
}

void tft_display_on() {
	tft_write_cmd(0x29, 0, 0);
}
//it's not work alway return 0x00
void tft_read_display_id() {
	uint8_t data[5] = {0x04, 0x00, 0x00, 0x00, 0x00};
	tft_cmd();
	spi1_SendDataDMA(&data[0], 5, 5);
	for (int i = 0; i < 1000; i++);
	tft_data();
}

void tft_read_display_status() {
	uint8_t data[6] = {0x09, 0,0,0,0,0};
	tft_cmd();
	spi1_SendDataDMA(&data[0], 6, 6);
	for (int i = 0; i < 100; i++);
	tft_data();
}

void tft_display_normal_mode() {
	uint8_t data = 0x13;
	tft_cmd();
	spi1_SendDataDMA(&data, 1, 1);
	for (int i = 0; i < 100; i++);
}


void tft_pixel_format() {
	uint8_t data = 0x05;
	tft_write_cmd(0x3A, &data, 1);
}

void tft_fill_color(uint8_t red, uint8_t green, uint8_t blue) {
	uint8_t data[360];
	data[0] = 0x00;
	tft_write_cmd(0x36, &data[0], 1);

	tft_set_column(20, 200);
	tft_set_row(20, 200);
	tft_ram_write();
	tft_set_color();

	for (int i = 0; i < 180; i += 2) {
		data[i] = 0x22;
		data[i + 1] = 0x44;
	}

	tft_data();
	for (int i = 0; i < 180; i++) {
		spi1_SendDataDMA(&data[0], 360, 360);
		for (int j = 0; j < 500; j++);
	}
//	for (int i = 0; i < 10; i++);

}

void tft_set_column(uint16_t col_start, uint16_t col_end) {
	uint8_t data_column[4] = {	(uint8_t)(col_start >> 8),
								(uint8_t)(col_start & 0xFF),
								(uint8_t)(col_end >> 8),
								(uint8_t)(col_end & 0xFF)};

	tft_write_cmd(0x2A, &data_column[0], 4);
}

void tft_set_row(uint16_t row_start, uint16_t row_end) {
	uint8_t data_row[4] = {	(uint8_t)(row_start >> 8),
								(uint8_t)(row_start & 0xFF),
								(uint8_t)(row_end >> 8),
								(uint8_t)(row_end & 0xFF)};

	tft_write_cmd(0x2B, &data_row[0], 4);
}

void tft_set_color() {

}

void tft_ram_write() {
	uint8_t data_row[4] = {	0x00, 0x00, 0x00, 0x00};

	tft_write_cmd(0x2C, &data_row[0], 4);
}


//отправл€ем команду 2ј, затем координаты начала и конца области по горизонтали
//отправл€ем команду 2B, затем координаты начала и конца области по вертикали
//отправл€ем команду 2—, то есть говорим: У—ейчас будем писать в видеоќ«”Ф
//посылаем кодировку цвета, который хотим вывести в текущей €чейке
//снова посылаем кодировку цвета, при этом координаты сами измен€тс€ по выбранному при инициализации алгоритму


