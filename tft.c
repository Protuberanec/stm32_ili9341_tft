/*
 * tft.c
 *
 *  Created on: 31 окт. 2020 г.
 *      Author: еу
 */

#include "tft.h"

/*
 * для вывода букв, их необходимо сконвертировать в байтовую маску... и записать во флэш...
 * при этом маска даже может быть битовой, и тогда необходимо будет каждый бит умножить на цвет символа и цвет фона
 * даже размер 8*8 пикселей это 64 слова (128 байт), 33 буквы русского алфавита, 10 цифр, около 9 спец символов
 * итого получается 6784 байт... (без учёта заглавных букв, и букв английского алфавита)
 * очень хорошо помещается во flash м даже в рам помещается... но...
 * можно также алфавиты и прочее хранить в SD карте
 *
 * вывод символов может быть....
 * MEMORY (SPI) -> DMA -> Memory uC -> DMA -> TFT (SPI)
 * при этом необходимо использовать глобальные переменные....
 *
 * также можно сделать и вывод картинки... необходимо смотреть datashIt о прерывании передачи картинки,
 * либо выпендриваться с адресацией...
 *
 * очень удобно использовать ОСРВ
 *
 */


void tft_gpio_init() {
	RCC->AHBENR |= LED_RCC | DC_RS_RCC | RESET_RCC;

	LED_PORT->MODER |= LED_PIN_MODER;	//LED
	DC_RS_PORT->MODER |= DC_RS_PIN_MODER;	//DC/RS
	RESET_PORT->MODER |= RESET_PIN_MODER;	//reset
}

void tft_data() {
	DC_RS_PORT->BSRR = 1 << DC_RS_PIN;	//GPIO_BSRR_BS_x
}

void tft_cmd() {
	DC_RS_PORT->BSRR = 1 << (DC_RS_PIN + 16);	//GPIO_BSRR_BR_x
}

void tft_led_on() {
	LED_PORT->BSRR = 1 << LED_PIN;
}

void tft_led_off() {
	LED_PORT->BSRR = 1 << (LED_PIN + 16);
}

void tft_reset() {
	RESET_PORT->BSRR = 1 << RESET_PIN;
}

void tft_init() {
	tft_gpio_init();

	spi1_master_init(8);
	tft_led_on();
	for (int i = 0; i < 100; i++);

	tft_reset();
	for (int i = 0; i < 20000; i++);

	tft_sleep_out();

	tft_display_normal_mode();
	tft_display_on();
	tft_pixel_format();

	tft_clearAllDisplay(0x0F, 0x41, 0x06);	//purple
}

void tft_write_cmd(uint8_t cmd, uint8_t *data, uint8_t size) {
	tft_cmd();
	spi1_SendGetDataDMA_1Byte(&cmd, 1);
	for (int i = 0; i < 10; i++);	//тупая задержка, но без неё надо будет городить какую-то очередь... пускай тупит 10 тактов

	if (size == 0) {
		return;
	}

	tft_data();
	spi1_SendGetDataDMA_1Byte(&data[0], size);

	for (int i = 0; i < 10; i++);
}

void tft_sleep_out() {
	tft_cmd();
	uint8_t data = 0x11;
	spi1_SendGetDataDMA_1Byte(&data, 1);
	for (int i = 0; i < 100000; i++);
}

void tft_display_off() {
	tft_write_cmd(0x28, 0, 0);
}

void tft_display_on() {
	tft_write_cmd(0x29, 0, 0);
}

//it's not work always return 0x00
//better to use 0x09,0x0A,0x0B,0x0C 0x0C
void tft_read_display_id() {
	uint8_t data[5] = {0x04, 0x00, 0x00, 0x00, 0x00};
	tft_cmd();
	spi1_SendGetDataDMA_1Byte(&data[0], 5);
	for (int i = 0; i < 1000; i++);
	tft_data();
}

void tft_read_display_status() {
	uint8_t data[6] = {0x09, 0,0,0,0,0};
	tft_cmd();
	spi1_SendGetDataDMA_1Byte(&data[0], 6);
	for (int i = 0; i < 100; i++);
	tft_data();
}

void tft_display_normal_mode() {
	uint8_t data = 0x13;
	tft_cmd();
	spi1_SendGetDataDMA_1Byte(&data, 1);
	for (int i = 0; i < 100; i++);
}

void tft_pixel_format() {
	uint8_t data = 0x55;	//pixel forma is 16 bit...
	tft_write_cmd(0x3A, &data, 1);
}

#ifdef DEBUG
void tft_fill_color(uint8_t red, uint8_t green, uint8_t blue) {
	uint8_t data[360];
	data[0] = 0x00;
	tft_write_cmd(0x36, &data[0], 1);

	tft_set_column(20, 200);
	tft_set_row(20, 200);
	tft_ram_write();

	static uint16_t total_color = 0;	//(blue & 0x1F) | ((green & 0x3F) << 5) | ((red & 0x1F) << 11);
	total_color += 1;

//	uint16_t temp_total_color = (total_color << 11) & 0xF800;	// blue
//	uint16_t temp_total_color = total_color & 0x001F;			//red
	uint16_t temp_total_color = (total_color << 5) & 0x07E0;	//green

	usart1_SlowSend1Byte((temp_total_color >> 8) & 0xFF);
	usart1_SlowSend1Byte(temp_total_color & 0xFF);

	for (int i = 0; i < 180; i += 2) {
		data[i] = (temp_total_color & 0xFF00) >> 8;
		data[i + 1] = (temp_total_color & 0xFF);
	}

	tft_data();
	for (int i = 0; i < 180; i++) {
		spi1_SendGetDataDMA_1Byte(&data[0], 100);
		for (int j = 0; j < 500; j++);
	}
//	for (int i = 0; i < 10; i++);

}
#endif

void tft_set_column(uint16_t col_start, uint16_t col_end) {
	uint8_t data_column[4] = {	(uint8_t)((col_start >> 8) & 0xFF),
								(uint8_t)(col_start & 0xFF),
								(uint8_t)((col_end >> 8) & 0xFF),
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

void tft_ram_write() {
	uint8_t data_row[4] = {	0x00, 0x00, 0x00, 0x00};
	tft_write_cmd(0x2C, &data_row[0], 4);
}

static uint16_t count_pixels;	//later put to the struct of Display...
void tft_set_region(uint16_t row_start, uint16_t row_end, uint16_t col_start, uint16_t col_end) {
	count_pixels = (row_end - row_start) * (col_end - col_start);
	uint8_t data = 0x00;
	tft_write_cmd(0x36, &data, 1);

	tft_set_column(col_start, col_end);
	tft_set_row(row_start, row_end);
	tft_ram_write();
}

void tft_colorise(uint8_t red, uint8_t green, uint8_t blue) {
	tft_data();

	uint16_t total_color = ((blue << 11) & 0xF800) | ((green << 5) & 0x07E0) | (red & 0x001F);
	total_color = (total_color << 8) | (total_color >> 8);

	//необходимо помнить!!!, чтобы переменная count_pixels была меньше 65536, потому что это максимальный размер DMA
	spi1_SendDataDMA_2byteNTimes(total_color, count_pixels);
}

void tft_clearAllDisplay(uint8_t red, uint8_t green, uint8_t blue) {
	uint16_t row_start = 0;
	uint16_t row_end = 0x10;

	for (;;) {
		tft_set_region(row_start, row_end, 0, 240);
		tft_colorise(red, green, blue);
		row_start += 0x10;
		row_end += 0x10;

		if (row_start > 320) {
			row_end = 0x10;
			row_start = 0;
			break;
		}
		for (int i = 0; i < 10000; i++);	//дебильные задержки...., в основном теле программы это будет приводить к тормозам
											//надо делать передачу 32 разрядных данных, это позволит не заморачиваться с ограниченными
											//возможностями DMA по числу передачи данных до 65535 данных...
	}
}



