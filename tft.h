/*
 * tft.h
 *
 *  Created on: 31 ���. 2020 �.
 *      Author: ��
 */

/*
 *  PB0 - turn on led
 *  PB1 - data or command
 *  PA4 - nss
 *  pc4 - nss
 *  pc5 - reset for display
 *  pa5 - sck
 *  pa6 - miso
 *  pa7 - mosi
 */

#ifndef TFT_H_
#define TFT_H_

#include <stm32f0xx.h>
#include "spi.h"


void tft_init();
void tft_gpio_init();

void tft_write_cmd(uint8_t cmd, uint8_t *data, uint8_t size);

void tft_sleep_out();	//0x11
void tft_read_display_id();	//0x04 - don't work
void tft_read_display_status();	//0x09
void tft_display_normal_mode();	//0x13
void tft_display_off();	//0x28
void tft_display_on();//0x29
void tft_pixel_format();	//0x3A

void tft_set_region(uint16_t row_start, uint16_t row_end, uint16_t col_start, uint16_t col_end);
void tft_colorise(uint8_t red, uint8_t green, uint8_t blue);
void tft_set_column(uint16_t col_start, uint16_t col_end);	//2A
void tft_set_row(uint16_t row_start, uint16_t row_end);	//2B
void tft_ram_write();	//2c

void tft_clearAllDisplay(uint8_t red, uint8_t green, uint8_t blue);






#endif /* TFT_H_ */
