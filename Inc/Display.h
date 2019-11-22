/*
 * Display.h
 *
 *  Created on: 15. 10. 2019
 *      Author: VladaS
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "main.h"
#include "u8x8.h"

uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_byte_parallel_hw_spi (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_delay_parallel(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);


#endif /* DISPLAY_H_ */
