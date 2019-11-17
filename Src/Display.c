/*
 * Display.c
 *
 *  Created on: 15. 10. 2019
 *      Author: VladaS
 */
#include "Display.h"
extern SPI_HandleTypeDef hspi1;

#define DC_PORT 	GPIOC
#define DC_PIN 		GPIO_PIN_5
//#define RST_PORT 	GPIOC
//#define RST_PIN 	GPIO_PIN_4
#define CS_PORT 	GPIOA
#define CS_PIN 		GPIO_PIN_4

void u8g_hw_port_delay_ns(uint8_t ns)
{
	// Core @72 MHZ: 14ns per instruction.
	// __NOP(); is direct "nop;" instruction to cpu.
	// Divide ns / 28 (extra instruction for jump back to beginning of the loop) for loop cycles.
	for (uint8_t i = 0; i < (ns / 10); i++)
	{
		__NOP();
	}
}

void u8g_hw_port_delay_100ns(uint8_t ns)
{
	// Same as in u8g_hw_port_delay_ns function.
	// 100 / 28 = 3.57;
	for (uint16_t i = 0; i < (ns * 10); i++)
	{
		__NOP();
	}
}

void u8g_hw_port_delay_10us(uint8_t us)
{
	// Same as in u8g_hw_port_delay_ns function.
	// 3.57 * 100 ? 357;
	for (uint16_t i = 0; i < (us * 100); i++)
	{
		__NOP();
	}
}

uint8_t
u8x8_stm32_gpio_and_delay (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
    {
	/*case U8X8_MSG_DELAY_NANO:
		u8g_hw_port_delay_ns(arg_int);
		break;
	case U8X8_MSG_DELAY_100NANO:
		u8g_hw_port_delay_100ns(arg_int);
		break;
	case U8X8_MSG_DELAY_10MICRO:
		u8g_hw_port_delay_10us(arg_int);
		break;*/
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      HAL_Delay (1);
      break;
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay (arg_int);
      break;
    case U8X8_MSG_GPIO_DC:
      HAL_GPIO_WritePin (DC_PORT, DC_PIN, arg_int);
      break;
    case U8X8_MSG_GPIO_RESET:
      break;
    }
  return 1;
}

uint8_t
u8x8_byte_4wire_hw_spi (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
      HAL_SPI_Transmit(&hspi1, (uint8_t *)arg_ptr, arg_int, 10);
      break;
    case U8X8_MSG_BYTE_INIT:
    	//HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      HAL_GPIO_WritePin (DC_PORT, DC_PIN, arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      //HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
      //u8g_hw_port_delay_100ns(10);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      //HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
      //u8g_hw_port_delay_100ns(10);
      break;
    default:
      return 0;
    }
  return 1;
}
