/*
 * Display.c
 *
 *  Created on: 15. 10. 2019
 *      Author: VladaS
 */
#include "Display.h"
#include "SPI_parallel.h"

extern SPI_HandleTypeDef hspi1;

#define DC_PORT 	GPIOC
#define DC_PIN 		GPIO_PIN_5
//#define RST_PORT 	GPIOC
//#define RST_PIN 	GPIO_PIN_4
#define CS_PORT 	GPIOA
#define CS_PIN 		GPIO_PIN_4

void u8g_hw_port_delay_ns(uint8_t ns)
{
	// Core @168 MHZ: 6ns per instruction.
	// __NOP(); is direct "nop;" instruction to cpu.
	// Divide ns / 28 (extra instruction for jump back to beginning of the loop) for loop cycles.
	for (uint8_t i = 0; i < (ns / 12); i++)
	{
		__NOP();
	}
}

void u8g_hw_port_delay_100ns(uint8_t ns)
{
	// Same as in u8g_hw_port_delay_ns function.
	// 100 / 6 = 16.8;
	for (uint16_t i = 0; i < (ns * 17); i++)
	{
		__NOP();
	}
}

void u8g_hw_port_delay_10us(uint8_t us)
{
	// Same as in u8g_hw_port_delay_ns function.
	// 16.8 * 100 = 1680;
	for (uint16_t i = 0; i < (us * 1680); i++)
	{
		__NOP();
	}
}

uint8_t
u8x8_stm32_gpio_and_delay (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
    {
	case U8X8_MSG_DELAY_NANO:
		u8g_hw_port_delay_ns(arg_int);
		break;
	case U8X8_MSG_DELAY_100NANO:
		u8g_hw_port_delay_100ns(arg_int);
		break;
	case U8X8_MSG_DELAY_10MICRO:
		u8g_hw_port_delay_10us(arg_int);
		break;
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
/*-----------------------------------------------------------------------------------*/
/*------------------------------- Parallel Display ----------------------------------*/
/*-----------------------------------------------------------------------------------*/
//static uint16_t send_buffer = 0;
static uint8_t Rst = 1,CS1 = 0, CS2= 0, Enab = 0, /*RW = 0, UNUSED */ DC = 0;

uint16_t make_word(uint8_t databyte)
{
	uint16_t new_word = (uint16_t)databyte << 3;
	if(Rst) new_word |= (1<<13);
	if(CS1) new_word |= (1<<11);
	if(CS2) new_word |= (1<<12);
	if(Enab) new_word |= (1<<2);
	//if(RW) new_word |= 2; //UNUSED - write only
	if(DC) new_word |= 1;
	return new_word;
}


uint8_t
u8x8_byte_parallel_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	uint8_t* pom;
  switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
    	pom = (uint8_t *)arg_ptr;
    	for(uint8_t i = 0; i < arg_int; i++)
    	{
    		spip_send_word(make_word(pom[i]));            Enab = 1;            spip_send_word(make_word(pom[i]));            Enab = 0;            spip_send_word(make_word(pom[i]));
    	}
      break;
    case U8X8_MSG_BYTE_INIT:
    	//HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
      break;
    case U8X8_MSG_BYTE_SET_DC:
    	DC = arg_int;
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    	if(arg_int == 1) CS1 = 1;
    	else if(arg_int == 2) CS2 = 1;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
    	 CS1 = 0;
    	 CS2 = 0;         spip_send_word(make_word(0x00));
      break;
    default:
      return 0;
    }
  return 1;
}

uint8_t u8x8_gpio_delay_parallel(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
	case U8X8_MSG_DELAY_NANO:
		u8g_hw_port_delay_ns(arg_int);
		break;
	case U8X8_MSG_DELAY_100NANO:
		u8g_hw_port_delay_100ns(arg_int);
		break;
	case U8X8_MSG_DELAY_10MICRO:
		u8g_hw_port_delay_10us(arg_int);
		break;
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		HAL_Delay (1);
    	break;
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay (arg_int);
		break;
//    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
//      break;
//    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
//    	Enab = arg_int;
//      break;
//    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
//      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
    	DC = arg_int;
      break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
    	Rst = arg_int;        //spip_send_word(make_word(0x00));
      break;
//    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
//    	CS1 = arg_int;
//      break;
//    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
//    	CS2 = arg_int;
//      break;
  }
  return 1;
}
