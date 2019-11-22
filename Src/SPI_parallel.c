#include "SPI_parallel.h"

extern SPI_HandleTypeDef hspi1;

#define DISP_RCK_Pin GPIO_PIN_4
#define DISP_RCK_GPIO_Port GPIOC
#define DISP_RCK(value) (HAL_GPIO_WritePin(DISP_RCK_GPIO_Port, DISP_RCK_Pin, value))

#define DISP_EN_Pin GPIO_PIN_5
#define DISP_EN_GPIO_Port GPIOC
#define DISP_EN(value) (HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, value))

#define DISP_SRCLR_Pin GPIO_PIN_4
#define DISP_SRCLR_GPIO_Port GPIOA
#define DISP_SRCLR(value) (HAL_GPIO_WritePin(DISP_SRCLR_GPIO_Port, DISP_SRCLR_Pin, value))



void _delay_100ns()
{
	// Same as in _delay_ns function.
	// 100 / 6 = 16.8;
	for (uint16_t i = 0; i < 17; i++)
	{
		__NOP();
	}
}

void spip_reset(void)
{
	DISP_RCK(0);
	DISP_EN(1);
	DISP_SRCLR(0);
	_delay_100ns();
}

void spip_init(void)
{
	spip_reset();
}



void spip_send_word(uint16_t word2send)
{
	DISP_SRCLR(1);
	DISP_RCK(0);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&word2send +1, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&word2send, 1, 10);
	DISP_RCK(1);
	DISP_EN(0);
	_delay_100ns();
}

/*void spip_send_byte(uint8_t* bytes2send, uint8_t byte_count)
{
	DISP_SRCLR(1);
	DISP_RCK(0);
	HAL_SPI_Transmit(&hspi1, bytes2send, byte_count, 10);
	DISP_RCK(1);
	DISP_EN(0);
	_delay_100ns();
}*/

