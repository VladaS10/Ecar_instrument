/*
 * SPI_parallel.h
 *
 *  Created on: 20. 11. 2019
 *      Author: VladaS
 */

#ifndef SPI_PARALLEL_H_
#define SPI_PARALLEL_H_

#include "main.h"

void spip_init(void);
void spip_send_word(uint16_t word2send);
void spip_send_byte(uint8_t* bytes2send, uint8_t byte_count);


#endif /* SPI_PARALLEL_H_ */
