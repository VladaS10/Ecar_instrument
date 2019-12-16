/*
 * can_handler.h
 *
 *  Created on: 14. 12. 2019
 *      Author: VladaS
 */

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include "main.h"


#define CAN_RX_COUNT 10

/*typedef struct
{
	uint16_t can_id;
	uint8_t can_data[8];
}CAN_MSG;*/

uint16_t can_id_table[CAN_RX_COUNT];// = {0x100,0x101,0x101,0x101,0x101,0x101,0x101,0x101,0x101,0x101};

uint8_t can_data_received[CAN_RX_COUNT][8];
//CAN_MSG can_data_received[CAN_RX_COUNT];

uint8_t get_CANID_index(uint16_t);
void copy_RX_message(CAN_RxHeaderTypeDef* can_header, uint8_t* data);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif /* CAN_HANDLER_H_ */
