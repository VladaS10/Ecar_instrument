/*
 * can_handler.h
 *
 *  Created on: 14. 12. 2019
 *      Author: VladaS
 */

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include "main.h"


#define CAN_RX_COUNT 12

uint8_t can_data_received[CAN_RX_COUNT][8];

uint8_t get_CANID_index(uint16_t);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif /* CAN_HANDLER_H_ */
