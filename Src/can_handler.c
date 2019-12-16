#include "can_handler.h"
#include "string.h"


uint8_t get_CANID_index(uint16_t can_id)
{
	for(uint8_t i = 0; i < CAN_RX_COUNT; i++)
	{
		if(can_id_table[i] == can_id) return i;
	}
	return 0;
}

void copy_RX_message(CAN_RxHeaderTypeDef* can_header, uint8_t* data)
{
	memcpy((void*)data, (void*)&can_data_received[get_CANID_index(can_header->StdId)][0], can_header->DLC);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

}

