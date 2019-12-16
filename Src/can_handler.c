#include "can_handler.h"
#include "string.h"



uint16_t can_id_table[CAN_RX_COUNT] =
{
  0x000, /* unknown - error*/
  0x105, /* drive mode - PRNDESI */
  0x115, /* speed data */
  0x116, /* torque data */
  0x117, /* general states */
  0x119, /* HV data */
  0x120, /* power data 1 - max power*/
  0x121, /* power data 2 - min max HV */
  0x122, /* torque limits % */
  0x125,  /* pedal position */
  0x506, /* temperature 1*/
  0x514 /* temperature 2*/
};

uint8_t get_CANID_index(uint16_t can_id)
{
	for(uint8_t i = 0; i < CAN_RX_COUNT; i++)
	{
	    if(can_id_table[i] == can_id) return i;
	}
	return 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  CAN_RxHeaderTypeDef can_message_header;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_message_header, can_message);

  memcpy((void*)&can_data_received[get_CANID_index(can_message_header.StdId)][0], (void*)can_message, can_message_header.DLC);

}

