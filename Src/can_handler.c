#include "can_handler.h"
#include "string.h"

uint8_t get_byte_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos)
{
  return (can_data_received[get_CANID_index(CAN_ID)][byte_pos]);
}

uint16_t get_word_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos)
{
   uint8_t* b_point = &can_data_received[get_CANID_index(CAN_ID)][byte_pos];
   return (b_point[1] + (((uint16_t)b_point[0]) << 8));
}


int16_t convert_CAN_temp(uint8_t temp_byte)
{
  return ((int16_t)temp_byte - 40);
}

uint16_t convert_CAN_percent(uint8_t percent_byte)
{
  return ((uint16_t)percent_byte * 4);
}

uint16_t convert_CAN_power(uint16_t power_word)
{
  return (power_word * 10) / 4;
}

uint16_t convert_CAN_HV(uint16_t power_word)
{
  return (power_word * 10) / 8;
}

uint16_t can_id_table[CAN_RX_COUNT] =
{
  0x000, /* unknown - error*/
  0x108, /* drive mode - PRNDESI */
  0x115, /* speed data */
  0x116, /* torque data */
  0x117, /* general states */
  0x119, /* HV data */
  0x120, /* power data 1 - max power*/
  0x121, /* power data 2 - min max HV */
  0x122, /* torque limits % */
  0x125,  /* accelerator position */
  0x506, /* temperature 1*/
  0x514, /* temperature 2*/
  0x600, /* Battery capacity */ /*TODO*/
  0x601, /* Battery temperature */
  0x602  /* Battery power available */
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

