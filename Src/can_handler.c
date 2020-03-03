#include "can_handler.h"
#include "string.h"

uint8_t get_byte_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos)
{
  return (can_data_received[get_CANID_index(CAN_ID)][byte_pos]);
}

int16_t get_word_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos)
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

int16_t convert_CAN_power(int16_t power_word)
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
  0x301, /* Battery U, I, CH[0.1%], T[C] */
  0x302, /* Battery U min, U max, Discharge max, Regen max. */
  0x303  /* Battery binary values
	   * B0:
		b0 – charge enable – povoleno nabíjení/rekuperace
		b1 – discharge enable – povoleno vybíjení
		b2 – charge immediately – je třeba nabíjet
		b3 – charging finished – nabíjení ukončeno
	   * B1:
		b0 – temperature too high – příliš vysoká teplota – snižuje se vybíjecí/nabíjecí proud
		b1 – temperature too low – příliš nízká teplota - snižuje se vybíjecí/nabíjecí proud
		b3 – over temperature – přehřání – dojde k odpojení baterie
		b4 – under tepmerature – příliš nízká teplota – omezení nabíjení baterie
		b5 – insulation test failure – selhání izolace
		b6 – precharge failure – selhání precharge (při pokusu o připojení proud neklesl pod stanovenou hodnotu za stanovený čas)  */

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

