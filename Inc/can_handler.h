/*
 * can_handler.h
 *
 *  Created on: 14. 12. 2019
 *      Author: VladaS
 */

/*o57_DI_temperature1
● Default CAN ID: 0x506
● Default frequency: 1 Hz
● Length: 8 bytes
○ DBC signal representation:
■ SG_ DI_pcbT : 0|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_inverterT : 8|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_statorT : 16|8@1+ (1,-40) [-40|200] "DegC" X
■ SG_ DI_dcCapT : 24|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_heatsinkT : 32|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_inletT : 40|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_inverterTpct : 48|8@1+ (0.4,0) [0|100] "%" X
■ SG_ DI_statorTpct : 56|8@1+ (0.4,0) [0|100] "%" X
○ Mostly unsigned 8-bit values in degrees C with a -40 offset */

/*o57_DI_temperature2
● Default CAN ID: 0x514
● Default frequency: 1 Hz
● Length: 5 bytes
○ DBC signal representation:
■ SG_ DI_ph1Temp : 0|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_ph2Temp : 8|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_ph3Temp : 16|8@1+ (1,-40) [-40|120] "DegC" X
■ SG_ DI_powerPcbT : 24|8@1+ (1,-40) [-40|150] "DegC" X
■ SG_ DI_IGBTJunctTemp : 32|8@1+ (1,-40) [-40|200] "DegC" X
○ Unsigned 8-bit values in degrees C with a -40 offset*/

/*o57_speedData
● Default CAN ID: 0x115
● Default frequency: 100 Hz
● Length: 6 bytes
● Data:
○ Motor RPM
■ 16-bit signed integer in data[0] and data[1]
○ Calculated Vehicle Speed * 10.0 (MPH)
■ 16-bit signed in data[2] and data[3]
○ Tesla firmware provided Vehicle Speed * 10.0
■ 16-bit signed in data[4] and data[5]*/

/*o57_torquePowerData
● Default CAN ID: 0x116
● Default frequency: 100 Hz
● Length: 4 bytes
● Data:
○ Output Torque * 4.0 (Nm)
■ 16-bit signed integer in data[0] and data[1]
○ Output Mechanical Power * 4.0 (kW)
■ 16-bit signed integer in data[2] and data[3]*/

/*o57_generalStates
● Default CAN ID: 0x117
● Default frequency: 100 Hz
● Length: 8 bytes
● Data:
○ Raw Input States
■ data[0] = Bit flags for the value of each of the six 12V general inputs
● Only enabled inputs can show “1”’s
○ Raw Output States
■ data[1] = Bit flags for the value of each of the Relay Control Outputs
● Only enabled outputs can show “1”s
● 1 = Relay enabled (Ground tied)
○ data[2] = Brake Light Status
○ data[3] = Reverse Light Status
○ data[4] = Regenerative braking over brake light torque threshold
○ data[5] = Brake pedal pressed
○ data[6] = Torque creep enabled
○ Data[7] = Current accepted gear
■ 1 = DRIVE, 2 = REVERSE. 3 = NEUTRAL*/

/*o57_HVLVdata
● Default CAN ID: 0x119
● Default frequency: 100 Hz
● Length: 5 bytes
● Data:
○ High voltage input voltage * 8.0 (Volts)
■ 16-bit unsigned integer in data[0] and data[1]
○ High voltage current estimate * 8.0 (Amps)
■ 16-bit signed integer in data[2] and data[3]
○ 12V input voltage * 8.0 (Volts)
■ 8-bit unsigned integer in data[4]*/

/*o57_powerData
● Default CAN ID: 0x120
● Default frequency: 1 Hz
● Length: 8 bytes
● Data:
○ Max HV charge/regen current * 10.0 (Amps)
■ 16-bit unsigned integer in data[0] and data[1]
○ Max HV discharge current * 10.0 (Amps)
■ 16-bit unsigned integer in data[2] and data[3]
○ Max HV charge/regen power * 10.0 (kW)
■ 16-bit unsigned integer in data[4] and data[5]
○ Max HV discharge power * 10.0 (kW)
■ 16-bit unsigned integer in data[6] and data[7]*/

/*o57_powerData2
● Default CAN ID: 0x121
● Default frequency: 1 Hz
● Length: 4 bytes
● Data:
○ Minimum HV bus voltage * 10.0 (Volts)
■ 16-bit unsigned integer in data[0] and data[1]
○ Maximum HV bus voltage * 10.0 (Volts)
■ 16-bit unsigned integer in data[2] and data[3]*/

/*o57_torqueLimits
● Default CAN ID: 0x122
● Default frequency: 10 Hz
● Length: 3 bytes
● Data:
○ Data[0] = Regen Torque Percent / 0.4 (250 = 100%)
○ Data[1] = Output Torque Percent / 0.4 (250 = 100%)
○ Data[2] = 057 Crude Traction Control Enabled*/

/*o57_pedalPos
● Default CAN ID: 0x125
● Default frequency: 10 Hz
● Length: 3 bytes
● Data:
○ Data[0] = Pedal Position Percent [filtered] / 0.4 (250 = 100%)
○ Data[1] = Pedal Position Track A Percent [unfiltered] / 0.4 (250 = 100%)
○ Data[2] = Pedal Position Track B Percent [unfiltered]*/

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include "main.h"


#define CAN_RX_COUNT 15

/*All values in format 1234.5 */
#define DRIVE_MODE (get_byte_CAN_RX(0x108,0))

#define BRAKE_PEDAL_PRESSED (get_byte_CAN_RX(0x117,5))
#define MOTOR_TEMP (convert_CAN_temp(get_byte_CAN_RX(0x506,2)))
#define INVERT_TEMP (convert_CAN_temp(get_byte_CAN_RX(0x506,1)))

#define MOTOR_POWER (convert_CAN_power(get_word_CAN_RX(0x116,2)))
#define MOTOR_TORQ (convert_CAN_power(get_word_CAN_RX(0x116,0)))

#define VEHICLE_SPEED (get_word_CAN_RX(0x115,2))

#define HV_BAT_VOLT (convert_CAN_HV(get_word_CAN_RX(0x119,0)))
#define HV_BAT_CURR (convert_CAN_HV(get_word_CAN_RX(0x119,2)))
#define LV_BAT_VOLT (convert_CAN_HV(get_byte_CAN_RX(0x119,4)))

#define MAX_REGEN_POW (get_word_CAN_RX(0x120,4))
#define MAX_DISCH_POW (get_word_CAN_RX(0x120,6))

#define MAX_HV_VOLT (get_word_CAN_RX(0x121,2))
#define MIN_HV_VOLT (get_word_CAN_RX(0x121,0))

#define PEDAL_POS (convert_CAN_percent(get_byte_CAN_RX(0x125,1)))


uint8_t can_data_received[CAN_RX_COUNT][8];


uint8_t get_byte_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos);
uint16_t get_word_CAN_RX(uint16_t CAN_ID, uint8_t byte_pos);
int16_t convert_CAN_temp(uint8_t temp_byte);
uint16_t convert_CAN_percent(uint8_t percent_byte);
uint16_t convert_CAN_power(uint16_t power_word);
uint16_t convert_CAN_HV(uint16_t power_word);

uint8_t get_CANID_index(uint16_t);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif /* CAN_HANDLER_H_ */
