/*
 * StepperMot.h
 *
 *  Created on: 2. 10. 2019
 *      Author: VladaS
 */
#ifndef STEPPERMOT_H_
#define STEPPERMOT_H_

#include "main.h"

typedef enum
{
	FORWARD,
	BACKWARD,
	STOP
}e_DIRECTION;

typedef struct
{
	uint16_t position;
	uint16_t wPosition;
	uint16_t max_position;
	uint8_t at_position;
	uint8_t step;
	uint8_t microstep;
	int16_t speed;
	e_DIRECTION direction;
	uint16_t pin_state;
	GPIO_TypeDef* port[4];
	uint16_t pin[4];
}s_STEPPER;

void StepperMot_init(s_STEPPER* stepper, uint16_t init_steps);
void StepperMot_reset(s_STEPPER*);
void StepperMot_step(s_STEPPER*);


#endif /* STEPPERMOT_H_ */
