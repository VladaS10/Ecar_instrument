/*
 * StepperMot.c
 *
 *  Created on: 2. 10. 2019
 *      Author: VladaS
 */
#include "StepperMot.h"

/* enables stepper motor coils according to actual position */
static void StepperMot_setOut(s_STEPPER* stepper)
{
	/* last step */
	if (stepper->step == 7)
	{
		stepper->pin_state = 0x09;//0b 0000 1001
	}
	/* even step */
	else if (stepper->step % 2)
	{
		stepper->pin_state = 1<<(stepper->step/2);
	}
	/* odd step */
	else
	{
		stepper->pin_state = 3<<(stepper->step/2);
	}

	/* SET GPIO */
	for (int i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(stepper->port[i], stepper->pin[i], (stepper->pin_state>>i) & 0x01);
	}
}

void StepperMot_init(s_STEPPER* stepper, uint16_t init_steps)
{
	StepperMot_setOut(stepper);
	stepper->position = init_steps;
	stepper->wPosition = 0;
}

void StepperMot_reset(s_STEPPER* stepper)
{
	stepper->wPosition = 0;
}

/* makes one step closer to wanted position */
void StepperMot_step(s_STEPPER* stepper)
{
	//wanted position
	if(stepper->wPosition == stepper->position ||
			stepper->direction == STOP)
	{
		stepper->at_position = 1;
		return;
	}
	//forward
	else if(stepper->wPosition > stepper->position)
	{
		stepper->at_position = 0;
		stepper->position++;
		if(stepper->direction == FORWARD)
			stepper->step = stepper->step < 7 ? stepper->step+1 : 0;
		else
			stepper->step = stepper->step > 0 ? stepper->step-1 : 7;
	}
	//backward
	else
	{
		stepper->at_position = 0;
		stepper->position--;
		if(stepper->direction == FORWARD)
			stepper->step = stepper->step > 0 ? stepper->step-1 : 7;
		else
			stepper->step = stepper->step < 7 ? stepper->step+1 : 0;
	}

	StepperMot_setOut(stepper);
}
