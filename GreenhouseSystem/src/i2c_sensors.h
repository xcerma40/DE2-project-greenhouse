/*
 * input_sensors.h
 *
 * Created: 14.12.2021 1:32:07
 *  Author: cerma
 */ 


#ifndef INPUT_SENSORS_H_
#define INPUT_SENSORS_H_

#include "../library/twi.h"            // TWI library for AVR-GCC

/* Variables ---------------------------------------------------------*/

typedef enum {             // FSM for luminescence sensor
	STATE_WRITE,
	STATE_READ
} state_bh;


uint8_t read_temperature(volatile uint8_t *temp_flag);

uint8_t read_luminescence(volatile uint8_t *luminescence_flag);

#endif /* INPUT_SENSORS_H_ */