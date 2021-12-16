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

uint8_t read_temperature(volatile uint8_t *temp_flag);

uint16_t read_luminescence(volatile uint8_t *luminescence_flag);

uint16_t get_lux(uint16_t data);

void init_bh1750();

#endif /* INPUT_SENSORS_H_ */