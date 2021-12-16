/*
 * adc_sensors.c
 *
 * Created: 14.12.2021 12:23:41
 *  Author: cerma
 */ 

#include "adc_sensors.h"

void init_soil_sensor(volatile uint8_t *admux_register, volatile uint8_t *adcsra_register){
	// Configure ADC to convert PC0[A0] analog value
	// Set ADC reference to AVcc
	*admux_register |= (1 << REFS0);
	*admux_register &= ~(1 << REFS1);
	// Set input channel to ADC0
	*admux_register &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
	// Enable ADC module
	*adcsra_register |= (1 << ADEN);
	// Enable conversion complete interrupt
	*adcsra_register |= (1 << ADIE);
	// Set clock prescaler to 128
	*adcsra_register |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}