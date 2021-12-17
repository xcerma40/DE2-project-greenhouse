/*
 * ADC_Sensors library for Greenhouse system.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 * adc_sensors.c
 *
 * Created: 14.12.2021 0:46:24
 * Author: Vaclav Cermak, Lungu Masauso, Tereza Berankova
 */  

/* Includes ----------------------------------------------------------*/
#include "adc_sensors.h"

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: init_soil_sensor()
 * Purpose:  Initializes soil sensor on analog pin PC0[A0], sets input channel, interrupt and prescaler
 * Input:    admux_register - register that selects reference source voltage to ADC and chooses pin to read from
 *           adcsra_register - register, that manages ADC conversion
 * Returns:  none
 **********************************************************************/
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