/*
 * adc_sensors.h
 *
 * Created: 14.12.2021 12:22:57
 *  Author: cerma
 */ 


#ifndef ADC_SENSORS_H_
#define ADC_SENSORS_H_

#include <avr/io.h>         // AVR device-specific IO definitions

void initSoilSensor(volatile uint8_t *admux_register, volatile uint8_t *adcsra_register);


#endif /* ADC_SENSORS_H_ */