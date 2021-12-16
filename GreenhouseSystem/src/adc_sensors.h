/*
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 */ 

/**
 * @file 
 * @code #include "adc_sensors.h" @endcode
 *
 * @brief Library for reading from sensors that use Analog to Digital converter
 *
 * Library contains control functions for soil moisture sensor ESES
 *
 * @author Vaclav Cermak, Lungu Masauso, Tereza Berankova
 *
 * @copyright (c) 2019-2021 Vaclav Cermak, Lungu Masauso, Tereza Berankova
 * @{
 */


#ifndef ADC_SENSORS_H_
#define ADC_SENSORS_H_

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions

/* Function prototypes -----------------------------------------------*/
/**
 * @name Functions
 */

/**
 * @brief  Initializes soil sensor on analog pin PC0[A0], sets input channel, interrupt and prescaler
 * @param  admux_register register that selects reference source voltage to ADC and chooses pin to read from
 * @param  adcsra_register register, that manages ADC conversion
 * @return none
 */
void init_soil_sensor(volatile uint8_t *admux_register, volatile uint8_t *adcsra_register);


#endif /* ADC_SENSORS_H_ */