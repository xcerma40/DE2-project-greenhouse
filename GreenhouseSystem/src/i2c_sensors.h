/*
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 */ 

/**
 * @file 
 * @code #include "i2c_sensors.h" @endcode
 *
 * @brief Library for sesnsors, that use i2c for communication with
 *
 * Library contains control functions for DHT12 (temperature) and GY30 (light sensitivity)
 *
 * @author Vaclav Cermak, Lungu Masauso, Tereza Berankova
 *
 * @copyright (c) 2019-2021 Vaclav Cermak, Lungu Masauso, Tereza Berankova
 * @{
 */

#ifndef INPUT_SENSORS_H_
#define INPUT_SENSORS_H_

/* Includes ----------------------------------------------------------*/

#include "../library/twi.h"            // TWI library for AVR-GCC

/* Function prototypes -----------------------------------------------*/
/**
 * @name Functions
 */

/**
 * @brief  Reads data from DHT12 sensor via i2c bus
 * @param  temp_flg is set if reading was successful
 * @return temperature as signed 16bit value (DHT12 can read from -20°C to +60°C)
 */
int16_t read_temperature(volatile uint8_t *temp_flag);

/**
 * @brief  Reads data from GY30 sensor via i2c bus.
 * @param  luminescence_flag is set if reading was successful
 * @return luminescence as unsigned integer value (typical values 0 - 600 Lx)
 */
uint16_t read_luminescence(volatile uint8_t *luminescence_flag);

/**
 * @brief  Function for conversion between raw data from GY30 to Lux.
 * @param  data is raw data from sensor
 * @return luminescence in Lux 
 */
uint16_t get_lux(uint16_t data);

/**
 * @brief  Function for GY30 (BH1750) initialization. High Resolution mode is set.
 * @return none
 */
void init_bh1750();

#endif /* INPUT_SENSORS_H_ */