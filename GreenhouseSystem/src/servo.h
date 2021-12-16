/*
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 */ 

/**
 * @file 
 * @code #include "servo.h" @endcode
 *
 * @brief Library for servo control
 *
 * Library contains functions for servo rotation (fully left and right)
 *
 * @author Vaclav Cermak, Lungu Masauso, Tereza Berankova
 *
 * @copyright (c) 2019-2021 Vaclav Cermak, Lungu Masauso, Tereza Berankova
 * @{
 */

#ifndef SERVO_H_
#define SERVO_H_

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>
#include <util/delay.h>

/* Function prototypes -----------------------------------------------*/
/**
 * @name Functions
 */

/**
 * @brief  Rotate servo fully to the left.
 * @param  reg_name Address of Port Register, such as &PORTB
 * @param  servo_pin  Pin to which servo is connected
 * @return none
 */
void servo_left(volatile uint8_t *reg_name, uint8_t servo_pin);

/**
 * @brief  Rotate servo fully to the right.
 * @param  reg_name Address of Port Register, such as &PORTB
 * @param  servo_pin  Pin to which servo is connected
 * @return none
 */
void servo_right(volatile uint8_t *reg_name, uint8_t servo_pin);

/**
 * @brief  Initialize servo on designated port
 * @param  reg_name Address of Data Directory Register, such as &DDRB
 * @param  servo_pin  Pin to which servo is connected
 * @return none
 */
void servo_init(volatile uint8_t *reg_name, uint8_t servo_pin);

#endif /* SERVO_H_ */