/*
 * servo.h
 *
 * Created: 14.12.2021 0:39:20
 *  Author: cerma
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