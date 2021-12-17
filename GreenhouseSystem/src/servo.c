/*
 * Servo library for Greenhouse system.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 * servo.c
 *
 * Created: 14.12.2021
 * Author: Vaclav Cermak, Lungu Masauso, Tereza Berankova
 */ 

/* Includes ----------------------------------------------------------*/
#include "servo.h"
#include "../library/gpio.h"

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: servo_left()
 * Purpose:  Rotate servo fully to the left by sending input signal of length 20ms
 *           in ratio 0,7ms (high) : 19,3ms (low)
 * Input:    reg_name - Address of servos Port register, such as &PORTB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_left(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_write_high(&PORTB, servo_pin);
	_delay_us(700);
	GPIO_write_low(&PORTB, servo_pin);
	_delay_us(300);
	_delay_ms(19);
};

/**********************************************************************
 * Function: servo_right()
 * Purpose:  Rotate servo fully to the right by sending input signal of length 20ms
 *           in ratio 2,4ms (high) : 17,6ms (low)
 * Input:    reg_name - Address of servos Port register, such as &PORTB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_right(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_write_high(&PORTB, servo_pin);
	_delay_us(2400);
	GPIO_write_low(&PORTB, servo_pin);
	_delay_us(600);
	_delay_ms(17);
};

/**********************************************************************
 * Function: servo_init()
 * Purpose:  Initialize servo in Data Direction Register
 * Input:    reg_name - Address of Data Direction Register, such as &DDRB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_init(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_config_output(reg_name, servo_pin);	
}