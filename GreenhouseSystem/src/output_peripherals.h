/*
 * output_peripherals.h
 *
 * Created: 14.12.2021 12:36:11
 *  Author: cerma
 */ 


#ifndef OUTPUT_PERIPHERALS_H_
#define OUTPUT_PERIPHERALS_H_

#define TRESHOLD_LUMINESCENCE_DARK	30
#define TRESHOLD_LUMINESCENCE_LIGHT	80

#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>         // C library. Needed for conversion function
#include <string.h>
#include <util/delay.h>
#include "../library/lcd.h"
#include "../library/lcd_definitions.h"
#include "../library/gpio.h"
#include "servo.h"

typedef enum {             // FSM for luminescence sensor
	LC_STATE_DL,
	LC_STATE_OPTIMAL,
} state_lc;

void lcd_fill_whitespace(uint8_t length);
void init_lcd();
void lcd_update_menu(float soil_moisture, uint16_t temperature, uint16_t luminescence);
void led_turn_on(volatile uint8_t *reg_name, uint8_t led_pin);
void led_turn_off(volatile uint8_t *reg_name, uint8_t led_pin);
void light_control_update(uint16_t luminescence, uint8_t light_led, volatile uint8_t *led_port_register, uint8_t servo_pin, volatile uint8_t *servo_port_register);
void light_control_init(uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register);


#endif /* OUTPUT_PERIPHERALS_H_ */