/*
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 */ 

/**
 * @file 
 * @code #include "output_preipherals.h" @endcode
 *
 * @brief Library for output peripheries manipulation, such as LCD display and LEDs.
 *
 * Library contains functions for checking state of 3 main parameters (soil, temp, luminescence).
 * Those functions activate output peripheries, which will try to get those parameters into ideal state.
 * This state is defined with macro constants for each parameter
 *
 * @author Vaclav Cermak, Lungu Masauso, Tereza Berankova
 *
 * @copyright (c) 2019-2021 Vaclav Cermak, Lungu Masauso, Tereza Berankova
 * @{
 */

#ifndef OUTPUT_PERIPHERALS_H_
#define OUTPUT_PERIPHERALS_H_

// Macro constants --------------------------------------------
#define TRESHOLD_LUMINESCENCE_DARK	100		// 100 Lux
#define TRESHOLD_LUMINESCENCE_LIGHT	600		// 600 Lux
#define TRESHOLD_TEMPERATURE_COLD	250		// 25 °C
#define TRESHOLD_TEMPERATURE_HOT	350		// 35 °C
#define TRESHOLD_SOIL_LOW			10		// 10%
#define TRESHOLD_SOIL_HIGH			50		// 50%

/* Includes ----------------------------------------------------------*/
#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>         // C library. Needed for conversion function
#include <string.h>
#include <util/delay.h>
#include "../library/lcd.h"
#include "../library/lcd_definitions.h"
#include "../library/gpio.h"
#include "servo.h"

/* Variables and type definitions -------------------------------------*/

/* Function prototypes -----------------------------------------------*/
/**
 * @name Functions
 */

/**
 * @brief  Fill whitespace on LCD to remove sagging characters from previous display refresh. Unused for optimalization purposes
 * @param  length Number of whitespace characters to be printed
 * @return none
 */
void lcd_fill_whitespace(uint8_t length);

/**
 * @brief  Initialize display, set pointer to beginning of the CGRAM memory and print default values on LCD
 * @return none
 */
void init_lcd();

/**
 * @brief  Updates parameter's values on LCD
 * @param  soil_moisture soil moisture in %
 * @param  temperature temperature in °C, value needs to be 10 times higher because of parsing
 * @return none
 */
void lcd_update_menu(float soil_moisture, int16_t temperature, uint16_t luminescence);

/**
 * @brief  Turns LED on. Checks if it's not on already.
 * @param  reg_name PORT register name, such as PORTB
 * @param  led_pin pin, to which LED is connected
 * @return none
 */
void led_turn_on(volatile uint8_t *reg_name, uint8_t led_pin);

/**
 * @brief  Turns LED off. Checks if it's not off already.
 * @param  reg_name PORT register name, such as PORTB
 * @param  led_pin pin, to which LED is connected
 * @return none
 */
void led_turn_off(volatile uint8_t *reg_name, uint8_t led_pin);

/**
 * @brief  Checks if luminescence is in optimal span. If not, pelmet is closed and artificial lightning is turned on.
 * @param  lumincescence luminescence in Lux
 * @param  light_led LED that simulates artificial lightning
 * @param  led_port_register lightning LEDs port register 
 * @param  servo_pin pin, to which servo is attached
 * @param  servo_port_register servo's port register -> PORTB for example
 * @return none
 */
void light_control_update(uint16_t luminescence, uint8_t light_led, volatile uint8_t *led_port_register, uint8_t servo_pin, volatile uint8_t *servo_port_register);

/**
 * @brief  Initializes output peripheries at start -> open pelmet servo and turn off lights
 * @param  light_led LED that simulates artificial lightning
 * @param  led_port_register lightning LEDs port register 
 * @param  servo_pin pin, to which servo is attached
 * @param  servo_port_register servo's port register -> PORTB for example
 * @return none
 */
void light_control_init(uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register);

/**
 * @brief  Checks if soil moisture is in optimal span. If too high or low, corresponding leds are turned on.
 * @param  soil soil moisture value to be checked
 * @param  led_low pin of the LED, that will be turned on if soil moisture is too low
 * @param  led_low_port_register LEDs PORT register, such as PORTB
 * @param  led_high pin of the LED, that will be turned on if soil moisture is too high
 * @param  led_high_port_register LEDs PORT register, such as PORTB
 * @return none
 */
void soil_control_update(float soil, uint8_t led_low, volatile uint8_t *led_low_port_register, uint8_t led_high, volatile uint8_t *led_high_port_register);

/**
 * @brief  Checks whether temperature is in optimal span. If too high or low, corresponding leds are turned on.
 * @param  temperature temperature value in celsius degrees to be checked (value is 10 times higher than real value)
 * @param  led_low pin of the LED, that will be turned on if temperature is too low
 * @param  led_low_port_register LEDs PORT register, such as PORTB
 * @param  led_high pin of the LED, that will be turned on if temperature is too high
 * @param  led_high_port_register LEDs PORT register, such as PORTB
 * @return none
 */
void temp_control_update(uint16_t temperature, uint8_t led_low, volatile uint8_t *led_low_port_register, uint8_t led_high, volatile uint8_t *led_high_port_register);


#endif /* OUTPUT_PERIPHERALS_H_ */