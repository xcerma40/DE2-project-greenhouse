/***********************************************************************
 * 
 * Greenhouse system watches over 3 main parameters - soil moisture,
 * air temperature and light intesity. If some of those parameters
 * fall out of thresholds, it indicates which parameters went out
 * and automatically starts to correct them with output peripheries.
 *
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 * 
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU				16000000	// CPU frequency in Hz for UART_BAUD_SELECT
#endif
#define LIGHT_LED			PB5			// This led works as artificial lighting.
#define TEMP_LED_LOW		PB4			// Led for too low temperature indication
#define TEMP_LED_HIGH		PB3			// Led for too high temperature indication
#define SOIL_LED_LOW		PD2			// Led for too low soil moisture indication
#define SOIL_LED_HIGH		PD3			// Led for too high soil moisture indication

#define PELMET_SERVO_PIN	PB2			// Servo for controlling pelmet

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>         // C library. Needed for conversion function

#include "library/timer.h"				// Timer library for AVR-GCC
#include "library/uart.h"				// Peter Fleury's UART library
#include "library/twi.h"				// TWI library for AVR-GCC
#include "library/lcd.h"				// Tomas Fryza's Library for LCD manipulation
#include "library/lcd_definitions.h"	// Definitions for LCD library
#include "library/gpio.h"				// Library for pin input/output settings and manipulation
#include "src/i2c_sensors.h"			// Functions for handling input peripherals connected via i2c (DHT12 and GY30)
#include "src/output_peripherals.h"		// Functions for handling output peripherals as LCD and LEDs
#include "src/servo.h"					// Servo handling
#include "src/adc_sensors.h"			// ADC sensors as soil moisture

/* Global variables for storing all main parameters -------------------*/

float soil_moisture = 0.0;
int16_t temperature = 0;		// value is 10 times higher than real value.
uint16_t luminescence = 0;		// value is 10 times higher than real value.
// flags that indicate, whether value was read from sensor correctly-----
uint8_t temp_flag = 0;	
uint8_t soil_moisture_flag = 0;
uint8_t luminescence_flag = 0;


/* Function declarations ----------------------------------------------*/

/**
 * @brief  Initialize greenhouse system
 * @return none
 */
void green_house_setup();
/**
 * @brief  Initialize leds in Data Direction Registers
 * @return none
 */
void init_leds();
/**
 * @brief  Initialize and enable interrupts
 * @return none
 */
void init_interrupts();

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: main()
 * Purpose:  Main function where the program execution begins.
  *			 Processes parmeters on change (from input peripheries)
			 and controls output peripheries for parameter's correction.
 * Returns:  none
 **********************************************************************/

int main(void)
{
	green_house_setup();
	
	// temporary values for checking if changes were done. Added for performance purposes
	float previous_soil_moisture = 0.0;
	int16_t previous_temperature = UINT16_MAX;
	uint16_t previous_luminescence = UINT16_MAX;
	
    // Infinite loop
    while (1)
    {
		if (soil_moisture_flag){
			if (soil_moisture != previous_soil_moisture){
				previous_soil_moisture = soil_moisture;
				//cli();
				soil_control_update(soil_moisture, SOIL_LED_LOW, &PORTD, SOIL_LED_HIGH, &PORTD);
				//sei();
			}
			soil_moisture_flag = 0;
		}
		if (temp_flag){
			if (temperature != previous_temperature){
				previous_temperature = temperature;
				//cli();
				temp_control_update(temperature, TEMP_LED_LOW, &PORTB, TEMP_LED_HIGH, &PORTB);
				//sei();
			}
			temp_flag = 0;
		}
		if (luminescence_flag){
			if (luminescence != previous_luminescence){
				previous_luminescence = luminescence;
				//cli();
				light_control_update(luminescence, LIGHT_LED, &PORTB, PELMET_SERVO_PIN, &PORTB);
				//sei();

			}
			luminescence_flag = 0;
		}
    }

    return 0;
}

/**********************************************************************
 * Function: green_house_setup()
 * Purpose:  Main setup for whole aplication.
 *			 Initializes all input and output peripheries.
 *			 This function must be called first.
 **********************************************************************/
void green_house_setup(){
	 // Initialize UART to asynchronous, 8N1, 9600 - not used
	 //uart_init(UART_BAUD_SELECT(9600, F_CPU));
	 //uart_puts("GH: init\r\n");
	 
	 // init support libraries and interrupts
	 init_interrupts();
	 twi_init();
	 init_lcd();
	 
	 // init peripheries
	 init_leds();
	 init_bh1750();
	 init_soil_sensor(&ADMUX, &ADCSRA);
	 servo_init(&DDRB, PELMET_SERVO_PIN);
	 // set pelmet servo to default position -> pelmet open
	 servo_left(&PORTB, PELMET_SERVO_PIN);
	 
	 //uart_puts("GH: done\r\n");
	 // Enables interrupts by setting the global interrupt mask
	 sei();
}

/**********************************************************************
 * Function: init_interrupts()
 * Purpose:  Initialization overflow interrupts.
 *			 Initializes 2 interrupt routines - one for lcd and one for
 * 			 reading from sensors (adc,i2c) - and enables them
 **********************************************************************/
void init_interrupts(){
	TIM0_overflow_16ms();
	TIM0_overflow_interrupt_enable();
		 
	TIM1_overflow_262ms();
	TIM1_overflow_interrupt_enable();
}

/**********************************************************************
 * Function: Initialization of all leds used by greenhouse
 * Purpose:  All leds are configured to their pin connection to arduino board.
 *           Data direction registers are set as output.
 **********************************************************************/
void init_leds(){
	GPIO_config_output(&DDRB, LIGHT_LED);
	GPIO_config_output(&DDRB, TEMP_LED_HIGH);
	GPIO_config_output(&DDRB, TEMP_LED_LOW);
	GPIO_config_output(&DDRD, SOIL_LED_HIGH);
	GPIO_config_output(&DDRD, SOIL_LED_LOW);
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Service routine for slow actions as sensors and ADC conversion.
 *           ADC conversion is performed every 262ms and every second
 *           temperature is being read from DHT12 and luminescence from BH1750.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
	static uint8_t iteration = 1;
	uint16_t result_l = 0;
	int16_t  result_t = 0;
	
	if (iteration == 2) {
		//read DHT12
		
		result_l = read_luminescence(&luminescence_flag);
		
		if (luminescence_flag) {
			luminescence = result_l;
		}
		
		result_t = read_temperature(&temp_flag);
		
		if (temp_flag) {
			temperature = result_t;
		}
		
		iteration = 0;
		
		// Start ADC conversion
		ADCSRA |= (1 << ADSC);
	}
	iteration++;
	
}

/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Service routine for fast actions. In this routine LCD display
 *           is refreshed and updated with new data from sensors every 16 ms
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
	lcd_update_menu(soil_moisture, temperature, luminescence);
}

/**********************************************************************
 * Function: ADC interrupt
 * Purpose:  Service routine starts when ADC completes the conversion.
 *           Reads soil moisture and stores it to global variable.
 *           Humidity change flag is set.
 *           High adc value corresponds to dry soil and low value wet soil
 **********************************************************************/
ISR(ADC_vect)
{
	cli();
	uint16_t adc_value = 0;
	adc_value = ADCW;    // Copy ADC result to 16-bit variable
	soil_moisture = 100 - ((float)adc_value/1023.0)*100;  // soil moisture in %
	soil_moisture_flag = 1;
	sei();
}