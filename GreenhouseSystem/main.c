/***********************************************************************
 * 
 * The I2C bus scanner detects the addresses of the modules that are 
 * connected to the SDA and SCL signals. A simple description of FSM is 
 * used.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak, Lungu Masauso, Terezie Berankova
 * 
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU				16000000	// CPU frequency in Hz for UART_BAUD_SELECT
#endif
#define LIGHT_LED		PB5			// Green
#define TEMP_LED_LOW		PB4			// Output pin for controlling water pump_
#define TEMP_LED_HIGH		PB3
#define SOIL_LED_LOW		PB2
#define SOIL_LED_HIGH		PD2

#define PELMET_SERVO_PIN	PD3
#define wet_soil			400			// Define max value to consider soil 'wet'
#define dry_soil			850			// Minimum value for dry soil

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
#include "library/gpio.h"				// Library for pin input/output manipulation
#include "src/i2c_sensors.h"			// Functions for handling output peripherals as LCD and LEDs
#include "src/output_peripherals.h"		// 
#include "src/servo.h"					// Servo handling
#include "src/adc_sensors.h"			// ADC sensors as soil moisture

/* Variables ---------------------------------------------------------*/

float soil_moisture = 0.0;
uint16_t temperature = 0;
uint16_t luminescence = 0;
// flags ---------------------------------------------------------------
uint8_t temp_flag = 0;
uint8_t soil_moisture_flag = 0;
uint8_t luminescence_flag = 0;


/* Function declarations ----------------------------------------------*/

void green_house_setup();
void updateLED(uint16_t intensity, uint8_t treshold, uint8_t led);
void init_leds();
void init_interrupts();
//void initLCD();
//void lcd_updateMenu();

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Processes input on change and controls output peripheries.
 * Returns:  none
 **********************************************************************/

int main(void)
{
	green_house_setup();
	//uart_puts("GH: Init\r\n");
	
	float previous_soil_moisture = 0.0;
	uint16_t previous_temperature = UINT16_MAX;
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

    // Will never reach this
    return 0;
}



void green_house_setup(){
	 init_interrupts();
	 twi_init();
	 init_bh1750();
	 init_lcd();
	 
	 // Initialize UART to asynchronous, 8N1, 9600
	 uart_init(UART_BAUD_SELECT(9600, F_CPU));
	 uart_puts("test");
	 
	 init_leds();
	 init_soil_sensor(&ADMUX, &ADCSRA);
	 
	 servo_init(&DDRB, PELMET_SERVO_PIN);
	 servo_left(&PORTB, PELMET_SERVO_PIN);
	 
	 // Enables interrupts by setting the global interrupt mask
	 sei();
}

void init_interrupts(){
	TIM0_overflow_16ms();
	TIM0_overflow_interrupt_enable();
		 
	TIM1_overflow_262ms();
	TIM1_overflow_interrupt_enable();
}





void init_leds(){
	GPIO_config_output(&DDRB, LIGHT_LED);
	GPIO_config_output(&DDRB, TEMP_LED_HIGH);
	GPIO_config_output(&DDRB, TEMP_LED_LOW);
	GPIO_config_output(&DDRD, SOIL_LED_HIGH);
	GPIO_config_output(&DDRD, SOIL_LED_LOW);
		
	/*GPIO_write_high(&PORTB, LIGHT_LED);
	GPIO_write_high(&PORTB, TEMP_LED_HIGH);
	GPIO_write_high(&PORTB, TEMP_LED_LOW);
	GPIO_write_high(&PORTD, SOIL_LED_HIGH);
	GPIO_write_high(&PORTD, SOIL_LED_LOW);*/
}

void updateLED(uint16_t intensity, uint8_t treshold, uint8_t led){

	if (intensity < treshold){ // luminescence < 10.0
		PORTC = PORTC | (1<<led);
	}
	else {
		PORTC = PORTC & ~(1<<led);
	}
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
	uint16_t result = 0;
	
	//cli();
	if (iteration == 2) {
		//read DHT12
		
		result = read_luminescence(&luminescence_flag);
		
		if (luminescence_flag) {
			luminescence = result;
			//light_control_update(luminescence, LIGHT_LED, &PORTB, PELMET_SERVO_PIN, &PORTB);
		}
		
		result = read_temperature(&temp_flag);
		
		if (temp_flag) {
			temperature = result;
			//temp_control_update(temperature, TEMP_LED_LOW, &PORTB, TEMP_LED_HIGH, &PORTB);
		}
		
		iteration = 0;
		
		// Start ADC conversion
		ADCSRA |= (1 << ADSC);
	}
	iteration++;
	
	//sei();
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
	//soil_control_update(soil_moisture, SOIL_LED_LOW, &PORTD, SOIL_LED_HIGH, &PORTD);
	sei();
}
