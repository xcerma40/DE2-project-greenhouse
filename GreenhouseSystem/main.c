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
#define F_CPU			16000000	// CPU frequency in Hz required for UART_BAUD_SELECT
#endif
#define LIGHT_LED		PB4			//Green
#define PUMP_LED		PC2			// Output pin for controlling water pump
#define TEMPERATURE_LED	PC3
#define SERVO_PIN		PB2
#define wet_soil		400			// Define max value to consider soil 'wet'
#define dry_soil		850			// Minimum value for dry soil

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
#include "src/servo.h"					// Servo handling
#include "src/i2c_sensors.h"			// 

/* Variables ---------------------------------------------------------*/

uint16_t humidity = 0;
uint16_t temperature = 0;
uint16_t luminescence = 0;
// flags ---------------------------------------------------------------
uint8_t temp_flag = 0;
uint8_t humid_flag = 0;
uint8_t luminescence_flag = 0;


/* Function declarations ----------------------------------------------*/

void initialSetup();
void updateLED(uint16_t intensity, uint8_t treshold, uint8_t led);
void initLEDs();
void initLCD();
void lcd_updateMenu();

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Processes input on change and controls output peripheries.
 * Returns:  none
 **********************************************************************/

int main(void)
{
	initialSetup();
	
	uint16_t previous_humidity = UINT16_MAX;
	uint16_t previous_temperature = UINT16_MAX;
	uint16_t previous_luminescence = UINT16_MAX;
    // Infinite loop
    while (1)
    {
		if (humid_flag){
			if (humidity != previous_humidity){
				previous_humidity = humidity;
				updateLED(luminescence, 400, PUMP_LED);
			}
			humid_flag = 0;
		}
		if (temp_flag){
			if (temperature != previous_temperature){
				previous_temperature = temperature;
				updateLED(luminescence, 200, TEMPERATURE_LED);
			}
			temp_flag = 0;
		}
		if (luminescence_flag){
			if (luminescence != previous_luminescence){
				previous_luminescence = luminescence;
				if (luminescence < 300){
					updateLED(luminescence, 300, LIGHT_LED);	
				}
				else if (luminescence > 800){
					servo_right(&PORTB, SERVO_PIN);
				}
			}
			luminescence_flag = 0;
		}
    }

    // Will never reach this
    return 0;
}

void initialSetup(){
	 twi_init();

	 // Initialize UART to asynchronous, 8N1, 9600
	 uart_init(UART_BAUD_SELECT(9600, F_CPU));
	 
	 initLEDs();
	 initSoilSensor(&ADMUX, &ADCSRA);
	 initLCD();
	 
	 servo_init(&DDRB, SERVO_PIN);
	 
	 // Enables interrupts by setting the global interrupt mask
	 sei();
	 //cli();
}

void initInterrupts(){
	// Configure 16-bit Timer/Counter1 to update FSM
	// Set prescaler to 33 ms and enable interrupt
	TIM0_overflow_16ms();
	TIM0_overflow_interrupt_enable();
		 
	TIM1_overflow_262ms();
	TIM1_overflow_interrupt_enable();
}

void initLCD(){
	// Initialize LCD display
	lcd_init(LCD_DISP_ON);

	// Set pointer to beginning of CGRAM memory
	lcd_command(1 << LCD_CGRAM);
	lcd_command(1 << LCD_DDRAM);
	
	lcd_updateMenu();
}

//predelat na itoa
void lcd_updateMenu(){
	char lcd_string[] = "00000000000000000";
	
	lcd_gotoxy(0, 0);
	lcd_puts("H:");
	//sprintf (lcd_string, "H:%d,%d  ", humidity / 10, humidity % 10);
	itoa(humidity / 10,lcd_string,10);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(humidity % 10,lcd_string,10);
	lcd_puts(lcd_string);
	
	lcd_gotoxy(9, 0);
	lcd_puts("T:");
	//sprintf (lcd_string, "T:%d,%d  ", temperature / 10, temperature % 10);
	itoa(temperature / 10,lcd_string,10);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(temperature % 10,lcd_string,10);
	lcd_puts(lcd_string);
	
	lcd_gotoxy(0, 1);
	lcd_puts("L:");
	//sprintf (lcd_string, "L:%d,%d  ", luminescence / 10, luminescence % 10);
	itoa(luminescence / 10,lcd_string,10);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(luminescence % 10,lcd_string,10);
	lcd_puts(lcd_string);
}

void initLEDs(){
	GPIO_config_output(&DDRB, LIGHT_LED);
	GPIO_config_output(&DDRC, PUMP_LED);
	GPIO_config_output(&DDRC, TEMPERATURE_LED);
		
	GPIO_write_high(&PORTB, LIGHT_LED);
	GPIO_write_high(&PORTC, PUMP_LED);
	GPIO_write_high(&PORTC, TEMPERATURE_LED);
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
	static iteration = 1;
	uint16_t result = 0;
	
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	
	if (iteration == 4) {
		result = read_luminescence(&luminescence_flag);
		
		if (luminescence_flag) {
			luminescence = result;
		}
		
		//read DHT12
		result = read_temperature(&temp_flag);
		
		if (temp_flag) {
			temperature = result;
		}
		
		iteration = 0;
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
	lcd_updateMenu();
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
	humidity = ADCW;    // Copy ADC result to 16-bit variable	
	humid_flag = 1;
}