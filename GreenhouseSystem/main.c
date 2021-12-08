/***********************************************************************
 * 
 * The I2C bus scanner detects the addresses of the modules that are 
 * connected to the SDA and SCL signals. A simple description of FSM is 
 * used.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021-Present Vaclav Cermak
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * 
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif
#define LIGHT_LED		PB4		//Green
#define PUMP_LED		PC2		// Output pin for controlling water pump
#define TEMPERATURE_LED	PC3
#define servo			PB2
#define SourceVoltage	5000	// Voltage value of the source in mV
#define wet_soil		400     // Define max value to consider soil 'wet'
#define dry_soil		850     // Minimum value for dry soil

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>         // C library. Needed for conversion function
#include "timer.h"          // Timer library for AVR-GCC
#include "uart.h"           // Peter Fleury's UART library
#include "twi.h"            // TWI library for AVR-GCC
#include "lcd.h"
#include "lcd_definitions.h"
#include "gpio.h"

/* Variables ---------------------------------------------------------*/

typedef enum {             // FSM for luminescence sensor
	STATE_WRITE,
	STATE_READ
} state_bh;

uint16_t humidity = 0;
uint16_t temperature = 0;
uint16_t luminescence = 0;
uint8_t temp_flag = 0;
uint8_t humid_flag = 0;
uint8_t luminescence_flag = 0;


/* Function declarations ----------------------------------------------*/

void initialSetup();
uint8_t read_temperature();
uint8_t read_luminescence();
void updateLED(uint16_t intensity, uint8_t treshold, uint8_t led);
void servoLeft();
void servoRight();
void initSoilSensor();
void initLEDs();
void initLCD();
void lcd_updateMenu();

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and send I2C (TWI) address every 33 ms.
 *           Send information about scanning process to UART.
 * Returns:  none
 **********************************************************************/

int main(void)
{
	initialSetup();

	char uart_string[] = "00000000000000000";      // String for converting numbers by itoa()
	
	uint16_t previous_humidity = -10000;
	uint16_t previous_temperature = -10000;
	uint16_t previous_luminescence = -10000;
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
					servoRight()
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
	 initSoilSensor();
	 initLCD();
	 
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

void initSoilSensor(){
	// Configure ADC to convert PC0[A0] analog value
	// Set ADC reference to AVcc
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	// Set input channel to ADC0
	ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
	// Enable ADC module
	ADCSRA |= (1 << ADEN);
	// Enable conversion complete interrupt
	ADCSRA |= (1 << ADIE);
	// Set clock prescaler to 128
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void updateLED(uint16_t intensity, uint8_t treshold, uint8_t led){
	//uart_puts("lightCheck\r\n");
	if (intensity < treshold){ // luminescence < 10.0
		PORTC = PORTC | (1<<led);
	}
	else {
		PORTC = PORTC & ~(1<<led);
	}
}

//open pelmet
void servoLeft(){
	GPIO_write_high(&PORTB, servo);
	_delay_us(700);
	GPIO_write_low(&PORTB, servo);
	_delay_us(300);
	_delay_ms(19);
};

//close pelmet
void servoRight(){
	GPIO_write_high(&PORTB, servo);
	_delay_us(2400);
	GPIO_write_low(&PORTB, servo);
	_delay_us(600);
	_delay_ms(17);
};

uint8_t read_temperature()
{
	temp_flag = 0;
	uint8_t addr = 0x5C;			// 7bit I2C address of humid+temp sensor, needs to be shifted to right for (n)ack

	uint8_t humid_integral = 0;
	uint8_t humid_scale = 0;
	uint8_t temperature_integral = 0;
	uint8_t temperature_scale = 0;
	
	uint8_t checksum = 0;
		
	char us1 [] = "00000000000000";
	
	uint8_t res = twi_start((addr << 1) + TWI_WRITE);
	
	if (res == 1){
		twi_stop();
		return 0;
	}
	
	twi_write(0x00);
	
	twi_start((addr << 1) + TWI_READ);
	
	//tohle zahazuju
	humid_integral = twi_read_ack();    // get fraction part 
	humid_scale = twi_read_ack();			// get scale part
	
	temperature_integral = twi_read_ack();    // get fraction part
	temperature_scale = twi_read_ack();			// get scale part

	checksum = twi_read_nack();			// get scale part
	twi_stop();
	
	if (checksum == (humid_integral + humid_scale + temperature_integral + temperature_scale)) {
		temp_flag = 1;	
		temperature = (uint16_t)temperature_integral * 10 + (uint16_t)temperature_scale;	// 25.5 °C -> 255
		//humidity = (uint16_t)humid_integral * 10 + (uint16_t)humid_scale;
	}
	else {
		temp_flag = 0; // spis nastavit error flag
	}
	
	
}

/************************************************************************
 * Function: getCorrect lux value from data							*
 * Purpose:  data needs to be shifted, last bit is 2^-1 (+5). Value is 10 times higher.
/************************************************************************/
uint16_t get_lux(uint16_t data){
	if (data & 1){
		return (((data >> 1) * 10) + 5) / 1.2;
	} else {
		return ((data >> 1) * 10) / 1.2; 	
	}
}

// read data from BH1750 light sensor
uint8_t read_luminescence(){	//manual str.12
	
	luminescence_flag = 0;
	static state_bh state = STATE_WRITE;	// Current state of the FSM
	//uint8_t addr = 0x5C;			// ADDR ? 0.7VCC -> H
	uint8_t addr = 0x23;			// ADDR ? 0.3VCC -> L
	uint16_t data = -1;
	uint8_t result = -1;

	// FSM
	switch (state)
	{
		case STATE_WRITE:
			result = twi_start((addr<<1) + TWI_WRITE);
			twi_write(0b00010001);
			twi_stop();
			state = STATE_READ;
		break;
		case STATE_READ:
			twi_start((addr<<1) + TWI_READ);
			data = twi_read_ack() >> 8;
			data += twi_read_nack();
			twi_stop();
			
			luminescence = get_lux(data);
			luminescence_flag = 1;
			
			state = STATE_WRITE;
		break;
		default:
			//uart_puts("T1 reading error"); // nastavit error flag?
			state = STATE_WRITE;
		break;
	}
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Service routine for slow actions. Every second, Humidity and temperature is being
 *           read from DHT12 and luminescence from BH1750.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
	static iteration = 1;
	//read light sensitivity
	
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	
	if (iteration == 4) {
		read_luminescence();
		//read DHT12
		read_temperature();
		iteration = 0;
	}
	iteration++;
}

ISR(TIMER0_OVF_vect)
{
	lcd_updateMenu();
}

/* -------------------------------------------------------------------*/
/**
 * ISR starts when ADC completes the conversion. Display soil moisture
 * on LCD and send it to UART. high adc value correspond to dry soil and 
 * low value wet soil
 */
ISR(ADC_vect)
{
	uint16_t soil_moisture = 0;
	soil_moisture = ADCW;    // Copy ADC result to 16-bit variable
	humidity = soil_moisture;
	
	humid_flag = 1;
}