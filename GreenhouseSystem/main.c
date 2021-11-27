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

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>         // C library. Needed for conversion function
#include "timer.h"          // Timer library for AVR-GCC
#include "uart.h"           // Peter Fleury's UART library
#include "twi.h"            // TWI library for AVR-GCC

/* Variables ---------------------------------------------------------*/
typedef enum {              // FSM declaration
    STATE_IDLE = 1,
    STATE_SEND,
    STATE_ACK
} state_t;

typedef enum {              // FSM declaration
	STATE_WRITE,
	STATE_READ
} state_bh;

uint16_t humidity = 0;
uint16_t temperature = 0;
uint8_t humid_temp_flag = 0;

uint16_t luminescence = 0;
uint8_t luminescence_flag = 0;


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and send I2C (TWI) address every 33 ms.
 *           Send information about scanning process to UART.
 * Returns:  none
 **********************************************************************/
void lcd_updateMenu(){
	char lcd_string[] = "00000000000000000";
	
	lcd_gotoxy(0, 0);
	sprintf (lcd_string, "H:%d,%d\r\n", humidity / 10, humidity % 10);
	lcd_puts(lcd_string);
	lcd_gotoxy(9, 0);
	sprintf (lcd_string, "T:%d,%d\r\n", temperature / 10, temperature % 10);
	lcd_puts(lcd_string);
	lcd_gotoxy(0, 1);
	sprintf (lcd_string, "L:%d,%d\r\n", luminescence / 10, luminescence % 10);
	lcd_puts(lcd_string);
}

int main(void)
{
    // Initialize I2C (TWI)
    twi_init();

    // Initialize UART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
	
	// Initialize LCD display
	lcd_init(LCD_DISP_ON);

	// Put string(s) at LCD display
	// Set pointer to beginning of CGRAM memory
	lcd_command(1 << LCD_CGRAM);

    // Configure 16-bit Timer/Counter1 to update FSM
    // Set prescaler to 33 ms and enable interrupt
	TIM0_overflow_16ms();
	TIM0_overflow_interrupt_enable();
	
    TIM1_overflow_262ms();
    TIM1_overflow_interrupt_enable();

	TIM2_overflow_16ms();
	TIM2_overflow_interrupt_enable();
	
    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put strings to ringbuffer for transmitting via UART
    uart_puts("Greenhouse: init\r\n");
	lcd_updateMenu();

	char uart_string[] = "00000000000000000";      // String for converting numbers by itoa()
	
	uint16_t previous_humidity = -10000;
	uint16_t previous_temperature = -10000;
	uint16_t previous_luminescence = -10000;
    // Infinite loop
    while (1)
    {
		if (humid_temp_flag){
			if (humidity != previous_humidity){
				sprintf (uart_string, "Humidity: %d,%d\r\n", humidity / 10, humidity % 10);
				uart_puts(uart_string);	
			}
			if (temperature != previous_temperature){
				sprintf (uart_string, "Temperature: %d,%d\r\n", temperature / 10, temperature % 10);
				uart_puts(uart_string);
			}
			humid_temp_flag = 0;
		}
		if (luminescence_flag){
			if (luminescence != previous_luminescence){
				sprintf (uart_string, "Luminescence: %d,%d\r\n", luminescence / 10, luminescence % 10);
				uart_puts(uart_string);
			}
			luminescence_flag = 0;
		}
    }

    // Will never reach this
    return 0;
}

void read_and_send_tmp_hum()
{
	uart_puts("reading data from DHT12\r\n");
	uint8_t addr = 0xB8 >> 1;			// 7bit I2C address of humid+temp sensor, needs to be shifted to right for (n)ack

	uint8_t humid_integral = 0;
	uint8_t humid_scale = 0;
	uint8_t temperature_integral = 0;
	uint8_t temperature_scale = 0;
	
	humid_temp_flag = 0;
	uint8_t checksum = 0;
		
	twi_start((addr<<1) + TWI_READ);
   
	humid_integral = twi_read_ack();    // get fraction part 
	humid_scale = twi_read_ack();			// get scale part
	
	temperature_integral = twi_read_ack();    // get fraction part
	temperature_scale = twi_read_ack();			// get scale part

	checksum = twi_read_nack();			// get scale part
	twi_stop();
	
	if (checksum == (humid_integral | humid_scale | temperature_integral | temperature_scale)) {
		humid_temp_flag = 1;	
		temperature = temperature_integral * 10 + temperature_scale;	// 25.5 °C -> 255
	}
	else {
		humid_temp_flag = 0;
		humidity = humid_integral * 10 + humid_scale;
	}
	
	
}

/************************************************************************
 * Function: getCorrect lux value from data							*
 * Purpose:  data needs to be shifted, last bit is 2^-1. Value is 10 times higher.
/************************************************************************/
uint16_t get_lux(uint16_t data){
	if (data & 1){
		return (((data >> 1) * 10) + 5) / 1.2;
	} else {
		return ((data >> 1) * 10) / 1.2; 	
	}
}

// read data from BH1750 light sensor
void read_luminescence(){	//manual str.12
	static state_bh state = STATE_IDLE;	// Current state of the FSM
	//uint8_t addr = 0x5C >> 1;			// ADDR ? 0.7VCC -> H
	uint8_t addr = 0x23 >> 1;			// ADDR ? 0.3VCC -> L
	uint16_t result = -1;
	luminescence_flag = 0;
	
	/*twi_start((addr<<1) + TWI_READ);
	
	humid_integral = twi_read_ack();    // get fraction part
	humid_scale = twi_read_ack();*/

	// FSM
	switch (state)
	{
		case STATE_WRITE:
			twi_start((addr<<1) + TWI_WRITE);
			twi_write(0b00010001);
			twi_stop();
		break;
		case STATE_READ:
			twi_start((addr<<1) + TWI_READ);
			result = twi_read_ack() >> 8;
			result += twi_read_nack();
			twi_stop();
			
			luminescence = get_lux(result);
			luminescence_flag = 1;
		break;
		default:
			uart_puts("T1 reading error");
			state = STATE_WRITE;
		break;
	}
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Update Finite State Machine and test I2C slave addresses 
 *           between 8 and 119.
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
	uart_puts("TIM1_overflow\r\n");
	read_and_send_tmp_hum();
	//scanDevices();
}

ISR(TIMER1_OVF_vect)
{
	//read light sensitivity
	uart_puts("TIM0_overflow\r\n");
	read_luminescence();
	//scanDevices();
}

ISR(TIMER3_OVF_vect)
{
	//update lcd
	lcd_updateMenu();
}