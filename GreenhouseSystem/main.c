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
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
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

/*uint8_t humid_integral = 0;
uint8_t humid_scale = 0;
uint8_t temperature_integral = 0;
uint8_t temperature_scale = 0;*/
uint16_t humidity = 0;
uint16_t temperature = 0;
uint8_t humid_temp_flag = 0;

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and send I2C (TWI) address every 33 ms.
 *           Send information about scanning process to UART.
 * Returns:  none
 **********************************************************************/

int main(void)
{
    // Initialize I2C (TWI)
    twi_init();

    // Initialize UART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    // Configure 16-bit Timer/Counter1 to update FSM
    // Set prescaler to 33 ms and enable interrupt
    TIM1_overflow_33ms();
    TIM1_overflow_interrupt_enable();
	
	TIM0_overflow_16ms();
	TIM0_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put strings to ringbuffer for transmitting via UART
    uart_puts("Greenhouse: init\r\n");

	char uart_string[] = "00000";      // String for converting numbers by itoa()
	
	uint16_t previous_humidity = -1;
	uint16_t previous_temperature = 0;
    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
		if (humid_temp_flag /*&& (humidity != previous_humidity)*/){
			
			//humidity = compose_float(humid_integral, humid_scale);
			uart_puts("Humidity: ");
			itoa(humidity, uart_string, 10);
			uart_puts(uart_string);
			
			//temperature = compose_float(temperature_integral, temperature_scale);
			uart_puts("Temperature: ");
			itoa(temperature, uart_string, 10);
			uart_puts(uart_string);
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

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Update Finite State Machine and test I2C slave addresses 
 *           between 8 and 119.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
	uart_puts("overflow\r\n");
	read_and_send_tmp_hum();
	//scanDevices();
}

ISR(TIMER2_OVF_vect)
{
	uart_puts("overflow\r\n");
	//read_and_send_tmp_hum();
	//scanDevices();
}