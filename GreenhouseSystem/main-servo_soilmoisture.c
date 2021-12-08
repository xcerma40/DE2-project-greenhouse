/*
 * green_house_monitor.c
 *
 * Created: 11/26/2021 12:44:01 PM
 * Author : masau
 */ 

#ifndef F_CPU
# define F_CPU 16000000 /**< @brief CPU frequency in Hz required for delay */
#endif

#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include "lcd.h"            // Peter Fleury's LCD library
#include "uart.h"           // library for serial communication
#include "twi.h"            // library for i2c comunication
#include "gpio.h"           // library for manipulating gpio
#include <stdlib.h>         // C library. Needed for conversion function

#include <util/delay.h>
#include <string.h>

#define wet_soil 400       // Define max value to consider soil 'wet'
#define dry_soil 850       // Minimum value for dry soil
#define pump     PB4       // Output pin for controlling water pump
#define servo    PB2
#define SourceVoltage 5000 // Voltage value of the source in mV


int main(void)
{
	// Initialize LCD display
	lcd_init(LCD_DISP_ON);
	lcd_gotoxy(0, 0); lcd_puts("Soil:");
	lcd_gotoxy(0, 1); lcd_puts("T():");
	lcd_gotoxy(7, 0); lcd_puts("a");        // Put ADC soil moisture value in decimal
	lcd_gotoxy(7,1); lcd_puts("b");         // Put temperature value in decimal
	lcd_gotoxy(9, 0); lcd_puts("H(%):");    // Humidity here
	lcd_gotoxy(9, 1); lcd_puts("L:");       // Humidity here
	
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
	
	// Initialize UART to asynchronous, 8N1, 9600
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	
	// Configure 16-bit Timer/Counter1 to start ADC conversion
	// Enable interrupt and set the overflow prescaler to 262 ms
	TIM1_overflow_262ms();
	TIM1_overflow_interrupt_enable();
	
	GPIO_config_output(&DDRB, pump);
	GPIO_config_output(&DDRB, servo);

	// Enables interrupts by setting the global interrupt mask
	sei();
	
    while (1) 
    {
		////empty
		//GPIO_write_high(&PORTB, servo);
		//_delay_us(2400);
		//GPIO_write_low(&PORTB, servo);
		//_delay_us(600);
		//_delay_ms(17);
		
    }
}

ISR(TIMER1_OVF_vect)
{
    // Start ADC conversion
	ADCSRA |= (1 << ADSC);	
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
	char lcd_string[8] = "00000000";
	soil_moisture = ADCW;    // Copy ADC result to 16-bit variable

	if (soil_moisture > dry_soil)
	{
		//uart_puts("Status: soil too dry, water now!");
		GPIO_write_high(&PORTB, pump);
	}
	else if ((soil_moisture >= wet_soil) && (soil_moisture < dry_soil))
	{
		//uart_puts("Status: Soil moisture is perfect");
		GPIO_write_low(&PORTB,pump);                  //turn pump off
	}
	
	else if (soil_moisture < wet_soil)
	{
		//uart_puts("Status: Soil too wet, stop watering");
		GPIO_write_low(&PORTB, pump);                   //turn pump off
	}
	
	// clear decimal and hex positions
	lcd_gotoxy(6,0);
	lcd_puts("        ");

	itoa(soil_moisture, lcd_string, 10);    // Convert to string in decimal
	lcd_gotoxy(6,0);
	lcd_puts(lcd_string);
	
	uart_puts("Soil status: ");
	uart_puts(lcd_string);
	uart_puts("\r\n");
	
}