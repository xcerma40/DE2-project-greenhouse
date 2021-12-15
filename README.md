# Greenhouse monitoring and control system

### Team members

* Beránková Tereza
* Čermák Václav
* Lungu Masauso


Link to the GitHub repository:

[https://github.com/xcerma40/DE2-project-greenhouse](https://github.com/xcerma40/DE2-project-greenhouse)

### Table of contents

* [Project objectives](#objectives)
* [Hardware description](#hardware)
* [Libraries description](#libs)
* [Main application](#main)
* [Video](#video)
* [References](#references)

<a name="objectives"></a>

## Project objectives

Our aim was to make monitoring system for greenhouse with automatic control of soil moisture, temperature and light intensity. 

<a name="hardware"></a>

## Hardware description
* **ATMEGA 328P**
* **DHT12** - Humidity and temperature sensor
  * we are using it for temperature only
  * Range: -20 to 60°C
  * Operation Voltage: 2.7 to .5 V
* **GY30** - Light intensity sensor 
  * Range: 0–65535LUX
  * Operation Voltage: 3 to 5 V
* **SG90** - Servo motor
  * Operating Voltage = 4.0 to 7.2 V
  * Position "0" (1.5 ms pulse) is middle, "90" (~2ms pulse) is all the way to the righ, "-90" (~1ms pulse) is all the way to the left.
* **Soil mouisture sensor**
  * LM393 comparator
  * Operation Voltage: 3.3 to 5 V
  * Analog output
* **LCD**
  * Hitachi HD44780
  * 2 lines
  * characters per 1 line: 16
  
<a name="libs"></a>

## Libraries description
* **gpio**
  * library for controlling General Purpose Input/Output pins
* **lcd**
  * library for controlling LCD display 
* **timer**
  * library for controlling the timer modules
* **uart**
  * Interrupt UART library with receive/transmit circular buffers
* **twi**
  * library for TWI serial comunication
  
* **adc_sensors.c**
  * library for reading values from soil moisture sensor
```c
/*
 * adc_sensors.c
 *
 * Created: 14.12.2021 12:23:41
 *  Author: cerma
 */ 

#include "adc_sensors.h"

void init_soil_sensor(volatile uint8_t *admux_register, volatile uint8_t *adcsra_register)
{
	// Configure ADC to convert PC0[A0] analog value
	// Set ADC reference to AVcc
	*admux_register |= (1 << REFS0);
	*admux_register &= ~(1 << REFS1);
	// Set input channel to ADC0
	*admux_register &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
	// Enable ADC module
	*adcsra_register |= (1 << ADEN);
	// Enable conversion complete interrupt
	*adcsra_register |= (1 << ADIE);
	// Set clock prescaler to 128
	*adcsra_register |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
```

* **i2c_sensors.c**
	* library for reading temperature and light intensity values from sensors DHT12 and light intensity sesor 
```c
/*
 * input_sensors.c
 *
 * Created: 14.12.2021 1:32:27
 *  Author: cerma
 */ 
#include "i2c_sensors.h"

uint8_t read_temperature(volatile uint8_t *temp_flag)
{
	*temp_flag = 0;
	uint8_t addr = 0x5C;			// 7bit I2C address of humid+temp sensor, needs to be shifted to right for (n)ack

	uint8_t humid_integral = 0;
	uint8_t humid_scale = 0;
	uint8_t temperature_integral = 0;
	uint8_t temperature_scale = 0;
	
	uint8_t checksum = 0;
	
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
		*temp_flag = 1;
	}
	else {
		*temp_flag = 0; // error while reading from DHT12
	}
	
	return (uint16_t)temperature_integral * 10 + (uint16_t)temperature_scale;	// 25.5 °C -> 255
}

/************************************************************************
 * Function: getCorrect lux value from data							*
 * Purpose:  data needs to be shifted, last bit is 2^-1 (+5). Value is 10 times higher.
 ************************************************************************/
uint16_t get_lux(uint16_t data){
	if (data & 1){
		return (((data >> 1) * 10) + 5) / 1.2;
	} else {
		return ((data >> 1) * 10) / 1.2; 	
	}
}

// read data from BH1750 light sensor
uint8_t read_luminescence(volatile uint8_t *luminescence_flag){	//manual str.12
	
	*luminescence_flag = 0;
	static state_bh state = BH_STATE_WRITE;	// Current state of the FSM
	//uint8_t addr = 0x5C;			// ADDR ? 0.7VCC -> H
	uint8_t addr = 0x23;			// ADDR ? 0.3VCC -> L
	uint16_t data = -1;

	// FSM
	switch (state)
	{
		case BH_STATE_WRITE:
			twi_start((addr<<1) + TWI_WRITE);
			twi_write(0b00010001);
			twi_stop();
			
			state = BH_STATE_READ;
		break;
		case BH_STATE_READ:
			twi_start((addr<<1) + TWI_READ);			
			data = twi_read_ack() >> 8;
			data += twi_read_nack();			
			twi_stop();
			
			*luminescence_flag = 1;
			state = BH_STATE_WRITE;
			
			return get_lux(data);
		break;
		default:
			//uart_puts("T1 reading error"); // nastavit error flag?
			state = BH_STATE_WRITE;
		break;
	}
	
	return -1;
}
```
<a name="main"></a>

## Main application

Write your text here.

<a name="video"></a>

## Video

Write your text here

<a name="references"></a>

## References

1. Write your text here.
