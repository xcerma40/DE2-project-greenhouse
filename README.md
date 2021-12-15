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
  * library for soil moisture sensor
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

<a name="main"></a>

## Main application

Write your text here.

<a name="video"></a>

## Video

Write your text here

<a name="references"></a>

## References

1. Write your text here.
