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
Value from every sensor is displayed on LCD display.
If soil moisture value is high then red LED is on, if it's too low then blue LED is on. 
If light intensity value is low, red 


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
  
  ![Greenhouse schmeatic](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/Pictures/greenhouse%20schematic.png)
<a name="libs"></a>

## Libraries description
* [gpio.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/gpio.c) / [gpio.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/gpio.h) - library for controlling General Purpose Input/Output pins
* [lcd.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/lcd.c) / [lcd.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/lcd.h) - library for controlling LCD display 
* [timer.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/timer.h) - library for controlling the timer modules
* [uart.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/uart.c) / [uart.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/uart.h) - Interrupt UART library with receive/transmit circular buffers
* [twi.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/twi.c) / [twi.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/library/twi.h) - library for TWI serial comunication
  
* [adc_sensors.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/adc_sensors.c) / [adc_sensors.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/adc_sensors.h) - library for reading values from soil moisture sensor

* [i2c_sensors.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/i2c_sensors.c) / [i2c_sensors.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/i2c_sensors.h) - library for reading temperature and light intensity values from sensors DHT12 and light intensity sesor 

* [output peripherals.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/output%20peripherals.c) / [output peripherals.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/output_peripherals.h) - Library where treshold values are defined for each sensor and the outpust peripherals are set.
	
* [servo.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/servo.c) / [servo.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/src/servo.h) - Library for setting servo positions

<a name="main"></a>

## Main application

[main.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/GreenhouseSystem/main.c)

<a name="video"></a>

## Video

Write your text here

<a name="references"></a>

## References

1. [DHT12 manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/dht12_manual.pdf)
2. [servo manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/SG90-TowerPro.pdf)
3. [soil moisture sensor manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/eses1474354607.pdf)
4. [ATmega328p manual](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
