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
* [Results discussion](#results)
* [Video](#video)
* [References](#references)

<a name="objectives"></a>

## Project objectives

Our aim was to make monitoring system for greenhouse with automatic control of soil moisture, temperature and light intensity. 
Value from every sensor is displayed on LCD display.
If soil moisture value is high then red LED is on, fan is on and windows are open. if it's too low then blue LED is on and water pump starts pumping water to the plant boxes.
If temperature is too high, red LED is on, windows are open and fan is on. if it's too low then blue LED is on and special light bulb for heating is on.
If light intensity value is high, sunblinds are closed. if it's too low then light bulb is on and blinds are open.

![greenhouse objectives](https://github.com/xcerma40/DE2-project-greenhouse/blob/dev/Images/greenhouse%20objectives.png)


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
* **5xLED diode**
  
  ![Greenhouse schmeatic](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/Pictures/greenhouse%20schematic.png)
<a name="libs"></a>

## Libraries description
* [gpio.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/gpio.c) / [gpio.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/gpio.h) - library for controlling General Purpose Input/Output pins
* [lcd.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/lcd.c) / [lcd.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/lcd.h) / [lcd_definitions.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/lcd_definitions.h) - library for controlling LCD display 
* [timer.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/timer.h) - library for controlling the timer modules
* [uart.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/uart.c) / [uart.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/uart.h) - Interrupt UART library with receive/transmit circular buffers
* [twi.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/twi.c) / [twi.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/library/twi.h) - library for TWI serial comunication
  
* [adc_sensors.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/adc_sensors.c) / [adc_sensors.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/adc_sensors.h) - library for reading values from soil moisture sensor

* [i2c_sensors.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/i2c_sensors.c) / [i2c_sensors.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/output_peripherals.h) - library for reading temperature and light intensity values from sensors DHT12 and light intensity sesor 

* [output peripherals.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/output%20peripherals.c) / [output peripherals.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/output_peripherals.h) - Library where treshold values are defined for each sensor and the outpust peripherals are set.
	
* [servo.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/servo.c) / [servo.h](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/src/servo.h) - Library for setting servo positions

<a name="main"></a>

## Main application

[main.c](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/GreenhouseSystem/main.c)

The main controlling unit of our application is the main.c. It contains all the initializations of the different userfunctions defined elsewhere,set up of sensors, lcd, i2c, uart and interrupt service routine of the twotimers/counters used and adc. The function of timer/counter 0 is to update the LCD displace with fresh datacollected from sensors every 16ms, whereas timer/counter 1 facilitates the process of reading data from twoi2c sensor (i.e DHT12 for temperature and BH1750 light sensor) and also start the analogy-digital conversionevery 262ms. The ADC set with a prescaler of 128 handles the reading and conversion of analogy value fromsoil moisture sensor connected to pin A0 of the Arduino uno board to the corresponding moisture value inpercentage.

However, to facilitate easier programming, we created four additional libraries described in the librarydescription section above. Additionally, within the main program, we divided our program into severalfunctins.

All the functions and source codes were created in Microchip studio (formally Atmel studio) and tested onAtmega328P based Arduino uno board.

<a name="results"></a>

## Results discussion

Inline with our objective, we created a simple green house monitoring and control application tomonitor and control the enviromental conditions of the garden. Ideally, the application was intended to workautonimously, whereby the Arduino uno collect input data about the condition of the garden recorded bysensors (soil, light and water), and upon comparing the data with predefined ideal conditions, carry outcounter measures to restore the conditions through output control elements as described in the ourarchitecture diagram shown in the Project objective section.

However, our final realisation is based on the schematic diagram presented in the hardware descriptionsection above, which is semi-automatic. The program collect data from the three sensors(temperature, soilmoisture and light intensity) and displays it on the LCD display. With the temperature thresholds set, whentemperature is above 35 degrees celsius, the high temperature LED connected to PB3 turns indicating thewarning that it is too hot, while when the temperature fall below 25 degrees celsius, the temperature low LEDdenoting a heating element turns on, while when the temperature is within 25 to 35 degree celsius alltemperature indicators turn off. In the same manner, when the moisture in the soil is below 10 %, the soil lowLED would turn on as an indication that the plants need to be watered and when the soil moisture goesbeyond 50%, the high soil LED turns on as a warning to stop the pump. Additionally, light falls below 100 lux,the light LED, denoting an artificial light bulb would turn on. Due to limitations, the water pump would haveto be turned on and off manually.

In our program, most challenging however, was the unable to display the data read from the sensors seriallyon uart. The uart could only display data when only one sensor was connected to i2c, else the programwould crush and do reading of data happened. Another major challenge we faced was the problem withreading data from two different i2c sensors simultaneously, the program would crush unexpectedly. As a result of the mentioned challenges, other functionality of our application such as outomatic control of waterpump using a relay, could not realized.

What next? Withstanding the challenges faced, if all the functionality of our program are realised in full, theapplication can be of vital importance in the application of smart argliculture. The application could also beexpanded to a fully connected iot project by adding wireless connection (GSM, LoRa,..) and a mobile/webapplication, which would enable the farmer to receive data about the status of the garden in real time andcarry out neccessory control measures at the click of the button from anywhere.

![Greenhouse results](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/Pictures/Greenhouse%20real.jpeg)

<a name="video"></a>

## Video

[link to our video](https://www.youtube.com/watch?v=TQb-roNkAjA)

<a name="references"></a>

## References

1. [DHT12 manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/dht12_manual.pdf)
2. [servo manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/SG90-TowerPro.pdf)
3. [soil moisture sensor manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/eses1474354607.pdf)
4. [ATmega328p manual](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
