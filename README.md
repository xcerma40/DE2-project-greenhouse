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



<a name="libs"></a>

## Libraries description

Write your text here.

<a name="main"></a>

## Main application

Write your text here.

<a name="video"></a>

## Video

Write your text here

<a name="references"></a>

## References

1. Write your text here.
