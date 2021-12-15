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
* **output peripherals.c**
	* Library with functions for LCD, LEDs and servo
```c
/*
 * output_peripherals.c
 *
 * Created: 14.12.2021 12:36:31
 *  Author: cerma
 */ 

#include "output_peripherals.h"

void lcd_fill_whitespace(uint8_t length){
	for (uint8_t i = 0; i < length; i++){
		lcd_putc(' ');
	}
}

void init_lcd(){
	// Initialize LCD display
	lcd_init(LCD_DISP_ON);

	// Set pointer to beginning of CGRAM memory
	lcd_command(1 << LCD_CGRAM);
	lcd_command(1 << LCD_DDRAM);
	
	lcd_update_menu(1,1,1);
}

//predelat na itoa
void lcd_update_menu(float soil_moisture, uint16_t temperature, uint16_t luminescence){
	char lcd_string[] = "000000000000000";
	uint8_t digits_length = 0;
	
	lcd_gotoxy(0, 0);
	lcd_puts("S:");
	//sprintf (lcd_string, "H:%u,%u  ", soil_moisture / 100, soil_moisture % 100);
	dtostrf(soil_moisture,3,2,lcd_string);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	//lcd_putc(',');
	//itoa(soil_moisture % 100,lcd_string,10);
	//lcd_puts(lcd_string);
	//lcd_fill_whitespace(7 - digits_length);
	
	//digits_length = 0;
	lcd_gotoxy(9, 0);
	lcd_puts("T:");
	//sprintf (lcd_string, "T:%u,%u  ", temperature / 10, temperature % 10);
	itoa(temperature / 10,lcd_string,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_putc('.');
	itoa(temperature % 10,lcd_string,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	//lcd_fill_whitespace(4 - digits_length);
	
	//digits_length = 0;
	lcd_gotoxy(0, 1);
	lcd_puts("L:");
	//sprintf (lcd_string, "L:%u,%u  ", luminescence / 10, luminescence % 10);
	itoa(luminescence / 10,lcd_string,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_putc('.');
	itoa(luminescence % 10,lcd_string,10);
	lcd_puts(lcd_string);
	//lcd_fill_whitespace(7 - digits_length);
}

void led_turn_on(volatile uint8_t *reg_name, uint8_t led_pin){
	GPIO_write_high(reg_name, led_pin);
}

void led_turn_off(volatile uint8_t *reg_name, uint8_t led_pin){
	GPIO_write_low(reg_name, led_pin);
}

void light_control_init(uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register){
	// open pelmet (servo)
	servo_right(servo_port_register,servo_pin);
	// turn off light (led)
	led_turn_off(led_port_register, light_led);
}
//todo
void light_control_update(uint16_t luminescence, uint8_t light_led, volatile uint8_t *led_port_register, uint8_t servo_pin, volatile uint8_t *servo_port_register){
	
	// whether its too dark or too shiny, close pelmet (servo) and turn artificial lighting on (led)
	if (luminescence <= TRESHOLD_LUMINESCENCE_DARK || luminescence >= TRESHOLD_LUMINESCENCE_LIGHT){
		servo_left(servo_port_register, servo_pin);
		led_turn_on(led_port_register, light_led);
	}
	// if light conditions are optimal, open pelmet (servo) and turn lights off (led)
	else {
		servo_right(servo_port_register, servo_pin);
		led_turn_off(led_port_register, light_led);
	}
}
//todo
void temperature_control_update(uint16_t luminescence, uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register){
	static state_lc actual_state = LC_STATE_OPTIMAL;
	static state_lc previous_state = LC_STATE_OPTIMAL;
	
	/*switch(actual_state){
		case : LC_STATE_OPTIMAL
			if (luminescence <= TRESHOLD_LUMINESCENCE_DARK || luminescence >= TRESHOLD_LUMINESCENCE_LIGHT){
				servo_left(servo_port_register, servo_pin);
				led_turn_on(led_port_register, light_led);
				previous_state = actual_state;
			}
		break;
		case : LC_STATE_DL
			
		break;
		default:
			state = LC_STATE_OPTIMAL;
		break;
	}*/
	
	// whether its too dark or too shiny, close pelmet (servo) and turn artificial lighting on (led)
	if (luminescence <= TRESHOLD_LUMINESCENCE_DARK || luminescence >= TRESHOLD_LUMINESCENCE_LIGHT){
		servo_left(servo_port_register, servo_pin);
		led_turn_on(led_port_register, light_led);
	}
	// if light conditions are optimal, open pelmet (servo) and turn lights off (led)
	else {
		servo_right(servo_port_register, servo_pin);
		led_turn_off(led_port_register, light_led);
	}
}
```
* **servo.c**
```c
/*
 * GPIO library for AVR-GCC.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 * servo.c
 *
 * Created: 14.12.2021 0:46:24
 * Author: Vaclav Cermak
 */ 

/* Includes ----------------------------------------------------------*/
#include "servo.h"
#include "../library/gpio.h"

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: servo_left()
 * Purpose:  Rotate servo fully to the left by sending input signal of length 20ms
 *           in ratio 0,7ms (high) : 19,3ms (low)
 * Input:    reg_name - Address of servos Port register, such as &PORTB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_left(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_write_high(&PORTB, servo_pin);
	_delay_us(700);
	GPIO_write_low(&PORTB, servo_pin);
	_delay_us(300);
	_delay_ms(19);
};

/**********************************************************************
 * Function: servo_right()
 * Purpose:  Rotate servo fully to the right by sending input signal of length 20ms
 *           in ratio 2,4ms (high) : 17,6ms (low)
 * Input:    reg_name - Address of servos Port register, such as &PORTB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_right(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_write_high(&PORTB, servo_pin);
	_delay_us(2400);
	GPIO_write_low(&PORTB, servo_pin);
	_delay_us(600);
	_delay_ms(17);
};

/**********************************************************************
 * Function: servo_init()
 * Purpose:  Initialize servo in Data Direction Register
 * Input:    reg_name - Address of Data Direction Register, such as &DDRB
 *           servo_pin - Pin to which servo is connected
 * Returns:  none
 **********************************************************************/
void servo_init(volatile uint8_t *reg_name, uint8_t servo_pin){
	GPIO_config_output(reg_name, servo_pin);	
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

1. [DHT12 manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/dht12_manual.pdf)
2. [servo manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/SG90-TowerPro.pdf)
3. [soil moisture sensor manual](https://github.com/xcerma40/DE2-project-greenhouse/blob/master/dokumenty/eses1474354607.pdf)
