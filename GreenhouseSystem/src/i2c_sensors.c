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
/************************************************************************/
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