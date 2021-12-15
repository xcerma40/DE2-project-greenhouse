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
	uint16_t humid_scale = 0;
	uint16_t temperature_integral = 0;
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
	
	return temperature_integral * 10 + temperature_scale;	// 25.5 °C -> 255
}

void init_bh1750(){
	//uint8_t addr = 0x5C;			// ADDR ? 0.7VCC -> H
	uint8_t addr = 0x23;			// ADDR ? 0.3VCC -> L
	
	twi_start((addr<<1) + TWI_WRITE);
	twi_write(0b00010000);		// high resolution
	twi_stop();
}

/************************************************************************
 * Function: getCorrect lux value from data							*
 * Purpose:  data needs to be shifted, last bit is 2^-1 (+5). Value is 10 times higher.
 ************************************************************************/
uint16_t get_lux(uint16_t data){
	return data * 10;
}

// read data from BH1750 light sensor
uint16_t read_luminescence(volatile uint8_t *luminescence_flag){	//manual str.12
	
	//*luminescence_flag = 0;
	//uint8_t addr = 0x5C;			// ADDR ? 0.7VCC -> H
	static uint8_t addr = 0x23;			// ADDR ? 0.3VCC -> L
	uint16_t data = -1;
	uint16_t result = 0;

	twi_start((addr<<1) + TWI_READ);
	result = twi_read_ack();
	data = result << 8;
	result = twi_read_nack();
	data += result;
	twi_stop();
			
	*luminescence_flag = 1;
			
	return data;
	//return data;
}