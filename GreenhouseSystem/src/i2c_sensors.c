/*
 * I2C sensors library for Greenhouse system.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 * i2c_sensors.c
 *
 * Created: 14.12.2021
 * Author: Vaclav Cermak, Lungu Masauso, Tereza Berankova
 */ 

/* Includes ----------------------------------------------------------*/
#include "i2c_sensors.h"

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: read_temperature()
 * Purpose:  Reads temperature and humidity from air. Sensor is used only
 *			 for temperature, so air humidity values are thrown away.
 * Input:    temp_flag - is set if data, which were read from sensor, are valid (checksum is correct)
 * Returns:  temperature in °C multiplied by 10
 **********************************************************************/
int16_t read_temperature(volatile uint8_t *temp_flag)
{
	*temp_flag = 0;
	static uint8_t addr = 0x5C;				// 7bit I2C address of DHT12 sensor, needs to be shifted to right for (n)ack

	uint8_t humid_integral = 0;
	uint8_t humid_scale = 0;
	int8_t temperature_integral = 0;
	int8_t temperature_scale = 0;
	
	uint8_t checksum = 0;
	
	uint8_t res = twi_start((addr << 1) + TWI_WRITE);
	
	if (res == 1){	//twi start must return 0 for correct comunication start
		twi_stop();
		return 0;
	}
	
	twi_write(0x00);
	
	twi_start((addr << 1) + TWI_READ);
	
	//humidity is thrown away
	humid_integral = twi_read_ack();		// get fraction part
	humid_scale = twi_read_ack();			// get scale part
	
	temperature_integral = twi_read_ack();  // get fraction part
	temperature_scale = twi_read_ack();		// get scale part

	checksum = twi_read_nack();				// get scale part
	twi_stop();								// end communication
	
	if (checksum == (humid_integral + humid_scale + temperature_integral + temperature_scale)) {
		*temp_flag = 1; // reading was successful, data are valid
	}
	else {
		*temp_flag = 0; // error while reading from DHT12
	}
	
	return temperature_integral * 10 + temperature_scale;	// 25.5 °C -> 255
}

/**********************************************************************
 * Function: init_bh1750()
 * Purpose:  Initializes sensor in High Resolution mode
 * Returns:  none
 **********************************************************************/
void init_bh1750(){
	
	uint8_t addr = 0x23;
	uint8_t mode = 0b00010000;		// value corresponds to High Resolution mode (more information in datasheet)
	
	twi_start((addr<<1) + TWI_WRITE);
	twi_write(mode);
	twi_stop();
}

/************************************************************************
 * Function: get_lux()
 * Purpose:  getCorrect lux value from raw sensor data. Result is inaccurate a bit
 *           for better performance
 ************************************************************************/
uint16_t get_lux(uint16_t data){
	return data;		// correct result is data / 1.2
}

/**********************************************************************
 * Function: read_luminescence()
 * Purpose:  Reads luminescence value from GY30 and converts it to Lux.
 *           More information can be found in sensor's documentation at page 12
 * Input:    luminescence_flag - is set if data, which were read from sensor
 * Returns:  luminescence in Lux
 **********************************************************************/
uint16_t read_luminescence(volatile uint8_t *luminescence_flag){
	
	*luminescence_flag = 0;
	static uint8_t addr = 0x23;			// i2c adress for GY30 (BH1750)
	uint16_t data = -1;
	uint16_t result = 0;

	twi_start((addr<<1) + TWI_READ);
	result = twi_read_ack();			//get high data byte
	data = result << 8;
	result = twi_read_nack();			//get low data byte
	data += result;
	twi_stop();
			
	*luminescence_flag = 1;
			
	return get_lux(data);
}