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
	static char lcd_string1[] = "00000000";
	static char lcd_string2[] = "00000000";
	static char lcd_string3[] = "00000000";
	
	uint8_t digits_length = 0;
	
	lcd_gotoxy(0, 0);
	lcd_puts("S:");
	//sprintf (lcd_string, "H:%u,%u  ", soil_moisture / 100, soil_moisture % 100);
	dtostrf(soil_moisture,3,2,lcd_string1);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string1);
	//lcd_putc(',');
	//itoa(soil_moisture % 100,lcd_string,10);
	//lcd_puts(lcd_string);
	//lcd_fill_whitespace(7 - digits_length);
	
	//digits_length = 0;
	lcd_gotoxy(9, 0);
	lcd_puts("T:");
	//sprintf (lcd_string, "T:%u,%u  ", temperature / 10, temperature % 10);
	itoa(temperature / 10,lcd_string2,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string2);
	lcd_putc('.');
	itoa(temperature % 10,lcd_string2,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string2);
	//lcd_fill_whitespace(4 - digits_length);
	
	//digits_length = 0;
	lcd_gotoxy(0, 1);
	lcd_puts("L:");
	//sprintf (lcd_string, "L:%u,%u  ", luminescence / 10, luminescence % 10);
	itoa(luminescence / 10,lcd_string3,10);
	//digits_length += strlen(lcd_string);
	lcd_puts(lcd_string3);
	lcd_putc('.');
	itoa(luminescence % 10,lcd_string3,10);
	lcd_puts(lcd_string3);
	//lcd_fill_whitespace(7 - digits_length);
}

void led_turn_on(volatile uint8_t *reg_name, uint8_t led_pin){
	if (GPIO_read(reg_name, led_pin) == 0)
	{
		GPIO_write_high(reg_name, led_pin);
	}
}

void led_turn_off(volatile uint8_t *reg_name, uint8_t led_pin){
	if (GPIO_read(reg_name, led_pin) == 1)
	{
		GPIO_write_low(reg_name, led_pin);
	}
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
		//servo_left(servo_port_register, servo_pin);
		led_turn_on(led_port_register, light_led);	
	}
	// if light conditions are optimal, open pelmet (servo) and turn lights off (led)
	else {
		//servo_right(servo_port_register, servo_pin);
		led_turn_off(led_port_register, light_led);
	}
}

//todo
void temp_control_update(uint16_t temperature, uint8_t led_low, volatile uint8_t *led_low_port_register, uint8_t led_high, volatile uint8_t *led_high_port_register){
	
	if (temperature <= TRESHOLD_TEMPERATURE_COLD) {
		led_turn_on(led_low_port_register, led_low);
	}

	else if (temperature >= TRESHOLD_TEMPERATURE_HOT) {
		led_turn_on(led_high_port_register, led_high);
	}
	else {
		led_turn_off(led_low_port_register, led_low);
		led_turn_off(led_high_port_register, led_high);
	}
}

void soil_control_update(float soil, uint8_t led_low, volatile uint8_t *led_low_port_register, uint8_t led_high, volatile uint8_t *led_high_port_register){

	if (soil <= TRESHOLD_SOIL_LOW) {
		led_turn_on(led_low_port_register, led_low);
	}

	else if (soil >= TRESHOLD_SOIL_HIGH) {
		led_turn_on(led_high_port_register, led_high);
	}
	else {
		led_turn_off(led_low_port_register, led_low);
		led_turn_off(led_high_port_register, led_high);
	}
}