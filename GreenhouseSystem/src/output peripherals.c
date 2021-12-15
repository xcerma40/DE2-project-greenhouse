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
	
	lcd_update_menu(0,0,0);
}

//predelat na itoa
void lcd_update_menu(uint16_t humidity, uint16_t temperature, uint16_t luminescence){
	char lcd_string[] = "00000000000000000";
	uint8_t digits_length = 0;
	
	lcd_gotoxy(0, 0);
	lcd_puts("H:");
	//sprintf (lcd_string, "H:%d,%d  ", humidity / 10, humidity % 10);
	itoa(humidity / 10,lcd_string,10);
	digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(humidity % 10,lcd_string,10);
	lcd_puts(lcd_string);
	lcd_fill_whitespace(7 - digits_length);
	
	digits_length = 0;
	lcd_gotoxy(9, 0);
	lcd_puts("T:");
	//sprintf (lcd_string, "T:%d,%d  ", temperature / 10, temperature % 10);
	itoa(temperature / 10,lcd_string,10);
	digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(temperature % 10,lcd_string,10);
	digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_fill_whitespace(4 - digits_length);
	
	digits_length = 0;
	lcd_gotoxy(0, 1);
	lcd_puts("L:");
	//sprintf (lcd_string, "L:%d,%d  ", luminescence / 10, luminescence % 10);
	itoa(luminescence / 10,lcd_string,10);
	digits_length += strlen(lcd_string);
	lcd_puts(lcd_string);
	lcd_putc(',');
	itoa(luminescence % 10,lcd_string,10);
	lcd_puts(lcd_string);
	lcd_fill_whitespace(7 - digits_length);
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
void light_control_update(uint16_t luminescence, uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register){
	
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