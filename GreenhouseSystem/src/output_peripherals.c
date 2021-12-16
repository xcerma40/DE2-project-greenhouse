/*
 * Output Peripherals library for Greenhouse system.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 * output_peripherals.c
 *
 * Created: 14.12.2021
 * Author: Vaclav Cermak, Lungu Masauso, Tereza Berankova
 */ 

/* Includes ----------------------------------------------------------*/
#include "output_peripherals.h"

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: lcd_fill_whitespace()
 * Purpose:  Fill whitespace on LCD to remove sagging characters from previous display refresh.
 *           Unused for optimalization purposes
 * Input:    length - Number of whitespace characters to be printed
 * Returns:  none
 **********************************************************************/
void lcd_fill_whitespace(uint8_t length){
	for (uint8_t i = 0; i < length; i++){
		lcd_putc(' ');
	}
}

/**********************************************************************
 * Function: init_lcd()
 * Purpose:  Initialize display, set pointer to beginning of the CGRAM memory and print default values on LCD
 * Returns:  none
 **********************************************************************/
void init_lcd(){
	// Initialize LCD display
	lcd_init(LCD_DISP_ON);

	// Set pointer to beginning of CGRAM memory
	lcd_command(1 << LCD_CGRAM);
	lcd_command(1 << LCD_DDRAM);
	
	lcd_update_menu(1,1,1);
}

/**********************************************************************
 * Function: lcd_update_menu()
 * Purpose:  Updates parameter's values on LCD
 * Input:    soil_moisture - soil moisture in %
 * Input:	 temperature temperature in °C, value needs to be 10 times higher because of parsing
 * Returns:  none
 **********************************************************************/
void lcd_update_menu(float soil_moisture, int16_t temperature, uint16_t luminescence){
	static char lcd_string1[] = "00000000";
	static char lcd_string2[] = "00000000";
	static char lcd_string3[] = "00000000";
	
	// print soil moisture value on display as "S:50.45"
	lcd_gotoxy(0, 0);
	lcd_puts("S:");
	dtostrf(soil_moisture,3,2,lcd_string1);
	lcd_puts(lcd_string1);
	
	// print soil temperature value on display as "T:26.8"
	lcd_gotoxy(9, 0);
	lcd_puts("T:");
	itoa(temperature / 10,lcd_string2,10);
	lcd_puts(lcd_string2);
	lcd_putc('.');
	itoa(temperature % 10,lcd_string2,10);
	lcd_puts(lcd_string2);
	
	// print luminescence value on display as "L:450"
	lcd_gotoxy(0, 1);
	lcd_puts("L:");
	itoa(luminescence,lcd_string3,10);
	lcd_puts(lcd_string3);
}

/**********************************************************************
 * Function: led_turn_on()
 * Purpose:  Turns LED on. Checks if it's not on already.
 * Input:    reg_name - PORT register name, such as PORTB
 * Input:	 led_pin - pin, to which LED is connected
 * Returns:  none
 **********************************************************************/
void led_turn_on(volatile uint8_t *reg_name, uint8_t led_pin){
	if (GPIO_read(reg_name, led_pin) == 0)
	{
		GPIO_write_high(reg_name, led_pin);
	}
}

/**********************************************************************
 * Function: led_turn_off()
 * Purpose:  Turns LED off. Checks if it's not off already.
 * Input:    reg_name - PORT register name, such as PORTB
 * Input:	 led_pin - pin, to which LED is connected
 * Returns:  none
 **********************************************************************/
void led_turn_off(volatile uint8_t *reg_name, uint8_t led_pin){
	if (GPIO_read(reg_name, led_pin) == 1)
	{
		GPIO_write_low(reg_name, led_pin);
	}
}

/**********************************************************************
 * Function: light_control_init()
 * Purpose:  Initializes output peripheries at start -> open pelmet servo and turn off lights
 * Input:	 light_led - LED that simulates artificial lightning
 * Input:	 servo_pin - pin, to which servo is attached
 * Input:	 servo_port_register - servo's port register -> PORTB for example
 * Returns:  none
 **********************************************************************/
void light_control_init(uint8_t light_led, uint8_t *led_port_register, uint8_t servo_pin, uint8_t *servo_port_register){
	// open pelmet (servo)
	servo_right(servo_port_register,servo_pin);
	// turn off light (led)
	led_turn_off(led_port_register, light_led);
}

/**********************************************************************
 * Function: light_control_update()
 * Purpose:  Checks if luminescence is in optimal span. If not, pelmet is closed and artificial lightning is turned on.
 * Input:    lumincescence - luminescence in Lux
 * Input:	 light_led - LED that simulates artificial lightning
 * Input:	 servo_pin - pin, to which servo is attached
 * Input:	 servo_port_register - servo's port register -> PORTB for example
 * Returns:  none
 **********************************************************************/
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

/**********************************************************************
 * Function: temp_control_update()
 * Purpose:  Checks whether temperature is in optimal span. If too high or low, corresponding leds are turned on.
 * Input:    temperature - temperature value in celsius degrees to be checked (value is 10 times higher than real value)
 * Input:	 led_low - pin of the LED, that will be turned on if temperature is too low
 * Input:	 led_low_port_register - LEDs PORT register, such as PORTB
 * Input:	 led_high - pin of the LED, that will be turned on if temperature is too high
 * Input:	 led_high_port_register - LEDs PORT register, such as PORTB
 * Returns:  none
 **********************************************************************/
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

/**********************************************************************
 * Function: soil_control_update()
 * Purpose:  Checks if soil moisture is in optimal span. If too high or low, corresponding leds are turned on.
 * Input:    soil soil moisture value to be checked
 * Input:	 led_low - pin of the LED, that will be turned on if temperature is too low
 * Input:	 led_low_port_register - LEDs PORT register, such as PORTB
 * Input:	 led_high - pin of the LED, that will be turned on if temperature is too high
 * Input:	 led_high_port_register - LEDs PORT register, such as PORTB
 * Returns:  none
 **********************************************************************/
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