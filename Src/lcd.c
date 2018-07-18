/*
 * lcd.c
 *
 *  Created on: 15 Ìבס 2018
 *      Author: Thanos
 */

#include "string.h"
#include "main.h"
#include "lcd.h"
#include "Macro_Definitions.h"
/*  LCD Screen Instructions:
 *
 * 						1)lcd_reset() : Used to reset and initialize the clock
 * 						2)lcd_write('',(value of RS)) : Used to write only one character
 * 						3)lcd_display_settings((turn LCD On),(Underline the last character written),(blink the last character written))
 * 						4)lcd_display_address() : sets the address to a specific point
 * 						5)lcd_print("") : prints the whole string
 *
 */

/* You have to extern the three variables below to your main.c */
uint8_t lcd_chars;
uint8_t lcd_lines;
uint8_t *lcd_line_addresses;

/*       Defines for the LCD       */

#define LCD_PORT GPIOD
#define LCD_RCC RCC_GPIOD
#define LCD_RS GPIO_PIN_10
#define LCD_CLOCK GPIO_PIN_11
#define LCD_4 GPIO_PIN_12
#define LCD_5 GPIO_PIN_13
#define LCD_6 GPIO_PIN_14
#define LCD_7 GPIO_PIN_15

/*      End Of Defines for the LCD       */

/*     Functions for the LCD Screen     */

// "Private" globals
uint8_t _lcd_char = 0;
uint8_t _lcd_line = 0;

void lcd_clock(void)
{
	// Pulse clock
	HAL_GPIO_WritePin(LCD_PORT,LCD_CLOCK,GPIO_PIN_SET);
	delay(1);
	HAL_GPIO_WritePin(LCD_PORT,LCD_CLOCK,GPIO_PIN_RESET);
	delay(1);
}

void lcd_reset(void)
{
	// Resets display from any state to 4-bit mode, first nibble.

	// Set everything low first
	HAL_GPIO_WritePin(LCD_PORT,LCD_RS,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_CLOCK,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_7,GPIO_PIN_RESET);

	// Reset strategy below based on Wikipedia description, should recover
	// from any setting

	// Write 0b0011 three times
	HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_SET);
	lcd_clock();
	lcd_clock();
	lcd_clock();
	// LCD now guaranteed to be in 8-bit state

	// Now write 0b0010 (set to 4-bit mode, ready for first nibble)
	HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_RESET);
	lcd_clock();
}

void lcd_write(uint8_t byte, uint8_t rs)
{
	// Writes a byte to the display (rs must be either 0 or 1)

	// Write second nibble and set RS

	if((byte >> 4 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_RESET);
	if((byte >> 5 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_RESET);
	if((byte >> 6 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_6,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_6,GPIO_PIN_RESET);
	if((byte >> 7 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_7,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_7,GPIO_PIN_RESET);
	if(rs)
		HAL_GPIO_WritePin(LCD_PORT,LCD_RS,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_RS,GPIO_PIN_RESET);

	lcd_clock();

	// Write first nibble

	if(byte & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_4,GPIO_PIN_RESET);
	if((byte >> 1 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_5,GPIO_PIN_RESET);
	if((byte >> 2 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_6,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_6,GPIO_PIN_RESET);
	if((byte >> 3 ) & 1)
		HAL_GPIO_WritePin(LCD_PORT,LCD_7,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCD_PORT,LCD_7,GPIO_PIN_RESET);

	lcd_clock();
}

void lcd_clear(void)
{
	// Clears display, resets cursor
	lcd_write(0b00000001, 0);
	_lcd_char = 0;
	_lcd_line = 0;
}

void lcd_display_settings(uint8_t on, uint8_t underline, uint8_t blink)
{
	// "Display On/Off & Cursor" command. All parameters must be either 0 or 1

	lcd_write(0b00001000 | (on << 2) | (underline << 1) | blink, 0);
}

void lcd_display_address(uint8_t address)
{
	lcd_write(0b10000000 | address, 0);
}

void lcd_print(char string[])
{
	uint8_t i;
	for(i = 0; string[i] != 0; i++) {
		// If we know the display properties and a newline character is
		// present, print the rest of the string on the new line.
		if(lcd_lines && string[i] == '\n') {
			if(_lcd_line < lcd_lines) {
				lcd_display_address(lcd_line_addresses[_lcd_line++]);
				_lcd_char = 0;
			}
		}
		else {
			// If we know the display properties and have reached the end of
			// line, print the rest on the next line
			if(lcd_chars)
				if((_lcd_char == lcd_chars) && (_lcd_line < lcd_lines)) {
					lcd_display_address(lcd_line_addresses[_lcd_line++]);
					_lcd_char = 0;
				}
			lcd_write(string[i], 1);
			if(lcd_chars) _lcd_char++;
		}
	}
}
/*     End Of Functions for the LCD Screen     */


