/*
 * lcd.h
 *
 *  Created on: 15 Ìבס 2018
 *      Author: Thanos
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void lcd_clock(void);
void lcd_reset(void);
void lcd_write(uint8_t byte, uint8_t rs);
void lcd_clear(void);
void lcd_display_settings(uint8_t on, uint8_t underline, uint8_t blink);
void lcd_display_address(uint8_t address);
void lcd_print(char string[]);



#endif /* LCD_H_ */
