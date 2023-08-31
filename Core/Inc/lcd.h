/*
 * lcd.h
 *
 *  Created on: Aug 27, 2023
 *      Author: big01
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f1xx.h"

#define USE_HAL_DELAY		0

/* Exported types ------------------------------------------------------------*/
/* LCD HW settings */

/* HD44780 CGRAM address start */
#define CGRAM_address_start 0x40

void LCD_LOAD_CGRAM(char tab[], uint8_t charnum);
void LCD_PWRON (void);
void LCD_PWROFF (void);
void LCD_INIT(void);
void LCD_printchar(unsigned char ascode);
void LCD_printstring(unsigned char *text);
void LCD_CMD(unsigned char data);
void LCD_CLEAR_DISPLAY(void);
void LCD_2ndROW(void);
void LCD_HOME(void);
void LCD_LSHIFT(void);
void LCD_RSHIFT(void);
void LCD_DISP_ON(void);
void LCD_DISP_OFF(void);
void LCD_LOCATE(uint8_t row, uint8_t column);
void LCD_printf(const char *fmt, ...);

void LCD_CLEARSCREEN(void);

#endif /* INC_LCD_H_ */
