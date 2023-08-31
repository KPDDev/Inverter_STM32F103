/*
 * lcd.c
 *
 *  Created on: Aug 27, 2023
 *      Author: big01
 */

#include "lcd.h"
#include "main.h"			//	 hard ware configuration
//#include "delay.h"
#include "stdarg.h"
#include "stdio.h"


void LCD_LOAD_CGRAM(char tab[], uint8_t charnum)
{
	uint8_t index;
	/* Each character contains 8 definition values*/
	charnum = charnum * 8;
	for(index =0; index < charnum; index++)
	{
	    /* Store values in LCD*/
	    LCD_printchar(tab[index]);

#ifdef USE_HAL_DELAY
	    HAL_Delay(1);
#else
	    delay_ms(1);
#endif

	}

}

void LCD_PWRON (void)	// if want to use application control ON LCD
{
	//__NOP();
}

void LCD_PWROFF (void) // if want to use application control OFF LCD
{
	//__NOP();
}

void LCD_ENABLE(void)
{
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
#ifdef USE_HAL_DELAY
	HAL_Delay(2);
#else
	delay_ms(2);
#endif
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
}

void LCD_CMD(unsigned char data)
{
	char data_lower, data_upper;

	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);  // RS = 0

	data_upper = ((data >>4) & 0x0F);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data_upper>>3)&0x01));
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data_upper>>2)&0x01));
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data_upper>>1)&0x01));
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data_upper>>0)&0x01));
    LCD_ENABLE();

    data_lower = (data & 0x0F);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data_lower>>3)&0x01));
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data_lower>>2)&0x01));
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data_lower>>1)&0x01));
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data_lower>>0)&0x01));
    LCD_ENABLE();

#ifdef USE_HAL_DELAY
	HAL_Delay(2);
#else
	delay_ms(2);
#endif
}

void LCD_INIT(void)
{
	//HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);	Away Low

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);  		// EN = 1
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);		// RS = 0
	 //Initialization of HD44780-based LCD (4-bit HW)
	LCD_CMD(0x33);
#ifdef USE_HAL_DELAY
	HAL_Delay(4);
#else
	delay_ms(4);
#endif
	LCD_CMD(0x32);
#ifdef USE_HAL_DELAY
	HAL_Delay(4);
#else
	delay_ms(4);
#endif
	//Function Set 4-bit mode
	LCD_CMD(0x28);
	//Display On/Off Control
	LCD_CMD(0x0C);
	//Entry mode set
	LCD_CMD(0x06);
	LCD_CLEAR_DISPLAY();
	//Minimum delay to wait before driving LCD module
#ifdef USE_HAL_DELAY
	HAL_Delay(200);
#else
	delay_ms(200);
#endif

}

void LCD_printchar(unsigned char ascode)
{
	char data_lower, data_upper;

	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);		// RS = 1

	data_upper = ((ascode >>4) & 0x0F);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data_upper>>3)&0x01));
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data_upper>>2)&0x01));
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data_upper>>1)&0x01));
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data_upper>>0)&0x01));
    LCD_ENABLE();

    data_lower = (ascode & 0x0F);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data_lower>>3)&0x01));
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data_lower>>2)&0x01));
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data_lower>>1)&0x01));
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data_lower>>0)&0x01));
    LCD_ENABLE();

#ifdef USE_HAL_DELAY
	HAL_Delay(2);
#else
	delay_ms(2);
#endif
}

void LCD_printstring(unsigned char *text)
{
	do
	{
		LCD_printchar(*text++);
	}
	while(*text != '\n');
}

void LCD_CLEAR_DISPLAY(void)
{
	LCD_CMD(0x01);
#ifdef USE_HAL_DELAY
	HAL_Delay(2);
#else
	delay_ms(2);
#endif
}

void LCD_2ndROW(void)
{
	LCD_CMD(0xC0);
}

void LCD_HOME(void)
{
	LCD_CMD(0x02);
#ifdef USE_HAL_DELAY
	HAL_Delay(2);
#else
	delay_ms(2);
#endif
}

void LCD_LSHIFT(void)
{
	LCD_CMD(0x18);
}

void LCD_RSHIFT(void)
{
	LCD_CMD(0x1C);
}

void LCD_DISP_ON(void)
{
	LCD_CMD(0x0C);
}

void LCD_DISP_OFF(void)
{
	LCD_CMD(0x08);
}

void LCD_LOCATE(uint8_t row, uint8_t column)
{
	column--;
	switch(row)
	{
		case 1:
		/* Set cursor to 1st row address and add index*/
		LCD_CMD(column |= 0x80);
		break;
		case 2:
		/* Set cursor to 2nd row address and add index*/
		LCD_CMD(column |= 0x40 | 0x80);
		break;
		default:
		break;
	}
}

void LCD_printf(const char *fmt, ...)
{
	//int32_t i;
	signed i;
	uint32_t text_size, letter;
	static char text_buffer[32];
	va_list args;

	va_start(args, fmt);
	text_size = vsprintf(text_buffer, fmt, args);

	// process the string
	for(i =0; i < text_size; i++)
	{
		letter = text_buffer[i];

		if(letter == 10)
			break;
		else
		{
			if((letter > 0x1F) && (letter < 0x80))
			LCD_printchar(letter);
		}
	}
}

void LCD_CLEARSCREEN(void)
{
	LCD_LOCATE(1, 1);
	LCD_printf("                ");
	LCD_LOCATE(2, 1);
	LCD_printf("                ");
}

