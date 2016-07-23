#define LCD_PORT	P1OUT
#define LCD_DIR		P1DIR
#define RS			BIT4
#define EN			BIT6
#define D4			BIT0
#define D5			BIT1
#define D6			BIT2
#define	D7			BIT3

#include <msp430.h>
#include "lcd.h"

const uint8_t lcdPins[4] = {D4,D5,D6,D7};

void delay(uint8_t t)
{
	uint8_t i;
	for(i=0; i < t; i++)
		__delay_cycles(1000);
}

void pulseEN(void)
{
	LCD_EN_OUT |= EN;
	delay(1);
	LCD_EN_OUT &= ~EN;
}

void write4bits(uint8_t value)
{
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		if(value & 0x01)
			LCD_DAT_OUT |= lcdPins[i];
		else
			LCD_DAT_OUT &= ~lcdPins[i];
		value = value >> 1;
	}
}

void lcd_write(uint8_t value, uint8_t mode)
{
	if(mode == CMD)
		LCD_RS_OUT &= ~RS;
	else
		LCD_RS_OUT |= RS;

	write4bits(value>>4);
	pulseEN();
	delay(5);

	write4bits(value&0x0F);
	pulseEN();
	delay(5);
}

void lcd_print(char *s)
{
	while(*s)
	{
		lcd_write(*s, DATA);
		s++;
	}
}

void lcd_setCursor(uint8_t row, uint8_t col)
{
	const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	lcd_write(LCD_SETDDRAMADDR | (col + row_offsets[row]), CMD);
	delay(2);
}

void lcd_init(void)
{
	LCD_DAT_DIR	|= (D4+D5+D6+D7);
	LCD_RS_DIR |= RS;
	LCD_EN_DIR |= EN;

	LCD_DAT_OUT &= ~(D4+D5+D6+D7);
	LCD_RS_OUT &= ~RS;
	LCD_EN_OUT &= ~EN;

	const char lcdMode = LCD_4BITMODE + LCD_2LINE + LCD_5x8DOTS;
	const char dispMode = LCD_DISPLAYON + LCD_CURSORON + LCD_BLINKON;

	lcd_write(0x33, CMD);	// Initialization Sequence for 4 bit mode
	delay(50);
	lcd_write(0x32, CMD);	// Initialization Sequence for 4 bit mode
	delay(2);
	lcd_write(LCD_FUNCTIONSET | lcdMode, CMD); 	// Set LCD mode
	delay(2);
	lcd_write(LCD_DISPLAYCONTROL | dispMode, CMD); // Display on Cursor on
	delay(2);
	lcd_write(LCD_CLEARDISPLAY, CMD); // Clear screen
	delay(2);
	lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT, CMD); // Auto Increment Cursor
	delay(2);
	lcd_setCursor(0,0); 		// Goto Row 1 Column 1
	delay(5);
}
