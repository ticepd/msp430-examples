#ifndef LCD_H_
#define LCD_H_

#include <inttypes.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#if defined(LCD_PORT) && defined(LCD_DIR)
#define LCD_DAT_OUT		LCD_PORT
#define LCD_RS_OUT		LCD_PORT
#define LCD_EN_OUT		LCD_PORT
#define LCD_DAT_DIR		LCD_DIR
#define LCD_RS_DIR		LCD_DIR
#define LCD_EN_DIR		LCD_DIR
#endif

#define CMD		0
#define DATA	1

void lcd_init(void);
void lcd_setCursor(uint8_t, uint8_t);
void lcd_print(char *);
void lcd_write(uint8_t, uint8_t);
void write4bits(uint8_t);
void pulseEN(void);
void delay(uint8_t);

#endif /* LCD_H_ */
