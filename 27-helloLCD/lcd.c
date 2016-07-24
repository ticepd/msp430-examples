#include <msp430.h>
#include "lcd.h"

// Global definitions for port & pin selection
const uint16_t ports[] = { (uint16_t) &P1OUT, (uint16_t) &P2OUT, (uint16_t) &P3OUT};
const uint16_t dirs[] = { (uint16_t) &P1DIR, (uint16_t) &P2DIR, (uint16_t) &P3DIR};
const uint16_t pins[] = {BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7};

uint16_t lcdPins[4], rsPin, enPin;
uint8_t lcdPort, rsPort, enPort;

#define pout(P)		( (volatile uint8_t *)( ports[P] ) )
#define pdir(P)     ( (volatile uint8_t *)( dirs[P] ) )

// Delay function for producing delay in 0.1 ms increments
void delay(uint8_t t)
{
	uint8_t i;
	for(i=t; i > 0; i--)
		__delay_cycles(100);
}

// Function to pulse EN pin after data is written
void pulseEN(void)
{
	volatile uint8_t *enout;
	enout = pout(enPort);

	*enout |= enPin;
	delay(1);
	*enout &= ~enPin;
	delay(1);
}

// Fuction to write 4 bits of data to D4-D7 pins
void write4bits(uint8_t value)
{
	volatile uint8_t *datout;
	datout = pout(lcdPort);
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		if(value & 0x01)
			*datout |= lcdPins[i];
		else
			*datout &= ~lcdPins[i];
		value = value >> 1;
	}
}

//Function to write data/command to LCD
void lcd_write(uint8_t value, uint8_t mode)
{
	volatile uint8_t *rsout;
	rsout = pout(rsPort);

	if(mode == CMD)
		*rsout &= ~rsPin;				// Set RS -> LOW for Command mode
	else
		*rsout |= rsPin;				// Set RS -> HIGH for Data mode

	write4bits(value>>4);				// Write high nibble first
	pulseEN();
	delay(1);

	write4bits(value&0x0F);				// Write low nibble next
	pulseEN();
	delay(1);
}

// Function to print a string on LCD
void lcd_print(char *s)
{
	while(*s)
	{
		lcd_write(*s, DATA);
		s++;
	}
}

// Function to move cursor to desired position on LCD
void lcd_setCursor(uint8_t row, uint8_t col)
{
	const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	lcd_write(LCD_SETDDRAMADDR | (col + row_offsets[row]), CMD);
	delay(1);
}

// Initialize LCD - Specify Port Number, Pin Number of D4, D5, D6, D7, RS and EN
void lcd_init(uint8_t dat_port, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t rs_port, uint8_t rs, uint8_t en_port, uint8_t en)
{
#if defined(EASY_MODE)
	lcdPins[0] = pins[d4];
	lcdPins[1] = pins[d5];
	lcdPins[2] = pins[d6];
	lcdPins[3] = pins[d7];

	rsPin = pins[rs];
	enPin = pins[en];
#else
	lcdPins[0] = d4;
	lcdPins[1] = d5;
	lcdPins[2] = d6;
	lcdPins[3] = d7;

	rsPin = rs;
	enPin = en;
#endif

	// Set SEL bits to GPIO mode for P2.6 & P2.7
	if(dat_port == 2)
		P2SEL &= ~(lcdPins[0] + lcdPins[1] + lcdPins[2] + lcdPins[3]);
	if(rs_port == 2)
		P2SEL &= ~rsPin;
	if(en_port == 2)
		P2SEL &= ~enPin;

	lcdPort = dat_port-1;
	rsPort = rs_port-1;
	enPort = en_port-1;

	volatile uint8_t *datdir;
	volatile uint8_t *rsdir;
	volatile uint8_t *endir;
	volatile uint8_t *datout;
	volatile uint8_t *rsout;
	volatile uint8_t *enout;

	datdir = pdir(lcdPort);
	rsdir = pdir(rsPort);
	endir = pdir(enPort);

	datout = pout(lcdPort);
	rsout = pout(rsPort);
	enout = pout(enPort);

	*datdir |= (lcdPins[0] + lcdPins[1] + lcdPins[2] + lcdPins[3]);
	*rsdir |= rsPin;
	*endir |= enPin;

	*datout &= ~(d4+d5+d6+d7);
	*rsout |= ~rsPin;
	*enout |= ~enPin;

	const char lcdMode = LCD_4BITMODE + LCD_2LINE + LCD_5x8DOTS;
	const char dispMode = LCD_DISPLAYON + LCD_CURSORON + LCD_BLINKON;

	delay(150);											// Wait for power up ( 15ms )
	lcd_write(0x33, CMD);								// Initialization Sequence 1
	delay(50);											// Wait ( 4.1 ms )
	lcd_write(0x32, CMD);								// Initialization Sequence 2
	delay(1);											// Wait ( 100 us )

	// All subsequent commands take 40 us to execute, except clear & cursor return (1.64 ms)

	lcd_write(LCD_FUNCTIONSET | lcdMode, CMD); 			// Set LCD mode
	delay(1);

	lcd_write(LCD_DISPLAYCONTROL | dispMode, CMD); 		// Display on Cursor on
	delay(1);

	lcd_write(LCD_CLEARDISPLAY, CMD); 					// Clear screen
	delay(20);

	lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT, CMD); 	// Auto Increment Cursor
	delay(1);

	lcd_setCursor(0,0); 								// Goto Row 1 Column 1
}
