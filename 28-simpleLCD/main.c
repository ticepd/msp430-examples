#include <msp430.h>
#include <inttypes.h>

#define CMD			0
#define DATA		1

#define LCD_OUT		P2OUT
#define LCD_DIR		P2DIR
#define D4			BIT4
#define D5			BIT5
#define D6			BIT6
#define D7			BIT7
#define RS			BIT2
#define EN			BIT3

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
	LCD_OUT |= EN;
	delay(1);
	LCD_OUT &= ~EN;
	delay(1);
}

//Function to write data/command to LCD
void lcd_write(uint8_t value, uint8_t mode)
{
	if(mode == CMD)
		LCD_OUT &= ~RS;				// Set RS -> LOW for Command mode
	else
		LCD_OUT |= RS;				// Set RS -> HIGH for Data mode

	LCD_OUT = ((LCD_OUT & 0x0F) | (value & 0xF0));				// Write high nibble first
	pulseEN();
	delay(1);

	LCD_OUT = ((LCD_OUT & 0x0F) | ((value << 4) & 0xF0));		// Write low nibble next
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
	const uint8_t row_offsets[] = { 0x00, 0x40};
	lcd_write(0x80 | (col + row_offsets[row]), CMD);
	delay(1);
}

// Initialize LCD
void lcd_init()
{
	P2SEL &= ~(BIT6+BIT7);
	LCD_DIR |= (D4+D5+D6+D7+RS+EN);
	LCD_OUT &= ~(D4+D5+D6+D7+RS+EN);

	delay(150);						// Wait for power up ( 15ms )
	lcd_write(0x33, CMD);			// Initialization Sequence 1
	delay(50);						// Wait ( 4.1 ms )
	lcd_write(0x32, CMD);			// Initialization Sequence 2
	delay(1);						// Wait ( 100 us )

	// All subsequent commands take 40 us to execute, except clear & cursor return (1.64 ms)

	lcd_write(0x28, CMD); 			// 4 bit mode, 2 line
	delay(1);

	lcd_write(0x0F, CMD); 			// Display ON, Cursor ON, Blink ON
	delay(1);

	lcd_write(0x01, CMD); 			// Clear screen
	delay(20);

	lcd_write(0x06, CMD); 			// Auto Increment Cursor
	delay(1);

	lcd_setCursor(0,0); 			// Goto Row 1 Column 1
}

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD; // stop watchdog

	lcd_init();
	lcd_setCursor(0,5);
	lcd_print("TI-CEPD");
	lcd_setCursor(1,5);
	lcd_print("MSP 430");
	lcd_setCursor(0,0);
	while(1);

}
