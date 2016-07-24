#include <msp430.h>
#include "lcd.h"

#define DAT_PORT	P2
#define D4			BIT4
#define D5			BIT5
#define D6			BIT6
#define D7			BIT7
#define RS_PORT		P2
#define RS			BIT2
#define EN_PORT		P2
#define EN			BIT3

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD; // stop watchdog

	// lcd_init(data_port, d4, d5, d6, d7, rs_port, rs, en_port, en)
	lcd_init(DAT_PORT, D4, D5, D6, D7, RS_PORT, RS, EN_PORT, EN);
	lcd_setCursor(0,6);
	lcd_print("CEDT");
	lcd_setCursor(1,5);
	lcd_print("MSP430");
	lcd_setCursor(0,0);
	while(1);
}
