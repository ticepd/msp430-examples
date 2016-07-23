#include <msp430.h>
#include "lcd.h"

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD; // stop watchdog

	lcd_init();
	lcd_setCursor(0,6);
	lcd_print("CEDT");
	lcd_setCursor(1,5);
	lcd_print("MSP430");
	lcd_setCursor(0,0);
	while(1);
}
