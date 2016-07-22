#include <msp430.h>

char message[] = { "Hello World ! \r\n" };

void print(char *text)
{
	unsigned int i = 0;
	while(text[i] != '\0')				// Check for end of string
	{
		while (!(IFG2&UCA0TXIFG));		// Check if TX is ongoing
		UCA0TXBUF = text[i];			// TX -> current char
		i++;							// Increment counter
	}
}

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;			// Stop Watchdog
	if (CALBC1_1MHZ==0xFF)				// Check if calibration constant erased
	{
		while(1);						// do not load program
	}
	DCOCTL = 0;							// Select lowest DCO settings
	BCSCTL1 = CALBC1_1MHZ;				// Set DCO to 1 MHz
	DCOCTL = CALDCO_1MHZ;

	P1SEL = BIT1 + BIT2 ;				// Select UART RX/TX function on P1.1,P1.2
	P1SEL2 = BIT1 + BIT2;

	UCA0CTL1 |= UCSSEL_2;				// UART Clock -> SMCLK
	UCA0BR0 = 104;						// Baud Rate Setting for 1MHz 9600
	UCA0BR1 = 0;						// Baud Rate Setting for 1MHz 9600
	UCA0MCTL = UCBRS_1;					// Modulation Setting for 1MHz 9600
	UCA0CTL1 &= ~UCSWRST;				// Initialize UART Module
	
	while(1)
	{
		print(message);
		__delay_cycles(100000);
	}
}
