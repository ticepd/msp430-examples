#include <msp430.h>

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 		// Stop watchdog timer
	P1DIR |= BIT5;                     				// Set 1.5 as Output (Latch)

	P1SEL = BIT2 + BIT4;							// Select P1.2 -> SIMO (Data)
	P1SEL2 = BIT2 + BIT4;							// Select P1.4 -> CLK (Clock)

	UCA0CTL1 |= UCSWRST;                     		// Hold USCI in reset state
	UCA0CTL0 |= UCCKPL + UCMST + UCSYNC;  			// 3-pin, 8-bit, SPI Master
	UCA0CTL1 |= UCSSEL_2;                     		// Clock -> SMCLK
	UCA0BR0 = 0x02;                          		// SPI CLK -> SMCLK/2
	UCA0CTL1 &= ~UCSWRST;                     		// Initialise USCI module

	while(1)
	{
		P1OUT &= ~BIT5;								// Set Latch to LOW
		while (!(IFG2 & UCA0TXIFG));				// Check if TX Buffer is empty
		UCA0TXBUF = 0xAA;							// Transmit first pattern
		while ((UCA0STAT & UCBUSY));				// Wait till TX Completes
		P1OUT |= BIT5;								// Set Latch to HIGH
		__delay_cycles(500000);						// Delay 500 ms

		P1OUT &= ~BIT5;								// Set Latch to LOW
		while (!(IFG2 & UCA0TXIFG));				// Check if TX Buffer is empty
		UCA0TXBUF = 0x55;							// Transmit second pattern
		while ((UCA0STAT & UCBUSY));				// Wait till TX Completes
		P1OUT |= BIT5;								// Set Latch to HIGH
		__delay_cycles(500000);						// Delay 500 ms
	}
}
