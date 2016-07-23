#include <msp430.h> 

#define	CLK		BIT4
#define DATA	BIT2
#define LATCH	BIT5

void shiftOut(unsigned int value, unsigned int length)
{
	P1OUT &= ~LATCH;				// Latch LOW
	volatile unsigned int i;
	for(i = 0; i < length; i++)		// For each bit
	{
		P1OUT &= ~CLK;				// Clock LOW
		if(value & 0x0001)			// Check LSB of value
			P1OUT |= DATA;			// Data HIGH if LSB = 1
		else
			P1OUT &= ~DATA;			// Data LOW if LSB = 0
		P1OUT |= CLK;				// Clock HIGH
		value = value >> 1;			// Move next bit to LSB
	}
	P1OUT |= LATCH;					// Latch HIGH
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	
    P1DIR |= (CLK+DATA+LATCH);		// Set CLK, DATA & LATCH pins as output

    unsigned int pattern = 0x80;	// Initial Value of pattern

    while(1)
    {
    	shiftOut(pattern, 8);		// Output current pattern
    	__delay_cycles(50000);		// Delay 50 ms

    	pattern = pattern >> 1;		// Shift pattern right by 1 bit
    	if(pattern == 0)			// If sequence complete
    		pattern = 0x80;			// Re-initialise value
    }
}
