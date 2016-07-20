#include <msp430.h> 

#define POUT	P2OUT					// PxOUT for Charlieplexed LEDs
#define PDIR	P2DIR					// PxDIR for Charlieplexed LEDs
#define P1		BIT0					// Charlieplex P1 -> P2.0
#define	P2		BIT1					// Charlieplex P2 -> P2.1
#define P3		BIT2					// Charlieplex P3 -> P2.2
#define P4		BIT3					// Charlieplex P4 -> P2.3

//Data Table for 12 Charliplexed LEDs
const unsigned int hi[12] = {P1,P2,P1,P3,P1,P4,P2,P3,P2,P4,P3,P4};
const unsigned int lo[12] = {P2,P1,P3,P1,P4,P1,P3,P2,P4,P2,P4,P3};
const unsigned int z1[12] = {P3,P3,P2,P2,P2,P2,P1,P1,P1,P1,P1,P1};
const unsigned int z2[12] = {P4,P4,P4,P4,P3,P3,P4,P4,P3,P3,P2,P2};

void charlie(unsigned int value)
{
	PDIR &= ~(z1[value] + z2[value]);	// Set high-Z pins as Input
	PDIR |= (lo[value] + hi[value]);	// Set high & low pins as Output
	POUT &= ~lo[value];					// Set state of low pin
	POUT |= hi[value];					// Set state of high pin
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer

    while(1)
    {
    	unsigned int i;
    	for(i = 0; i < 12; i++)
    	{
    		charlie(i);					// Switch on LED (i)
    		__delay_cycles(100000);		// Delay 100 ms
    	}
    }
}
