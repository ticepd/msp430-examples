#include <msp430.h> 

#define POUT	P2OUT					// PxOUT for Charlieplexed LEDs
#define PDIR	P2DIR					// PxDIR for Charlieplexed LEDs
#define P1		BIT0					// Charlieplex P1 -> P2.0
#define	P2		BIT1					// Charlieplex P2 -> P2.1
#define P3		BIT2					// Charlieplex P3 -> P2.2
#define P4		BIT3					// Charlieplex P4 -> P2.3

#define SW		BIT3					// Switch -> P1.3

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


unsigned int leds[12]= {0,0,0,0,0,0,0,0,0,0,0,0};	// Global Array of LEDs
int count = 0;										// Switch Press Counter
int dir = 1;										// Count Direction (1 -> Increment, 0 -> Decrement)

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer

    P1DIR &= ~SW;						// Set SW pin -> Input
    P1REN |= SW;						// Enable Resistor for SW pin
    P1OUT |= SW;						// Select Pull Up for SW pin

    P1IES &= ~SW;						// Select Interrupt on Rising Edge
    P1IFG &= ~SW;						// Clear Interrupt Flag
    P1IE |= SW;							// Enable Interrupt on SW pin
    __enable_interrupt();				// Enable CPU Interrupts

    while(1)
    {
    	PDIR &= ~(P1+P2+P3+P4);			// Switch OFF all LEDs
    	unsigned int i;
    	for(i = 0; i < 12; i++)
    	{
    		if(leds[i])					// Check Global LED array
    			charlie(i);				// Switch on LED (i)
    		__delay_cycles(2000);
    	}
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	__delay_cycles(1000);
	if(P1IN & SW)						// Ignore switch presses shorter than 1 ms
	{
		leds[count] = dir;				// Set LED (count) as ON/OFF in Global Array

		if(dir)							// Incremnt/Decrement Count based on Direction
			count++;
		else
			count--;

		if(count == 12)					// Change Direction on Max Increment
		{
			count = 11;
			dir = 0;
		}

		if(count == -1)					// Change Direction on Min Count
		{
			count = 0;
			dir = 1;
		}
	}
    P1IFG &= ~SW;						// Clear Interrupt Flag
}
