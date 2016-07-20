#include <msp430.h> 

#define GREEN	BIT6				// Green LED -> P1.6

void delay(unsigned int t)			// Custom delay function
{
	unsigned int i;
	for(i = t; i > 0; i--)
		__delay_cycles(1);			// __delay_cycles accepts only constants !
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    P1DIR |= GREEN;					// Green LED -> Output

    while(1)
    {
    	unsigned int j;
    	for(j = 1; j < 500; j++)	// Increasing Intensity
    	{
   			P1OUT |= GREEN;			// LED ON
   			delay(j);				// Delay for ON Time
   			P1OUT &= ~GREEN;		// LED OFF
   			delay(500-j);			// OFF Time = Period - ON Time
    	}
    	for(j = 500; j > 1; j--)	// Decreasing Intensity
    	{
   			P1OUT |= GREEN;			// LED ON
   			delay(j);				// Delay for ON Time
   			P1OUT &= ~GREEN;		// LED OFF
   			delay(500-j);			// OFF Time = Period - ON Time
    	}
    }
}
