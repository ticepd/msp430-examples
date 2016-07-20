#include <msp430.h> 

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	
    P1DIR |= (BIT0+BIT6);			// P1.0 (Red LED), P1.1 (Green LED)

    while(1)
    {
    	volatile unsigned long i;

    	P1OUT &= ~BIT6;				//Green LED -> OFF
    	P1OUT |= BIT0;				//Red LED -> ON

    	for(i = 0; i<10000; i++);	//delay

    	P1OUT &= ~BIT0;				//Red LED -> OFF
    	P1OUT |= BIT6;				//Green LED -> ON

    	for(i = 0; i<10000; i++);	//delay
    }
}
