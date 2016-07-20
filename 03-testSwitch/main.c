#include <msp430.h> 

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    P1DIR |= BIT0;				// P1.0 (Red) -> Output

    P1DIR &= ~BIT3;				// P1.3 (SW2) -> Input
    P1REN |= BIT3;				// P1.3 Pull Up/Down Enable
    P1OUT |= BIT3;				// P1.3 Pull Up Enable

    while(1)
    {
    	if(P1IN & BIT3)			// If SW is NOT pressed
    		P1OUT &= ~BIT0;		// LED OFF
    	else
    		P1OUT |= BIT0;		// else LED ON
    }
}
