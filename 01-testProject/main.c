#include <msp430.h> 

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;		//Stop watchdog timer
	
    P1DIR |= BIT0+BIT6;				//P1.0 (Red) & P1.6 (Grn) -> Output

    P1OUT |= BIT0;					//Red LED -> ON
    P1OUT &= ~BIT6;					//Green LED -> OFF

    while(1)
    {
    	P1OUT ^= (BIT0+BIT6);		//Toggle LEDs

    	volatile unsigned long i;
    	for(i = 0; i < 20000; i++);	//delay

    }
}
