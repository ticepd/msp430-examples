#include <msp430.h> 

#define SW	BIT3					// Switch -> P1.3
#define LED	BIT0					// Red LED -> P1.0

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    P1DIR |= LED;					// Set LED pin -> Output
    P1DIR &= ~SW;					// Set SW pin -> Input
    P1REN |= SW;					// Enable Resistor for SW pin
    P1OUT |= SW;					// Select Pull Up for SW pin

    while(1)
    {
    	if(!(P1IN & SW))			// If SW is Pressed
    	{
    		__delay_cycles(20000);	// Wait 20ms to debounce
    		while(!(P1IN & SW));	// Wait till SW Released
       		P1OUT ^= LED;			// Toggle LED
    	}
    }
}
