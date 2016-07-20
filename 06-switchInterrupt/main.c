#include <msp430.h> 

#define SW		BIT3					// Switch -> P1.3
#define GREEN	BIT6					// Green LED -> P1.6

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer

    P1DIR |= GREEN;						// Set LED pin -> Output
    P1DIR &= ~SW;						// Set SW pin -> Input
    P1REN |= SW;						// Enable Resistor for SW pin
    P1OUT |= SW;						// Select Pull Up for SW pin

    P1IES &= ~SW;						// Select Interrupt on Rising Edge
    P1IE |= SW;							// Enable Interrupt on SW pin

    __bis_SR_register(LPM4_bits + GIE);	// Enter LPM4 and Enable CPU Interrupt
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	P1OUT ^= GREEN;						// Toggle Green LED
	P1IFG &= ~SW;						// Clear SW interrupt flag
}
