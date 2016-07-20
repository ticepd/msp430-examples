#include <msp430.h>

#define LED	BIT6						// Green LED -> P1.6

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;			// Stop watchdog timer

    P1DIR |= LED;						// Set LED pin -> Output
	P1OUT &=~ LED;						// Turn OFF LED

    TACCR0 = 5000;						// Set Timer Timeout Value
    TACCTL0 |= CCIE;					// Enable Overflow Interrupt
    TACTL |= MC_1 + TASSEL_1 + TACLR ;	// Set Mode -> Up Count, Clock -> ACLK, Clear Timer

   	__bis_SR_register(LPM3_bits + GIE);	// Goto LPM3 (Only ACLK active), Enable CPU Interrupt
}

#pragma vector = TIMER0_A0_VECTOR		// CCR0 Interrupt Vector
__interrupt void CCR0_ISR(void)
{
	P1OUT ^= LED;						// Toggle LED
}
