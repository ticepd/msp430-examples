#include <msp430.h> 

#define SW		BIT3
#define GREEN	BIT6

volatile unsigned int mode = 0;

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer
	
	if (CALBC1_1MHZ==0xFF)				// Check if calibration constant erased
	{
		while(1);						// Do not run program. Hold here.
	}

	DCOCTL = 0;							// Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;				// Set DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	P1DIR |= GREEN;						// Green LED -> Output
	P1SEL |= GREEN;						// Green LED -> Select Timer Output

	CCR0 = 99;							// Set Timer0 PWM Period
	CCTL1 = OUTMOD_7;					// Set TA0.1 Waveform Mode - Clear on Compare, Set on Overflow
	CCR1 = 50;							// Set TA0.1 PWM duty cycle
	TACTL = TASSEL_2 + MC_1;			// Timer Clock -> SMCLK, Mode -> Up Count

    P1DIR &= ~SW;						// Set SW pin -> Input
    P1REN |= SW;						// Enable Resistor for SW pin
    P1OUT |= SW;						// Select Pull Up for SW pin

    P1IES &= ~SW;						// Select Interrupt on Rising Edge
    P1IE |= SW;							// Enable Interrupt on SW pin

    __bis_SR_register(LPM0_bits + GIE);	// Enter LPM0 and Enable CPU Interrupt

}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	switch(mode)
	{
		case 0:
			BCSCTL1 = CALBC1_1MHZ;				// Set DCO to 1MHz
			DCOCTL = CALDCO_1MHZ;
			break;
		case 1:
			BCSCTL1 = CALBC1_8MHZ;				// Set DCO to 8MHz
			DCOCTL = CALDCO_8MHZ;
			break;
		case 2:
			BCSCTL1 = CALBC1_12MHZ;				// Set DCO to 12MHz
			DCOCTL = CALDCO_12MHZ;
			break;
		case 3:
			BCSCTL1 = CALBC1_16MHZ;				// Set DCO to 16MHz
			DCOCTL = CALDCO_16MHZ;
			break;
		default:
			break;
	}
	mode++;
	if(mode > 3)
		mode = 0;
	P1IFG &= ~SW;
}
