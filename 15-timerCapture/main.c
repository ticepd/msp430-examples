#include <msp430.h>

volatile unsigned int count, edge1, edge2, period;	// Global variables
volatile unsigned long freq;

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 		// Stop watchdog timer

	if (CALBC1_1MHZ==0xFF)							// Check if calibration constant erased
	{
		while(1);                               	// Do not run program. Hold here.
	}

	DCOCTL = 0;                               		// Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;                    		// Set DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	P1DIR &= ~BIT2;                           		// Set P1.2 -> Input
	P1SEL |= BIT2;                            		// Set P1.2 -> TA0.1 Capture Mode

	TA0CCTL1 = CAP + CM_1 + CCIE + SCS + CCIS_0;	// Capture Mode, Rising Edge, Interrupt
                                            		// Enable, Synchronize, Source -> CCI0A
	TA0CTL |= TASSEL_2 + MC_2 + TACLR;        		// Clock -> SMCLK, Cont. Mode, Clear Timer

	while(1)
	{
		count = 0;									// Initialise count for new capture
		__bis_SR_register(LPM0_bits + GIE);   		// Enter LPM0, Enable Interrupt

		//Exits LPM0 after 2 rising edges are captured

		if(edge2 > edge1)							// Ignore calculation if overflow occured
		{
			period = edge2 - edge1;             	// Calculate Period
			freq = 1000000L/period;
		}
		__no_operation();                     		// For inserting breakpoint in debugger
	}
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR (void)
{
	switch(__even_in_range(TA0IV,0x0A))
	{
		case  TA0IV_NONE: break;              				// Vector  0:  No interrupt

		case  TA0IV_TACCR1:                   				// Vector  2:  TACCR1 CCIFG

			if (!count)										// Check value of count
            {
                edge1 = TA0CCR1;							// Store timer value of 1st edge
                count++;									// Increment count
            }
            else
            {
                edge2 = TA0CCR1;							// Store timer value of 2nd edge
                count=0;									// Reset count
                __bic_SR_register_on_exit(LPM0_bits + GIE);	// Exit LPM0 on return to main
            }
			break;

		case TA0IV_TACCR2: break;             				// Vector  4:  TACCR2 CCIFG
		case TA0IV_6: break;                  				// Vector  6:  Reserved CCIFG
		case TA0IV_8: break;                  				// Vector  8:  Reserved CCIFG
		case TA0IV_TAIFG: break;              				// Vector 10:  TAIFG
		default: 	break;
	}
}
