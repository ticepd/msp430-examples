#include <msp430.h> 

// Define Pin Mapping of 7-segment Display
// Segments are connected to P2.0 - P2.7
#define SEG_A	BIT0
#define SEG_B	BIT1
#define SEG_C	BIT2
#define SEG_D	BIT3
#define SEG_E	BIT4
#define SEG_F	BIT5
#define SEG_G	BIT6
#define SEG_DP	BIT7

// Define each digit according to truth table
#define D0	(SEG_A + SEG_B + SEG_C + SEG_D + SEG_E + SEG_F)
#define D1	(SEG_B + SEG_C)
#define D2	(SEG_A + SEG_B + SEG_D + SEG_E + SEG_G)
#define D3	(SEG_A + SEG_B + SEG_C + SEG_D + SEG_G)
#define D4	(SEG_B + SEG_C + SEG_F + SEG_G)
#define D5	(SEG_A + SEG_C + SEG_D + SEG_F + SEG_G)
#define D6	(SEG_A + SEG_C + SEG_D + SEG_E + SEG_F + SEG_G)
#define D7	(SEG_A + SEG_B + SEG_C)
#define D8	(SEG_A + SEG_B + SEG_C + SEG_D + SEG_E + SEG_F + SEG_G)
#define D9	(SEG_A + SEG_B + SEG_C + SEG_D + SEG_F + SEG_G)


// Define mask value for all digit segments except DP
#define DMASK	~(SEG_A + SEG_B + SEG_C + SEG_D + SEG_E + SEG_F + SEG_G)

// Store digits in array for display
const unsigned int digits[10] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9};

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer
	
    P2SEL &= ~(BIT6+BIT7);				// Set P2.6 & P2.7 as GPIO (Default is XIN/XOUT)

    // Initialize 7-segment pins as Output
    P2DIR |= (SEG_A + SEG_B + SEG_C + SEG_D + SEG_E+ SEG_F + SEG_G + SEG_DP);

	while(1)
	{
		volatile unsigned int i;
		for(i = 0; i < 10; i++)
		{
			P2OUT = (P2OUT & DMASK) + digits[i];	// Display current digit
			P2OUT ^= SEG_DP;						// Toggle decimal point
			__delay_cycles(1000000);				// Delay 1s
		}
	}
}
