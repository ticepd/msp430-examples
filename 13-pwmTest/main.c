#include <msp430.h>

#define GREEN	BIT6						// Green LED -> P1.6

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;				// Stop WDT
	P1DIR |= GREEN;							// Green LED -> Output
	P1SEL |= GREEN;							// Green LED -> Select Timer Output

	CCR0 = 500;								// Set Timer0 PWM Period
	CCTL1 = OUTMOD_7;						// Set TA0.1 Waveform Mode - Clear on Compare, Set on Overflow
	CCR1 = 1;								// Set TA0.1 PWM duty cycle
	TACTL = TASSEL_2 + MC_1;				// Timer Clock -> SMCLK, Mode -> Up Count

	while(1)
	{
		unsigned int i;
		for(i = 0; i < 500; i++)
		{
			CCR1 = i;						// Increase Duty from min to max
			__delay_cycles(5000);
		}
		for(i = 500; i > 0; i--)
		{
			CCR1 = i;						// Decrease Duty from max to min
			__delay_cycles(5000);
		}
	}
}
