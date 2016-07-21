#include <msp430.h>

#define GREEN	BIT6
#define AIN		BIT1

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop watchdog timer

	ADC10AE0 |= AIN;                         	// P1.1 ADC option select
	ADC10CTL1 = INCH_1;         				// ADC Channel -> 1 (P1.1)
	ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON;	// Ref -> Vcc, 64 CLK S&H

	P1DIR |= GREEN;								// Green LED -> Output
	P1SEL |= GREEN;								// Green LED -> Select Timer Output

	CCR0 = 1023;								// Set Timer0 PWM Period
	CCTL1 = OUTMOD_7;							// Set TA0.1 Waveform Mode
	CCR1 = 0;									// Set TA0.1 PWM duty cycle
	TACTL = TASSEL_2 + MC_1;					// Timer Clock -> SMCLK, Mode -> Up Count

	while(1)
	{
		ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start

		while(ADC10CTL1 & ADC10BUSY);			// Wait for conversion to end
	
		CCR1 = ADC10MEM;						// Set Duty Cycle = ADC Value
	}
}
