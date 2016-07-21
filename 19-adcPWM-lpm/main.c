#include <msp430.h>

#define GREEN	BIT6
#define AIN		BIT0
unsigned int adcValue;

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop watchdog timer

	ADC10AE0 |= AIN;                         	// P1.0 ADC option select
	ADC10CTL1 = INCH_0 + ADC10DIV_3;         	// ADC Channel -> 0 (P1.0), CLK/4
	ADC10CTL0 = SREF_0 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
												// Ref -> Vcc, 64 CLK S&H, Interrupt Enabled
	P1DIR |= GREEN;								// Green LED -> Output
	P1SEL |= GREEN;								// Green LED -> Select Timer Output

	CCR0 = 1023;								// Set Timer0 PWM Period
    CCTL0 |= CCIE;								// Enable Overflow Interrupt
	CCTL1 = OUTMOD_7;							// Set TA0.1 Waveform Mode
	CCR1 = 0;									// Set TA0.1 PWM duty cycle
	TACTL = TASSEL_2 + MC_1;					// Timer Clock -> SMCLK, Mode -> Up Count

   	__bis_SR_register(LPM0_bits + GIE);			// Goto LPM0, Enable CPU Interrupt
}

#pragma vector=ADC10_VECTOR						// ADC Conversion Complete Interrupt Vector
__interrupt void ADC10_ISR (void)
{
	CCR1 = ADC10MEM;							// Set Duty Cycle = ADC Value
}

#pragma vector = TIMER0_A0_VECTOR				// CCR0 Interrupt Vector
__interrupt void CCR0_ISR(void)
{
	if(!(ADC10CTL1 & ADC10BUSY))				// Check if ADC Conversion is in progress
		ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
}
