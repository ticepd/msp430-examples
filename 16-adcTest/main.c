#include <msp430.h>

long adcValue, tempC;

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop watchdog timer
	ADC10CTL1 = INCH_10 + ADC10DIV_3;         	// ADC Channel -> 10 (Temp Sensor), CLK/4
	ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
												// Ref -> 1.5V, 64 CLK S&H, Interrupt Enabled
	__delay_cycles(100);						// Wait for reference to settle
	while(1)
	{
		ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
		__bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled

		adcValue = ADC10MEM;					// Fetch ADC conversion result

		// C = ( (adcValue/1024)*1500mV)-986mV)*1/3.55mV
		tempC = ((adcValue - 673) * 423) / 1024;

		__no_operation();                       // Required for breakpoint
	}
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
	__bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
