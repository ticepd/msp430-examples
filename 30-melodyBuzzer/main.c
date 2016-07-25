#include <msp430.h> 
#include "notes.h"

#define BUZZER	BIT1								// Buzzer -> P2.1
#define F_CPU	1200000L							// CPU Freq approx 1.2 MHz

//Define melody notes and respective duration (1/n seconds)
const unsigned int melody[] = {NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
const unsigned int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4 };

volatile unsigned long count;

// Function to generate a note for a specified duration
void playNote(unsigned int note, unsigned int duration)
{
	volatile unsigned int period, cycles;

	period = F_CPU / note;							// Timer Period = F_CPU / Fnote
	cycles = (F_CPU * duration)/(1000L * period);	// Note duration as number of Timer cycles

	count = cycles;									// Set global count variable
	TA1CCR0 = period;								// Set timer period
	TA1CCR1 = period/2;								// Generate output on TA1.1 at 50% duty cycle
	TA1CTL = TACLR + TASSEL_2 +  MC_1;				// Timer -> SMCLK, Up Mode, Clear
	TA1CCTL0 |= CCIE;								// Enable CCR0 interrupt
	TA1CCTL1 |= OUTMOD_7;							// Output mode for TA1.1

	if(note > 0 && duration > 0)					// Only if note/duration is non zero
		P2SEL |= BUZZER;							// Send Timer Output to pin

	__bis_SR_register(LPM0_bits + GIE);				// Goto LPM and wait for note to complete
	TA1CTL = MC_0;									// Stop Timer
	P2SEL &= ~BUZZER;								// Turn off timer output to pin
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	P2DIR |= BUZZER;								// Set Buzzer pin as Output
	
	while(1)
	{
		unsigned int i;
		for(i = 0; i < 8; i++)						// Play notes one by one
			playNote(melody[i], (1000/noteDurations[i]));
	}
}

#pragma vector = TIMER1_A0_VECTOR					// Timer 1 CCR0 Interrupt Vector
__interrupt void CCR0_ISR(void)
{
	count--;										// Decrement duration counter
	if(count == 0)
		__bic_SR_register_on_exit(LPM0_bits);		// Exit LPM when note is complete
}
