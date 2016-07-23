#include <msp430.h>

unsigned int count;
unsigned int data;

int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;					// Stop Watchdog
	P1SEL |= BIT6 + BIT7;						// Set P1.6 -> SCL & P1.7 -> SDA
	P1SEL2|= BIT6 + BIT7;
	UCB0CTL1 |= UCSWRST;						// Hold USCI Module in Reset State
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C Master, synchronous mode
	UCB0CTL1 |= UCSSEL_2;						// Use SMCLK
	UCB0BR0 = 12;								// fSCL = SMCLK/12 = ~100kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x48;							// Set slave address for LM75
	UCB0CTL1 &= ~UCSWRST;						// Initialise USCI Module
	IE2 |= UCB0RXIE;							// Enable RX interrupt

	while (1)
	{
	  count = 2;                          		// Reset RX byte counter
	  UCB0CTL1 |= UCTXSTT;                    	// Send I2C start condition
	  __bis_SR_register(CPUOFF + GIE);        	// Enter LPM0, enable interrupts
	  // Remains in LPM0 until data is received

	  data &= 0x01FF;							// Use only 9 bits of data
	  data = data / 2;
	  __delay_cycles(100000);					// Delay 100 ms
  }
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	count--;									// Decrement RX byte counter

	if (count)
	{
		data = (unsigned int)UCB0RXBUF<<1;		// Get first received byte
		UCB0CTL1 |= UCTXSTP;					// Generate I2C stop condition
	}
	else
	{
		data |= UCB0RXBUF>>7;					// Get second received byte & combine
		__bic_SR_register_on_exit(CPUOFF);		// Exit LPM0
	}
}

