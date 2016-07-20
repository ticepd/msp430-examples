#include <msp430.h> 

#define SW		BIT3						// Switch -> P1.3
#define RED		BIT0						// Red LED -> P1.0
#define GREEN 	BIT6						// Green LED -> P1.6

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer

    P1DIR |= RED+GREEN;					// Set LED pins -> Output
    P1DIR &= ~SW;						// Set SW pin -> Input
    P1REN |= SW;						// Enable Resistor for SW pin
    P1OUT |= SW;						// Select Pull Up for SW pin

    volatile unsigned int flag = 0;
    while(1)
    {
    	if(!(P1IN & SW))				// If SW is Pressed
    	{
    		__delay_cycles(20000);		// Wait 20ms
    		if(!(P1IN & SW))			// Check if SW is still pressed
    		{							// Ignores presses shorter than 20ms
    			while(!(P1IN & SW));	// Wait till SW Released
    			flag = !flag;			// Change flag value
    		}
    	}
    	if(flag)						// Check flag value
    	{
    		P1OUT &= ~GREEN;			// Green -> OFF
    		P1OUT |= RED;				// Red -> ON
    	}
    	else
    	{
    		P1OUT &= ~RED;				// Red -> OFF
    		P1OUT |= GREEN;				// Green -> ON
    	}
    }
}
