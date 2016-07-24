#include "CTS_Layer.h"

// Uncomment to have this compiler directive run characterization functions only
// Comment to have this compiler directive run example application
//#define ELEMENT_CHARACTERIZATION_MODE	

#ifdef ELEMENT_CHARACTERIZATION_MODE
// Delta Counts returned from the API function for the sensor during characterization
unsigned int dCnt;
#endif

// Main Function
void main(void)
{ 
  // Initialize System Clocks
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  BCSCTL1 = CALBC1_1MHZ;                // Set DCO to 1, 8, 12 or 16MHz
  DCOCTL = CALDCO_1MHZ;
  BCSCTL3 |= LFXT1S_2;                  // LFXT1 = VLO
  
  P2OUT = 0x00;							// Drive all Port 2 pins low
  P2DIR = 0xFF;							// Configure all Port 2 pins outputs

  TI_CAPT_Init_Baseline(&one_button);	// Initialize Baseline measurement
  
  TI_CAPT_Update_Baseline(&one_button,5);   // Update baseline measurement (Average 5 measurements)

  while (1)
  {
  	
  	#ifdef ELEMENT_CHARACTERIZATION_MODE
	// Get the raw delta counts for element characterization 
	TI_CAPT_Custom(&one_button,&dCnt);
	__no_operation(); 					// Set breakpoint here	
	#endif
	  	
	#ifndef ELEMENT_CHARACTERIZATION_MODE	  	
	if(TI_CAPT_Button(&one_button))
	{
		__delay_cycles(10000);
		if(TI_CAPT_Button(&one_button))
			P2OUT |= BIT2;                            // Turn on center LED
	}
	else
	{
		__delay_cycles(10000);
		if(!TI_CAPT_Button(&one_button))
			P2OUT &= ~BIT2;                           // Turn off center LED
	}
	
    __delay_cycles(100000);
    #endif

  }
}
