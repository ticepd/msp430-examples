#include <msp430.h>

void main(void)
{
  P1DIR |= 0x01;                            // Set P1.0 to output
  P1OUT ^= 0x01;                            // Toggle P1.0
  while(1);
}
