# MSP430 CCS Code Examples
Various Code Examples for MSP430G2553 (used at the Training Programme of TI-CEPD)

## List of Examples
| Project Name          | Description   |
| ----------------------|---------------|
| 00-blink              | Blink onboard Red LED
| 01-testProject        | Blink onboard Red and Green LEDs alternatively
| 02-alternateBlink     | Different implementation of alternative blinking
| 03-testSwitch         | Turn ON/OFF onboard LED using onboard Switch
| 04-switchToggle       | Toggle the onboard LED using the onboard Switch
| 05-switchAlternate    | Toggle between Red and Green LED using onboard Switch
| 06-switchInterrupt    | Toggle LED using external interrupt and low power modes 
| 07-blinkWatchdog      | Illustrates the Watchdog Timer timeout and reset 
| 08-charlieplexing     | Using 4 pins to control 12 LEDs using Charlieplexing
| 09-charlieSwitch      | Increment/Decrement charlieplexed LEDs using onboard Switch
| 10-timerBlink         | Use internal timer and low frequency oscillator to blink LED
| 11-sevenSegment       | Countinuously count from 0-9 on a seven segment disply
| 12-sevenSegCounter    | Use the onboard switch to increment the count on the seven segment display
| 13-pwmTest            | Fading effect on onboard Green LED using hardware PWM
| 14-softwarePWM        | Implementation of PWM using software time delays
| 15-timerCapture		| Uses the Input Capture mode of Timer to measure external signal period
| 16-adcTest			| Reads the internal temperature sensor and converts value to Celcius
| 17-dcoTest			| Cycles through calibrated DCO frequencies during runtime
| 18-adcPWM				| Use ADC input to control intensity of onboard Green LED
| 19-adcPWM-lpm			| Implementation of ADC based PWM control using low power modes
| 20-uartEcho			| Implements UART link at a baud rate of 9600. Echoes RX char + 1
| 21-helloWorld			| Prints the string Hello World ! on the UART link
| 22-uartPrintNum		| Converts an integer value to string and prints it on the UART link
| 23-uartADC			| Displays the ADC value on the UART link
| 24-spiShift			| Use SPI protocol to send data to a shift register
| 25-i2cTempSensor		| Read temperature values from a LM75 sensor over I2C
| 26-bitBangShift		| Implements SPI communication with shift register by bit banging GPIO
| 27-helloLCD			| Connects to a 16x2 LCD using 4-bit mode, generic library .c & .h files
| 28-simpleLCD			| Simplified version of 16x2 LCD, requires all LCD pins to be on same port
| 29-capTouch			| Implementation of capacitive touch sensing using TI Libraries
| 30-melodyBuzzer		| Play a musical melody on a piezo buzzer using timers
