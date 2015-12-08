/* 
Lab10_T03 Modify the code to display the temperature values in RGB Led, for Green being cool
and red being hot.
*/

#define TEMP_ADDR  0x4F		// Address for Temp Sensor

// Define needed for pin_map.h
#define PART_TM4C123GH6PM

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"

void Print_header();	// Prints Header
float Read_temp();		// Read Temperature sensor

void main(void) {
	float value;

	// Setup the I2C see lab 7 ***************************************************************************************
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //setup clock

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		// Enable I2C hardware
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// Enable Pin hardware

	GPIOPinConfigure(GPIO_PB3_I2C0SDA);				// Configure GPIO pin for I2C Data line
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);				// Configure GPIO Pin for I2C clock line

	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);  // Set Pin Type

	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);// SDA MUST BE STD
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);	// SCL MUST BE OPEN DRAIN
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false); // The False sets the controller to 100kHz communication
	I2CMasterSlaveAddrSet(I2C0_BASE, TEMP_ADDR, true);  				// false means transmit

	// Set up GPIO output for LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  					// PORTF
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3); 	// red and green LEDs

	while(1){
		value = Read_temp();	// Read Data from Temp Sensor
		if (value > 27) // If temp > room temp, light red.
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 2);
		else 			// else, light green.
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 8);
		SysCtlDelay(6000000);	// Delay
	}
}

float Read_temp(){	// Read Temperature sensor
	unsigned char temp[2];	//  storage for data
	float value;

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);	// Start condition
	SysCtlDelay(20000);													// Delay
	temp[0] = I2CMasterDataGet(I2C0_BASE);								// Read first char
	SysCtlDelay(20000);													// Delay
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);		// Push second Char
	SysCtlDelay(20000);													// Delay
	temp[1] = I2CMasterDataGet(I2C0_BASE);								// Read second char
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);	// Stop Condition

	value = temp[0];

	if (temp[1] != 128)
		value += 0.5;
	return value;
}
