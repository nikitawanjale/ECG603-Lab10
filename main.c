/* 
Lab10_T02 Modify the code to display the X, Y, and Z values of the accelerometer in the UART.
*/

// Addresses for the accelerometer
#define ACCEL_W 0x3A		// Write
#define ACCEL_R 0x3B 		// read
#define ACCEL_X 0x32		// LSB x-axis reg
#define ACCEL_Y 0x34		// LSB y-axis reg
#define ACCEL_Z 0x36		// LSB z-axis reg
#define ACCEL_ADDR 0x1D

// Define needed for pin_map.h
#ifndef PART_TM4C123GH6PM
#define PART_TM4C123GH6PM
#endif

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h" // UART APIs


void Accel_int(); 						// Function prototype to initialize the Accelerometer
signed int Accel_read(unsigned char);	// Function prototype to read the Accelerometer
void Print_header();					// Print Header at start of program
void print_shortInt(signed short int);	// Print short int
void print_axis_header(char);			// Print label for axis being printed

void main(void) {

	signed short int value;

	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //setup clock

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		// Enable I2C hardware
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// Enable Pin hardware

	GPIOPinConfigure(GPIO_PB3_I2C0SDA);				// Configure GPIO pin for I2C Data line
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);				// Configure GPIO Pin for I2C clock line

	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);  		// Set Pin Type

	// Enable UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		// Enable UART hardware
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		// Enable Pin hardware

	GPIOPinConfigure(GPIO_PA0_U0RX);		// Configure GPIO pin for UART RX line
	GPIOPinConfigure(GPIO_PA1_U0TX);		// Configure GPIO Pin for UART TX line
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);     // Set Pins for UART

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,		// Configure UART to 8N1 at 115200bps
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	// Setup the I2C
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);// SDA MUST BE STD
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);	// SCL MUST BE OPEN DRAIN
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false); 							// The False sets the controller to 100kHz communication

	Accel_int();		// Function to initialize the Accelerometer
	Print_header();
	while(1){
		value = Accel_read(ACCEL_X);
		print_axis_header('x');
		print_shortInt(value);

		value = Accel_read(ACCEL_Y);
		print_axis_header('y');
		print_shortInt(value);

		value = Accel_read(ACCEL_Z);
		print_axis_header('z');
		print_shortInt(value);
	}
}

void print_axis_header(char axis)			// Print header for axis
{
	unsigned char *label = "-axis = ";
	int i = 0; 								// general counter
	UARTCharPut(UART0_BASE, '\r');
	UARTCharPut(UART0_BASE, axis);

	while(label[i] != '\0'){				// Print Header at start of program
		UARTCharPut(UART0_BASE, label[i]);
		i++;
	}
}

void print_shortInt(signed short int value){
	char buffer[10];
	char sign = '\0';
	int i = 0; // iterator
	int temp = value;

	if (value == 0)
	{
		UARTCharPut(UART0_BASE, '0');
		UARTCharPut(UART0_BASE, '\n');UARTCharPut(UART0_BASE, '\r');
		return;
	}

	if (value < 0)
	{
		sign  = '-';
		value *= -1;
	}

	// Convert to string
	while(temp != 0) // count the number of digits
	{
		i++;
		temp /= 10;
	}
	buffer[i] = '\0';
	i--;
	for( i; i >= 0; i--) // convert digits to chars, and store in buffer
	{
		buffer[i] = value % 10 + '0';
		value /= 10;
	}
	UARTCharPut(UART0_BASE, sign);
	for(i = 0; i < sizeof(buffer); i++)  // Loop to print out data string
	{
		if (buffer[i] == '\0') break;
		UARTCharPut(UART0_BASE, buffer[i]);
	}
	UARTCharPut(UART0_BASE, '\n');UARTCharPut(UART0_BASE, '\r');
}

void Print_header(){			// Print Header at start of program
	unsigned char *start_screen	= "\n\n\rLab 10 Accelerometer Sensor Read\n\r";
	int i = 0; // general counter

	while(start_screen[i] != '\0'){	// Print Header at start of program
		UARTCharPut(UART0_BASE, start_screen[i]);
		i++;
	}
}

void Accel_int(){													// Function to initialize the Accelerometer

	I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);  			// false means transmit

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);	// Send Start condition

	I2CMasterDataPut(I2C0_BASE, 0x2D); 								// Writing to the Accel control reg
	SysCtlDelay(20000);												// Delay for first transmission
	I2CMasterDataPut(I2C0_BASE, 0x08);								// Send Value to control Register

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);	// Send Stop condition

	while(I2CMasterBusBusy(I2C0_BASE)){};							// Wait for I2C controller to finish operations

}

signed int Accel_read(unsigned char axis_addr) {// Function to read the Accelerometer

	//signed int data;
	signed short value = 0;			// value of x

	unsigned char MSB;
	unsigned char LSB;

	I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);  				// false means transmit

	I2CMasterDataPut(I2C0_BASE, axis_addr);
	SysCtlDelay(20000);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);			// Request LSB of X Axis
	SysCtlDelay(2000000);												// Delay for first transmission

	I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);  				// false means transmit

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);			//Request LSB of X Axis
	SysCtlDelay(20000);

	LSB = I2CMasterDataGet(I2C0_BASE);
	SysCtlDelay(20000);

	I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);  				// false means transmit
	I2CMasterDataPut(I2C0_BASE, axis_addr + 1);

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);			// Request LSB of X Axis
	SysCtlDelay(2000000);												// Delay for first transmission

	I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);  				// false means transmit

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);			//Request LSB of X Axis
	SysCtlDelay(20000);

	MSB = I2CMasterDataGet(I2C0_BASE);

	value = (MSB << 8 | LSB);
	SysCtlDelay(2000);
	return value;
}
