#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "adxl345.h"

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define ASSERT_READY() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_GREEN)
#define ASSERT_STANDBY() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_BLUE)
#define ASSERT_FALSE() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED)

#define DELAY(x) SysCtlDelay(SysCtlClockGet() / (1000 * 3) * x)

#define NUMBYTES 6

void readRegister(uint32_t registerAddress, int numBytes, uint32_t * values)
{

	uint32_t address = 0x0080 | registerAddress;
	if(numBytes > 1) address |= 0x0040;

	// Set ChipSelect to 0
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);

	// Set MSB for read, start at X0 and enable multi-byte
	// and ask for data
	SSIDataPut(SSI2_BASE, address);

	for (int i = 0; i < numBytes; i++ ) {
		SSIDataPut(SSI2_BASE, 0x00 );
		while(SSIBusy(SSI2_BASE));
		// UARTprintf("%d ",values[i]);
		SSIDataGet(SSI2_BASE, values+i );
		*(values+i) &= 0xFF;
	}

	// Set ChipSelect to 1
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0xFF);
}

void InitConsole(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, 16000000);
}

void InitSPI2(void) {
	uint32_t chunk[32];

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 
			GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 );

	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB5_SSI2FSS);
	GPIOPinConfigure(GPIO_PB6_SSI2RX);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);

	GPIOPinTypeSSI(GPIO_PORTB_BASE,
			GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 );

	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
			SSI_MODE_MASTER, 100000, 8);

	SSIDMADisable(SSI2_BASE, SSI_DMA_RX | SSI_DMA_TX);

	SSIEnable(SSI2_BASE);

	while(SSIDataGetNonBlocking(SSI2_BASE, chunk));
	SSIAdvModeSet(SSI2_BASE,SSI_ADV_MODE_READ_WRITE);
}

void InitLED (void)
{
	// Enable LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);
}


int main(void) {
	uint32_t values[10];
	int16_t x,y,z;
	int16_t xg,yg,zg;

	for(int i=0; i<NUMBYTES; i++) *(values+i)=0;

	// Set the clocking to run directly from the external crystal/oscillator.
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

	InitConsole();
	InitLED();
	InitSPI2();

	UARTprintf("............................................\n");

	ASSERT_STANDBY();

	DELAY(500);

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
	SSIDataPut(SSI2_BASE, 0x31); // DATA_FORMAT
	while(SSIBusy(SSI2_BASE));
	SSIDataPut(SSI2_BASE, 0x01); // +/-4g 10-bit mode
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0xFF);

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
	SSIDataPut(SSI2_BASE, 0x2D); // POWER_CTL
	while(SSIBusy(SSI2_BASE));
	SSIDataPut(SSI2_BASE, 0x08);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0xFF);

	DELAY(500);

	ASSERT_READY();

	while (1) {

		readRegister(0x0032,NUMBYTES,values);

		//The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits).
		//To get the full value, two bytes must be combined for each axis.
		//The X value is stored in values[0] and values[1].
		x = ((int)values[1]<<8)|(int)values[0];
		xg = x * 0.0078f;
		//The Y value is stored in values[2] and values[3].
		y = ((int)values[3]<<8)|(int)values[2];
		yg = y*0.0078f;
		//The Z value is stored in values[4] and values[5].
		z = ((int)values[5]<<8)|(int)values[4];
		zg= z*0.0078f;

		UARTprintf("\n%d :: %d :: %d",xg,yg,zg);

		DELAY(1000);
	}

	return(EXIT_FAILURE);
}
