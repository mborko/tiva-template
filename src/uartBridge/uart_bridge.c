#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define DELAY(x) SysCtlDelay(SysCtlClockGet() / (1000 * 3) * x)

void UARTBridge(uint32_t, uint32_t);

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
	void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void
UART0IntHandler(void)
{
	uint32_t ui32Status;
	ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
	UARTBridge(UART0_BASE, UART1_BASE);
	ROM_UARTIntClear(UART0_BASE, ui32Status);
}

void
UART1IntHandler(void)
{
	uint32_t ui32Status;
	ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
	UARTBridge(UART1_BASE, UART0_BASE);
	ROM_UARTIntClear(UART1_BASE, ui32Status);
}

void
UARTBridge(uint32_t SOURCE, uint32_t DESTINATION)
{
	//
	// Loop while there are characters in the receive FIFO.
	//
	while(ROM_UARTCharsAvail(SOURCE))
	{
		//
		// Read the next character from the UART and write it back to the UART.
		//
		ROM_UARTCharPutNonBlocking(DESTINATION,
				ROM_UARTCharGetNonBlocking(SOURCE));
	}
	//
	// Blink the LED to show a character transfer is occuring.
	//
	if (SOURCE == UART0_BASE) 
		GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE);
	else 
		GPIOPinWrite(GPIO_PORTF_BASE, LED_GREEN, LED_GREEN);
	GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE|LED_GREEN, 0);
}

int
main(void)
{
	//
	// Set the clocking to run directly from the crystal at 16MHz.
	//
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	g_ui32SysClock = SysCtlClockGet();

	//
	// Enable the GPIO port that is used for the on-board LED.
	// Enable the GPIO pins for the LED (PN0).
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_BLUE|LED_GREEN);

	GPIOPinWrite(GPIO_PORTF_BASE, LED_GREEN, LED_GREEN);
	DELAY(500);
	GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE|LED_GREEN, 0);

	//
	// Enable the peripherals used by this example.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Enable processor interrupts.
	//
	ROM_IntMasterEnable();

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//
	// Set GPIO B0 and B1 as UART pins.
	//
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


	//
	// Configure the UART for 115,200, 8-N-1 operation.
	//
	ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
			 UART_CONFIG_PAR_NONE));
	ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
			 UART_CONFIG_PAR_NONE));

	//
	// Enable the UART interrupt.
	//
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

	ROM_UARTFIFODisable(UART0_BASE);
	ROM_UARTFIFODisable(UART1_BASE);
	
	//
	// Loop forever echoing data through the UART.
	//
	while(1);
}
