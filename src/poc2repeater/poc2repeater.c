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

// Delay for x millisecond.  Each SysCtlDelay is about 3 clocks.
#define DELAY(x) SysCtlDelay(SysCtlClockGet() / (1000 * 3) * x);
// Messagelength
#define MSG						100
// LEDs
#define LED1					GPIO_PIN_0
#define LED2					GPIO_PIN_1
#define LED3					GPIO_PIN_0
#define LED4					GPIO_PIN_4
// UserButtons
#define USR_SW1                 GPIO_PIN_0
#define USR_SW2                 GPIO_PIN_1
#define ALL_BUTTONS             (USR_SW1 | USR_SW2)

uint32_t g_ui32SysClock;

static uint8_t message[MSG];
static uint8_t ptr = 0;

static void UARTLed(uint32_t LEDx);
static void BlinkLed(uint32_t LEDx);
static void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
static void modify_message(uint8_t *msg, uint8_t modifier);
static void transform_message(uint8_t *msg);

static void
UARTLed(uint32_t LEDx)
{
        // Blink the LED to show a character transfer is occuring.
        GPIOPinWrite(GPIO_PORTN_BASE, LEDx, LEDx);
        DELAY(1);
        // Turn off the LED
        GPIOPinWrite(GPIO_PORTN_BASE, LEDx, 0);
}

static void
BlinkLed(uint32_t LEDx)
{
        // Blink the LED to show a character transfer is occuring.
        GPIOPinWrite(GPIO_PORTF_BASE, LEDx, LEDx);
        DELAY(1);
        // Turn off the LED
        GPIOPinWrite(GPIO_PORTF_BASE, LEDx, 0);
}

static void
modify_message(uint8_t *msg, uint8_t modifier)
{
	msg[4]  = msg[0] ^ modifier;
	msg[5]  = msg[1] ^ modifier;
	msg[6]  = msg[2] ^ modifier;
	msg[7]  = msg[3] ^ modifier;
}

static void
transform_message(uint8_t *msg)
{
	uint32_t ui32Buttons;
    ui32Buttons = ROM_GPIOPinRead(GPIO_PORTJ_BASE, ALL_BUTTONS);
	ui32Buttons &= ALL_BUTTONS;
	
	if ( ui32Buttons == USR_SW1 ) {				// Sensor
		BlinkLed(LED3);
		modify_message(msg, 0xFF);
	} else if ( ui32Buttons == USR_SW2 ) {		// Failure
		BlinkLed(LED4);
	    modify_message(msg, 0xAA);
	} else {
		modify_message(msg, 0x55);
	}
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = ROM_UARTIntStatus(UART7_BASE, true);
    ROM_UARTIntClear(UART7_BASE, ui32Status);

    while(ROM_UARTCharsAvail(UART7_BASE))
    {
        // ROM_UARTCharPutNonBlocking(UART7_BASE,ROM_UARTCharGetNonBlocking(UART7_BASE));

		UARTLed(LED1);

		message[ptr] = ROM_UARTCharGetNonBlocking(UART7_BASE);

		if(message[ptr] == '\n')
		{
			  if(ptr == 32) /* hard coded message size ;-) */
			  {
				/* answer first, then display */
				transform_message(message);
				for(uint8_t i = 0; i < ptr; ++i)
				{
				  UARTSend(&message[i],1);
				}
				UARTSend((const uint8_t*) '\n',1);
			  }
			  ptr = 0;
		}
		else
		{
			  ++ptr;
			  
			  if(ptr >= 64)
			  {
				ptr = 0;
			  }
		}
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
static void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        ROM_UARTCharPutNonBlocking(UART7_BASE, *pui8Buffer++);
		UARTLed(LED2);
    }
}

//*****************************************************************************
//
// PoC2Repeater
//
//*****************************************************************************
int
main(void)
{
    // Set the clocking to run directly from the crystal at 120MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    // Enable the peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);


    // Enable the GPIO pins for the LEDs (PN0 and PN1).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);


	// ButtonsInit
    ROM_GPIODirModeSet(GPIO_PORTJ_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE, ALL_BUTTONS,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);	

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // Set GPIO PC4 and PC5 as UART pins.
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Configure the UART for 115,200, 8-N-1 operation.
    ROM_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 |
                             UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    ROM_IntEnable(INT_UART7);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
    
    // Reset message info
    for(uint8_t i = 0; i < MSG; i++)
    	message[i] = 0;

    // Loop forever echoing data through the UART.
    while(1)
    {
    }
}
