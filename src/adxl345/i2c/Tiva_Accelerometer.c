//============================================================================
// Name        : Tiva_Accelerometer.c
// Author      : Mahendra Gunawardena
// Version     : Rev 0.01
// Copyright   : Your copyright notice
// Description : Tiva ADXL345Accelerometer in C++, Ansi-style
//============================================================================
/*
 * Tiva_Accelerometer.c
 * Implementation of a class to interface with the Analog Devices ADXL345 3 Axis Accelerometer
 * over the I2C bus
 *
 * Copyright Mahendra Gunawardena, Mitisa LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ADXL345_Accelerometer.h"

#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define ASSERT_READY() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_GREEN)
#define ASSERT_STANDBY() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_BLUE)
#define ASSERT_FALSE() GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED)

#define DELAY(x) SysCtlDelay(SysCtlClockGet() / (1000 * 3) * x)

void InitConsole(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, 16000000);
}

void InitLED(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);
}


int main (void)
{
	uint8_t i2c_data;
	unsigned char dev_id[2];
	signed int accelerationX, accelerationY, accelerationZ;

	// Set the clocking to run directly from the external crystal/oscillator.
	//SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	InitLED();
	InitConsole();
	UARTprintf("............................................\n");

	ASSERT_STANDBY();

	init_ADXL345_Accelerometer();
	i2c_data = getAccelerometer_ID();
	dev_id[0] = ((i2c_data & 0xF0) >> 4) + 0x30 + 0x07;
	dev_id[1] = (i2c_data & 0x0F) + 0x30;
	SetPowerMode(0x01);

	ASSERT_READY();

	while(1){
		accelerationX = getAcceleration_X();
		if (accelerationX>=0) {
			UARTprintf("\nx: %d", accelerationX);
		} else {
			UARTprintf("\nx: -%d",(accelerationX*-1));
		}

		accelerationY = getAcceleration_Y();
		if (accelerationY>=0) {
			UARTprintf("\ny: %d", accelerationY);
		} else {
			UARTprintf("\ny: -%d",(accelerationY*-1));
		}

		accelerationZ = getAcceleration_Z();
		if (accelerationZ>=0) {
			UARTprintf("\nz: %d", accelerationZ);
		} else {
			UARTprintf("\nz: -%d",(accelerationZ*-1));
		}
		DELAY(1000);
	}

	return 0;
}
