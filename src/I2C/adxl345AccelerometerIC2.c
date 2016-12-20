#define ACCEL_W 0x3A // Addresses for the accelerometer
#define ACCEL_R 0x3B
#define ACCEL_ADDR 0x1D
#define X_VAL 0x32
#define X_VAL2 0x33
// Define needed for pin_map.h

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

void InitConsole(void); //Init UART

uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);

void Accel_int();

// Function prototype to read the Accelerometer
void main(void) {
//    signed short int LED_value = 1;
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);//setup clock
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);// Enable I2C hardware
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);// Enable Pin hardware
    //GPIOPinConfigure(GPIO_PB3_I2C0SDA);// Configure GPIO pin for I2C Data line
    //GPIOPinConfigure(GPIO_PB2_I2C0SCL);// Configure GPIO Pin for I2C clock line
    //GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);// Set Pin Type
    // Enable Peripheral ports for output
//    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);// LED 1 LED 2
//    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);// LED 4
//    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);// LED 3

    //setup the I2C
    //I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);//The False sets the controller to 100kHz communication

    InitConsole();
    UARTprintf("Console Initialized");
    Accel_int();// Function to initialize the Accelerometer
    UARTprintf("Listening...");
    while (1) {
        uint32_t readVal= I2CReceive(ACCEL_ADDR, X_VAL);
        UARTprintf("Received Value: %d",readVal);

        uint32_t readVal2= I2CReceive(ACCEL_ADDR, X_VAL2);
        UARTprintf("Received Value: %d",readVal2);
    }
}

// Function to initialize the Accelerometer
void Accel_int() {
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
