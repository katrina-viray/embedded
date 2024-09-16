#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include <stdbool.h>
//#include "utils/uartstdio.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
int count = 0;
char inputStr[4];
void FWD(void);
void BWD(void);
void LFT(void);
void RGT(void);

typedef struct{
    char cmd [4];
    void (*fp)(void);
} Cmd;

const Cmd cmdLookUp[] = {
     {"fwd", FWD},
     {"bwd", BWD},
     {"lft", LFT},
     {"rgt", RGT}
};

void displayTerminal(void);

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;
    char c;

    // Get the interrupt status.
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        // Read the next character from the UART.
        c = ROM_UARTCharGet(UART0_BASE);

        // Echo the character back to the UART0.
        ROM_UARTCharPut(UART0_BASE, c);
        ROM_UARTCharPut(UART1_BASE, c);

        displayTerminal();
    }
}

//*****************************************************************************
//
// UART1 interrupt handler.
//
//*****************************************************************************
void UART1_IntHandler(void)
{
    uint32_t ui32Status;
    uint8_t c;

    // Get the interrupt status.
    ui32Status = UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART1_BASE))
    {
        c = UARTCharGet(UART1_BASE);

        //Read next character from the UART and write it back to the UART
        UARTCharPut(UART0_BASE, c);

        displayTerminal();
        inputStr[strlen(inputStr)] = c;
    }
}

//*****************************************************************************
//
// Change LED color based on the character count.
//
//*****************************************************************************
void displayTerminal(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    switch (count % 3)
    {
        case 0:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // Blue
            break;
        case 1:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Red
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // Green
            break;
    }
    // Increment character count.
    count++;
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        ROM_UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// Send a string to UART1.
//
//*****************************************************************************
void UART1_Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        UARTCharPut(UART1_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    // Enable lazy stacking for interrupt handlers.
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Enable the GPIO port that is used for the on-board LED.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LEDs (PF1 to PF3).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Enable the peripherals used by this example.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));
    {
    }

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // Set GPIO pins as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UARTs.
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable UART FIFOs.
    ROM_UARTFIFOEnable(UART0_BASE);
    ROM_UARTFIFOEnable(UART1_BASE);

    // Enable UART interrupts.
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    // Enable UARTs.
    ROM_UARTEnable(UART0_BASE);
    ROM_UARTEnable(UART1_BASE);

    // Set initial LED state to green.
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // Prompt for text to be entered.
    UARTSend((uint8_t *)"Please enter characters in the remote Bluetooth Serial Terminal and see the color change: ", 90);
    UART1_Send((uint8_t *)"Please enter 3-letter commands: ", 32);

    // Loop forever.
    while(1)
    {
        if(strlen(inputStr) == 3){
            inputStr[3] = '\0';
            UARTSend((uint8_t *)"\nCOMMAND IS ", 13);
            UARTSend((uint8_t *)inputStr, 3);

            // Search in the CmdLookup table
            int i;
            bool matched = false;
            for(i = 0; i < sizeof(cmdLookUp) / sizeof(cmdLookUp[0]); i++){
                if(!strncmp(inputStr, cmdLookUp[i].cmd, 3)) {
                    UARTSend((uint8_t *)"\nMatched command: ", 19);
                    UART1_Send((uint8_t *)"\nMatched command: ", 19);
                    cmdLookUp[i].fp();
                    matched = true;
                    break;
                }
            }

            // if command was not found in table, give warning
            if(!matched){
                UARTSend((uint8_t *)"\nWARNING: command not found ", 29);
                UART1_Send((uint8_t *)"\nWARNING: command not found ", 29);
            }

            memset(inputStr, 0, sizeof(inputStr));
        }
    }
}

    void FWD(void){
        UARTSend((uint8_t *)"Move Forward\n", 14);
        UART1_Send((uint8_t *)"Move Forward\n", 14);
    }

    void BWD(void){
        UARTSend((uint8_t *)"Move Backward\n", 15);
        UART1_Send((uint8_t *)"Move Backward\n", 15);
    }

    void LFT(void){
        UARTSend((uint8_t *)"Move Left\n", 11);
        UART1_Send((uint8_t *)"Move Left\n", 11);
    }

    void RGT(void){
        UARTSend((uint8_t *)"Move Right\n", 12);
        UART1_Send((uint8_t *)"Move Right\n", 12);
    }
