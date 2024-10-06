/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Standard C libraries */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Driverlib and hardware specific includes */
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "utils/uartstdio.h"

extern ti_sysbios_knl_Semaphore_Handle Task0Semaphore;
volatile uint64_t startCount;
volatile uint64_t endCount;

int lineCounter = 0;  // Keeps track of the number of thin lines detected

// Function Prototypes
void EnablePeripherals();
void InitializeUART0LocalTerminal();
void InitializeTimer1();
void GPIOAIntHandler();
void Task0Func();
void StopRobot();

// Set the PWM duty cycle to 0 to stop the robot
void StopRobot() {
    // Assuming PWM module is set up, you would set the duty cycle to 0 like this:
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);  // Stop PWM output
}

// Interrupt handler for Timer1
void WTimer1IntHandler() {
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    UARTprintf("Timer Interrupt Triggered\n");
    Semaphore_post(Task0Semaphore);
}

// Interrupt handler for GPIOA (Reflectance Sensor Input)
void GPIOAIntHandler() {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);  // Clear interrupt flag
    endCount = TimerValueGet(WTIMER0_BASE, TIMER_A);  // Capture end timer value

    uint64_t elapsedCount = endCount - startCount;
    UARTprintf("Decay Count Ticks: %i\n", (int)elapsedCount);

    // Use elapsed time to determine whether the sensor detected a black line (thin or thick)
    if (elapsedCount > 40000) {
        UARTprintf("On a dark surface\n");

        // Check if it's a thick black line (stop the robot)
        if (elapsedCount > 60000) {  // Threshold for thick black line (adjust if necessary)
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);  // Turn on RED LED
            StopRobot();  // Stop the robot
        } else {  // Thin black line detected
            lineCounter++;
            if (lineCounter == 1) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);  // Turn on GREEN LED
            } else if (lineCounter == 2) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x04);  // Turn on BLUE LED
            }
        }

    } else {
        UARTprintf("On a white surface\n");
    }
}

// Main Task function
void Task0Func() {
    while (1) {
        Semaphore_pend(Task0Semaphore, BIOS_WAIT_FOREVER);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);  // Set PA4 as output
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);  // Write high
        SysCtlDelay(SysCtlClockGet() / 3 * 10 * 1.0 / 1000000);  // Delay
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);  // Set PA4 as input
        startCount = TimerValueGet(WTIMER0_BASE, TIMER_A);  // Capture start timer value
    }
}

// Enable necessary peripherals
void EnablePeripherals() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // Enable Port F for LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
}

// Initialize UART0 for local terminal
void InitializeUART0LocalTerminal() {
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}

// Initialize Timer 0
void InitializeTimer0() {
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

// Initialize Timer 1
void InitializeTimer1() {
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5);  // Adjust as necessary
    TimerEnable(WTIMER1_BASE, TIMER_A);
}

// Initialize GPIO A interrupt for the reflectance sensor
void InitializeGPIOAInterrupt() {
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
}

// Main function
int main(void) {
    // Run system clock at 40 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    IntMasterEnable();

    EnablePeripherals();
    InitializeUART0LocalTerminal();
    InitializeTimer0();
    InitializeTimer1();
    InitializeGPIOAInterrupt();

    // Initialize the LEDs on Port F
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // PF1 = RED, PF2 = BLUE, PF3 = GREEN

    // Start initial timer counts
    startCount = TimerValueGet(WTIMER0_BASE, TIMER_A);
    SysCtlDelay(SysCtlClockGet() / 3 * 4);
    endCount = TimerValueGet(WTIMER0_BASE, TIMER_A);

    uint64_t elapsedCount = endCount - startCount;
    UARTprintf("Main Program is running!\n");

    // Start BIOS
    BIOS_start();

    return (0);
}