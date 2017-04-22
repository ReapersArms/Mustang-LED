//*****************************************************************************
//
// project.c - Simple project to use as a starting point for more complex
//             projects.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
// from uart echo
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
// from qs-rgb
#include "utils/uartstdio.h"
#include "utils/cmdline.h"



// Project Includes
//#include "m_uart.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Simple Project (project)</h1>
//!
//! A very simple example that can be used as a starting point for more complex
//! projects.  Most notably, this project is fully TI BSD licensed, so any and
//! all of the code (including the startup code) can be used as allowed by that55
//! license.
//!
//! The provided code simply toggles a GPIO using the Tiva Peripheral Driver
//! Library.
//
//*****************************************************************************

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




//*****************************************************************************
//
// Function to cycle LEDs
//
//*****************************************************************************

void cycle_leds(void) {
	int i = 0;
	for (i=0; i<3; i++) {
		switch (i) {
		case 0:
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
			break;
		case 1:
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
			break;
		case 2:
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
			break;
		}

		ROM_SysCtlDelay(1000000);

        // Write all pins low just for easy code
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

        // Delay for a while.
        ROM_SysCtlDelay(1000000);
	}
}

//*****************************************************************************
//
// Hardware Initialization
//
//*****************************************************************************
void hardwareInit(void) {
    //
    // Enable the GPIO module.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlDelay(1);

    //
    // Configure PF1 as an output.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

//*****************************************************************************
//
// UART Configuration
//
//*****************************************************************************
void uartInit(void) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);

	UARTprintf("Mr. Hammond, the UART is working.");
}

//*****************************************************************************
//
// UART to Command Line interface
//
//*****************************************************************************
void CmdLineFxn(void){
    UARTprintf("\n>");

    //
    // Peek to see if a full command is ready for processing
    //
    while(UARTPeek('\r') == -1)
    {
        //
        // millisecond delay.  A SysCtlSleep() here would also be OK.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 / 3));

        //
        // Check for change of mode and enter hibernate if requested.
        // all other mode changes handled in interrupt context.
        //
//        if(g_sAppState.ui32Mode == APP_MODE_HIB)
//        {
//            AppHibernateEnter();
//        }
    }
}

//*****************************************************************************
//
// Input buffer for the command line interpreter.
//
//*****************************************************************************
#define APP_INPUT_BUF_SIZE 128
static char g_cInput[APP_INPUT_BUF_SIZE];

//*****************************************************************************
//
// Main Application Entrance
//
//*****************************************************************************
int
main(void)

{
	hardwareInit();

	uartInit();

    // Loop forever.
    while(1)
    {
    	// Check for a user return
        while(UARTPeek('\r') == -1) {

        }

        //
        // a '\r' was detected get the line of text from the user.
        //
        UARTgets(g_cInput,sizeof(g_cInput));
        UARTprintf("\nMaybe it's the power trying to come back on...");

        // Cycle LEDs
    	if (1) {
    		cycle_leds();
    	}
    }
}
