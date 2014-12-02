/*
 * seven_segment_driver.c
 *
 *  Created on: Nov 25, 2014
 *      Author: Keith Lueneburg
 */

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "seven_segment_driver.h"

/*
 * This driver is for the TI Tiva TM4C1294XL development
 * board, using the following pinout. The 7-segment being
 * driven is LSD322X-XX (Jameco PN: 24740)
 *
 *      __a__
 *     |     |
 *    f|     |b
 *     |__g__|
 *     |     |
 *    e|     |c
 *     |_____|
 *        d
 *
 *  Segment:        GPIO Pin:
 *     a               C4
 *     b               C5
 *     c               C6
 *     d               C7
 *     e               E0
 *     f               E1
 *     g               E2
 *
 */

void displayDigit(uint8_t digit)
{
	switch (digit)
	{

	case 0:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, G_PIN);
		break;


	case 1:
		ROM_GPIOPinWrite(A_BASE, A_PIN, A_PIN);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, D_PIN);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, G_PIN);
		break;

	case 2:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, C_PIN);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 3:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 4:
		ROM_GPIOPinWrite(A_BASE, A_PIN, A_PIN);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, D_PIN);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 5:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 6:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 7:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, D_PIN);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, G_PIN);
		break;

	case 8:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 9:
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, E_PIN);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 10: // 0xA
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, D_PIN);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 11: // 0xb
		ROM_GPIOPinWrite(A_BASE, A_PIN, A_PIN);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 12: // 0xc
		ROM_GPIOPinWrite(A_BASE, A_PIN, A_PIN);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, C_PIN);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 13: // 0xd
		ROM_GPIOPinWrite(A_BASE, A_PIN, A_PIN);
		ROM_GPIOPinWrite(B_BASE, B_PIN, 0);
		ROM_GPIOPinWrite(C_BASE, C_PIN, 0);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, F_PIN);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 14: // 0xE
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, C_PIN);
		ROM_GPIOPinWrite(D_BASE, D_PIN, 0);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;

	case 15: // 0xF
		ROM_GPIOPinWrite(A_BASE, A_PIN, 0);
		ROM_GPIOPinWrite(B_BASE, B_PIN, B_PIN);
		ROM_GPIOPinWrite(C_BASE, C_PIN, C_PIN);
		ROM_GPIOPinWrite(D_BASE, D_PIN, D_PIN);

		ROM_GPIOPinWrite(E_BASE, E_PIN, 0);
		ROM_GPIOPinWrite(F_BASE, F_PIN, 0);
		ROM_GPIOPinWrite(G_BASE, G_PIN, 0);
		break;
	}
}

void sevenSegSetup()
{
	//
	// Enable the GPIO module.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	ROM_SysCtlDelay(2); // Delay to ensure peripherals are setup

	//
	// Configure pins as outputs
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);

	//
	// Initialize all pins to 0
	//
	ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
	ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
	ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
	ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

	ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
}


