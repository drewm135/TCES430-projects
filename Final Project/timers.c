/*
 * Timer 0 = Real Time Clock
 * Timer 1 = Thermistor Read (200 ms)
 * Timer 2 = Proximity Read  (500 ms)
 * Timer 3 = Light Read      (100 ms)
 */





//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator throught the UART.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//Math Constants
float DENOMINATOR = 0.000001790830963;

int B = 3950;

int VIN = 5;

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32InterruptFlags; //Bit 0 = Thermistor value read
                               //Bit 1 = Proximity value read
                               //Bit 2 = Light value read
                               //Bits 3-31 = Not Used
uint32_t TimerACount;

uint32_t TimerBCount;

uint32_t TimerCCount;

uint32_t TimerDCount;

//*****************************************************************************
//
// Real Time Clock days, hours, minutes, and seconds
//
//*****************************************************************************
uint32_t RTC_Days;

uint32_t RTC_Hours;

uint32_t RTC_Minutes;

uint32_t RTC_Seconds;

//*****************************************************************************
//
// Read Data
//
//*****************************************************************************
uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

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
// The interrupt handler for the real time clock interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
	//Real time clock interrupt, keep track of days, hours, minutes, seconds
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //Increment Timer A Count
    TimerACount++;
    RTC_Seconds = (TimerACount / 10);
    RTC_Minutes = (RTC_Seconds / 60);
    RTC_Hours   = (RTC_Minutes / 60);
    RTC_Days    = RTC_Hours   / 24;

    /*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

    //
    // Update the interrupt status.
    //
    ROM_IntMasterDisable();
    //UARTprintf("\rDays: %d   Hours: %d   Minutes: %d   Seconds: %d", RTC_Days, RTC_Hours % 24, RTC_Minutes % 60, RTC_Seconds % 60);
    //UARTprintf("\rT1: %d  T2: %d  T3: %d  T4: %d", TimerACount, TimerBCount, TimerCCount, TimerDCount);
    ROM_IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt. (temperature)
//
//*****************************************************************************
void
Timer1IntHandler(void)
{


    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Clear ADC interrupt
    //
    ROM_ADCIntClear(ADC0_BASE, 3);

    //Increment Timer A Count
    TimerBCount++;

    /*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

    //
    // Toggle the flag for the temperature timer.
    //
    HWREGBITW(&g_ui32InterruptFlags, 0) = 1;

    //
    // Trigger ADC Conversion
    //
    ROM_ADCProcessorTrigger(ADC0_BASE, 3);

    //
    //Wait for ADC conversion to complete
    //
    while(!ROM_ADCIntStatus(ADC0_BASE, 3, false)) { //Wait for ADC to finish sampling
    }
	ROM_ADCSequenceDataGet(ADC0_BASE, 3, adc_value); //Get data from Sequencer 3
    //
    // Update the interrupt status.
    //
    //ROM_IntMasterDisable();
    //UARTprintf("ADC Value = %d\n", adc_value[0]); //Print out the first (and only) value
    //ROM_IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the third timer interrupt. (proximity)
//
//*****************************************************************************
void
Timer2IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //Increment Timer A Count
    TimerCCount++;

    /*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

    //
    // Toggle the flag for the proximity timer.
    //
    HWREGBITW(&g_ui32InterruptFlags, 1) = 1;

    //
    // Update the interrupt status.
    //
    ROM_IntMasterDisable();
    //UARTprintf("\rT1: %d  T2: %d  T3: %d  T4: %d", TimerACount, TimerBCount, TimerCCount, TimerDCount);
    ROM_IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the fourth timer interrupt. (light)
//
//*****************************************************************************
void
Timer3IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    //Increment Timer A Count
    TimerDCount++;

    /*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

    //
    // Toggle the flag for the light timer.
    //
    HWREGBITW(&g_ui32InterruptFlags, 2) = 1;

    //
    // Update the interrupt status.
    //
    ROM_IntMasterDisable();
    //UARTprintf("\rT1: %d  T2: %d  T3: %d  T4: %d", TimerACount, TimerBCount, TimerCCount, TimerDCount);
    ROM_IntMasterEnable();
}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

void
calcTemp()
{
	//Calculates Temperature reading
	if (HWREGBITW(&g_ui32InterruptFlags, 0)) { //Temp reading is ready
		HWREGBITW(&g_ui32InterruptFlags, 0) = 0; //Reset flag
		//Read temp reading and calc temp
		int adc = adc_value[0]; //Save current temp
		float tempK = (float) B / log((((float) VIN/(adc*3.3 / 4096)) - 1) / DENOMINATOR); //Temp in Kelvin

		float tempF = (tempK - 273.15) * 1.8000 + 32.00;
		UARTprintf("Fahrenheit Temp = %d\n", (int) tempF); //Print out temp in Fahrenheit
	}
}

void
calcProx()
{     //Calculates Proximity reading
	if (HWREGBITW(&g_ui32InterruptFlags, 1)) { //Proximity reading is ready
		HWREGBITW(&g_ui32InterruptFlags, 1) = 0; //Reset flag
		//UARTprintf("Proximity was read!\n");
	}
}

void
calcLight()
{    //Calculates Light reading
	if (HWREGBITW(&g_ui32InterruptFlags, 2)) { //Light reading is ready
		HWREGBITW(&g_ui32InterruptFlags, 2) = 0; //Reset flag
		//UARTprintf("Light was read!\n");
	}
}

void
UARTSendData()
{ //Sends read data through UART
	//TODO

}

void
configureADC()
{

    //
    // Setup ADC Using ROM functions
    //
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Enable ADC0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enable GPIO E
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64); //Configure hardware oversampling to sample 64 times and average
	HWREG(ADC0_BASE + ADC_O_PC) = ADC_PC_SR_125K; //Set ADC speed to 125K

	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //Enable ADC on PE3
	ROM_ADCSequenceDisable(ADC0_BASE, 3); //Disable sequence before configuring it
	ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); //Use sequencer 3 to trigger at all times with a priority of 0 (highest)
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); //Enable sampling using sequencer 3 on CH0 (PE3)

	ROM_ADCSequenceEnable(ADC0_BASE, 3); //Enable the sequencer
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    ROM_FPULazyStackingEnable(); //Enable lazy stacking for faster FPU performance
    ROM_FPUEnable(); //Enable FPU

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();

    UARTprintf("\033[2JFinal Project Timers Example\n");
    UARTprintf("T1: 0  T2: 0 T3: 0 T4:0");

    configureADC();

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Real Time Clock
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //Temperature
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); //Proximity
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); //Light

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Configure the four 32-bit periodic timers.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 10); //RTC       100ms
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 5);  //Temp      200ms
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock / 2);  //Proximity 500ms
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock / 10); //Light     100ms



    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A); //RTC
    ROM_IntEnable(INT_TIMER1A); //Temp
    ROM_IntEnable(INT_TIMER2A); //Proximity
    ROM_IntEnable(INT_TIMER3A); //Light
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //RTC
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //Temp
    ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); //Proximity
    ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT); //Light

    //
    // Enable the timers.
    //
    ROM_TimerEnable(TIMER0_BASE, TIMER_A); //RTC
    ROM_TimerEnable(TIMER1_BASE, TIMER_A); //Temp
    ROM_TimerEnable(TIMER2_BASE, TIMER_A); //Proximity
    ROM_TimerEnable(TIMER3_BASE, TIMER_A); //Light

    TimerACount = 0;
    TimerBCount = 0;
    TimerCCount = 0;
    TimerDCount = 0;

    //
    // Loop forever while the timers run.
    //
    while(1)
    {
    	//Process data (Busy-Wait Loop)
    	//If a flag hasn't been set by the interrupt the calc functions will simply exit
    	//For performance it makes more sense to check flags here instead of at the beginning of the functions
    	calcTemp();     //Calculates Temperature reading
    	calcProx();     //Calculates Proximity reading
    	calcLight();    //Calculates Light reading
    	UARTSendData(); //Sends read data through UART
    }
}
