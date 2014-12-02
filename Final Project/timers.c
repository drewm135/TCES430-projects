//Drew, Thinh, Fahad, Keith

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

// Constants for ultrasonic ranger
#define TIMER4_TAMR_R           (*((volatile uint32_t *)0x40034004))
#define TIMER4_RIS_R            (*((volatile uint32_t *)0x4003401C))
#define TIMER4_TAR_R            (*((volatile uint32_t *)0x40034048))
#define TIMER4_TAV_R            (*((volatile uint32_t *)0x40034050))
#define TIMER4_ICR_R            (*((volatile uint32_t *)0x40034024))

#define SOUND_CM_PER_S 34326

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

// Light input capture declarations
uint32_t Timer;
uint32_t loopCount;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
volatile uint32_t g_ui32InterruptFlags; //Bit 0 = Thermistor value read
                               //Bit 1 = Proximity value read
                               //Bit 2 = Light value read
                               //Bits 3-31 = Not Used

volatile uint32_t g_ui32PrintFlags;     //Bit 0 = Timer ready to print
                               //Bit 1 = Thermister data ready to print
                               //Bit 2 = Proximity data ready to print
                               //Bit 3 = Light data ready to print
uint32_t TimerACount;

uint32_t TimerBCount;

uint32_t TimerCCount;

uint32_t TimerDCount;

//*****************************************************************************
//
// Real Time Clock days, hours, minutes, and seconds
//
//*****************************************************************************
volatile uint32_t RTC_Days;

volatile uint32_t RTC_Hours;

volatile uint32_t RTC_Minutes;

volatile uint32_t RTC_Seconds;

//*****************************************************************************
//
// Read Data
//
//*****************************************************************************
uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

volatile uint32_t g_ui32PulseLengthTicks; // Length of ultrasonic echo pulse in system clock ticks

//*****************************************************************************
//
// Queues to hold converted data before output
//
//*****************************************************************************
float    g_temp_data[50];   //Temperature data

uint32_t g_prox_data[20];   //Proximity sensor data

uint32_t g_light_data[100]; //Light sensor data

//*****************************************************************************
//
// Indexes for data arrays
//
//*****************************************************************************
uint32_t g_temp_index;  //Temp array index

uint32_t g_prox_index;  //Proximity array index

uint32_t g_light_index; //Light array index

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
// Delay for 5 us.
//
//*****************************************************************************
void delayFiveMicroseconds(uint32_t g_ui32SysClock) {
	//
	// Delay for 5 us. The value of the number provided to SysCtlDelay
	// is the number of loops (3 assembly instructions each) to iterate through.
	// Interrupts are disabled temporarily to ensure the pulse length is 5us.
	//
	ROM_IntMasterDisable();

	ROM_SysCtlDelay(g_ui32SysClock / 3 / 200000);

	ROM_IntMasterEnable();
}

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

    HWREGBITW(&g_ui32PrintFlags, 0) = 1;

    /*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

    //
    // Update the interrupt status.
    //

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

	uint32_t ui32PulseStartTime; // Timer value at echo pulse rising edge
	uint32_t ui32PulseStopTime; // Timer value at echo pulse falling edge

	// // // Send a trigger pulse \\ \\ \\

	//
	// Enable the GPIO pin for the trigger pulse (M4).
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);

	//
	// Turn on pulse.
	//
	MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_PIN_4);

	//
	// Delay for 5 us.
	//
	delayFiveMicroseconds(g_ui32SysClock);

	//
	// Turn off pulse.
	//
	MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, 0);


	// // // Measure pulse length \\ \\ \\

	//
	// Clear Timer A capture flag
	//
	TIMER4_ICR_R |= (1 << 2);

	//
	// Enable GPIO pin for timer event capture (M4).
	//
	ROM_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4);

	//
	// Configure PM4 as Timer 4 CCP0
	//
	ROM_GPIOPinConfigure(GPIO_PM4_T4CCP0);

	//
	// Start Timer4 A
	//
	ROM_TimerEnable(TIMER4_BASE, TIMER_A);

	//
	// Wait for first capture event
	//
	while((TIMER4_RIS_R & (0x1 << 2)) == 0){};

	//
	// After first event, save timer A's captured value.
	//
	ui32PulseStartTime = TIMER4_TAR_R;

	//
	// Clear Timer A capture flag
	//
	TIMER4_ICR_R |= (1 << 2);

	//
	// Wait for second capture event
	//
	while((TIMER4_RIS_R & (0x1 << 2)) == 0){};

	//
	// After second event, save timer A's captured value.
	//
	ui32PulseStopTime = TIMER4_TAR_R;

	//
	// Calculate length of the pulse by subtracting the two timer values.
	// Note that even if stop time is less than start time, due to the
	// timer overflowing and starting over at 0, the 24 LSBs of our result are
	// still valid
	//
	g_ui32PulseLengthTicks = (ui32PulseStopTime - ui32PulseStartTime) & 0x0FFFFFF;

	//
	// Print the start time, stop time, and pulse length
	//
	//	UARTprintf("Pulse length: %d - %d = %d\n", ui32PulseStopTime,
	//		ui32PulseStartTime, ui32PulseLengthTicks);

	//
	// Print the distance
	//
	//UARTprintf("Distance: %d\n", ui32DistanceCM);

	//
	// Stop Timer4 A.
	//
	ROM_TimerDisable(TIMER4_BASE, TIMER_A);


	/*//
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);*/

	//
	// Toggle the flag for the proximity timer.
	//
	HWREGBITW(&g_ui32InterruptFlags, 1) = 1;
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



	// Get the timer value and reset it
	ROM_TimerDisable(TIMER5_BASE, TIMER_A);
	// Get the timer value
	Timer = ROM_TimerValueGet(TIMER5_BASE, TIMER_A);
	// Reset the timer value to 65000
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, 65000); // Timer5
	ROM_TimerEnable(TIMER5_BASE, TIMER_A);

    //
    // Toggle the flag for the light timer.
    //
    HWREGBITW(&g_ui32InterruptFlags, 2) = 1;

    //
    // Update the interrupt status.
    //

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
		g_temp_data[g_temp_index] = tempF;
		g_temp_index++; //Increase index
		if (g_temp_index == 50) {
			g_temp_index = 0;
			HWREGBITW(&g_ui32PrintFlags, 1) = 1; //Set print flag
		}
		//UARTprintf("Fahrenheit Temp = %d\n", (int) tempF); //Print out temp in Fahrenheit
	}
}

void
calcProx()
{     //Calculates Proximity reading
	uint32_t ui32DistanceCM; // Sensor output converted to centimeters
	if (HWREGBITW(&g_ui32InterruptFlags, 1)) { //Proximity reading is ready
		HWREGBITW(&g_ui32InterruptFlags, 1) = 0; //Reset flag
		//UARTprintf("Proximity was read!\n");
		//
		// pulse length / system clock = pulse length in seconds
		// pulse length in seconds / (1 / sound speed cm per s) = total wave flight distance
		// total distance / 2 = distance from sensor
		//
		ui32DistanceCM = g_ui32PulseLengthTicks / 2 / (g_ui32SysClock / SOUND_CM_PER_S);
		//UARTprintf("\033[2J\nProx data = %d\n", ui32DistanceCM);

		g_prox_data[g_prox_index] = ui32DistanceCM;
		g_prox_index++; //Increase index
		if (g_prox_index == 20) {
			g_prox_index = 0;
			HWREGBITW(&g_ui32PrintFlags, 2) = 1; //Set print flag
		}
		//Update 7Seg LED

		//TODO


		//UARTprintf("Distance (cm) = %d\n", ui32DistanceCM); //Print out temp in Fahrenheit (For DEBUG)
	}
}

void
calcLight()
{    //Calculates Light reading
	if (HWREGBITW(&g_ui32InterruptFlags, 2)) { //Light reading is ready
		HWREGBITW(&g_ui32InterruptFlags, 2) = 0; //Reset flag
		uint32_t Freq; // The frequency
		Freq = (65000-Timer)*10; // Freq = Counts/second. Multiplied by 10 is the same as divided by 0.1second
		g_light_data[g_light_index] = Freq;
		g_light_index++;
		if (g_light_index == 100) {
			g_light_index = 0;
			HWREGBITW(&g_ui32PrintFlags, 3) = 1; //Set print flag
		}
	}
}

void
UARTSendData()
{ //Sends read data through UART
	//TODO
	int i;
	float avg_temp;
	float min_temp = 5000;
	float max_temp = 0;
	float std_dev_temp;

	float    avg_prox;
	uint32_t min_prox = 5000;
	uint32_t max_prox = 0;
	float    std_dev_prox;

	float    avg_light;
	uint32_t min_light = 500000;
	uint32_t max_light = 0;
	float    std_dev_light;


	if (HWREGBITW(&g_ui32PrintFlags, 0)) { //Clock reading is ready
		HWREGBITW(&g_ui32PrintFlags, 0) = 0; //Reset flag
	    ROM_IntMasterDisable();
		UARTprintf("\033[1;1HDays:Hours:Minutes:Seconds: %02d:%02d:%02d:%02d", RTC_Days, RTC_Hours % 24, RTC_Minutes % 60, RTC_Seconds % 60);
		ROM_IntMasterEnable();
	}
	if (HWREGBITW(&g_ui32PrintFlags, 1)) { //Temp readings are ready
		HWREGBITW(&g_ui32PrintFlags, 1) = 0; //Reset flag
		//Print temp data
		for (i = 0; i < 50; i++) { //Calculate min, max, and mean
			avg_temp += g_temp_data[i];
			min_temp = g_temp_data[i] < min_temp ? g_temp_data[i] : min_temp;
			max_temp = g_temp_data[i] > max_temp ? g_temp_data[i] : max_temp;
		}
		avg_temp = avg_temp / 50;
		for (i = 0; i < 50; i++) { //Calculate standard deviation
			std_dev_temp += (g_temp_data[i] - avg_temp) * (g_temp_data[i] - avg_temp);
		}
		std_dev_temp = std_dev_temp / 50;
		std_dev_temp = sqrt(std_dev_temp);
		//std_dev_temp = std_dev_temp * 100;

		ROM_IntMasterDisable();
		/*UARTprintf("\n\n+------------------------------------------------------------------------+\n");
		UARTprintf(    "|                                 Temperature                            |\n");
		UARTprintf(    "+------------------+-------------------+------------------+--------------+\n");
		UARTprintf(    "|       Min        |        Max        |       Mean       |   Std. Dev.  |\n");
		UARTprintf(    "+------------------+-------------------+------------------+--------------+\n");
		UARTprintf(    "|      %d.%02d       |       %d.%02d       |       %d.%02d      |    %d.%03d     |\n",
				(uint32_t) min_temp,     (uint32_t) ((int) (min_temp     * 100) % (int) min_temp     * 100) / 100,
				(uint32_t) max_temp,     (uint32_t) ((int) (max_temp     * 100) % (int) max_temp     * 100) / 100,
				(uint32_t) avg_temp,     (uint32_t) ((int) (avg_temp     * 100) % (int) avg_temp     * 100) / 100,
				(uint32_t) std_dev_temp, (uint32_t) ((int) (std_dev_temp * 100) % (int) std_dev_temp * 100) / 100);
		UARTprintf(    "+------------------+-------------------+------------------+--------------+");*/



		UARTprintf("\033[3;1H+-----------------+------------------+-------------------+------------------+--------------+\n");
		UARTprintf("|      Sensor     |       Min        |        Max        |       Mean       |   Std. Dev.  |\n");
		UARTprintf("+-----------------+------------------+-------------------+------------------+--------------+\n");

		UARTprintf("Min       Temp = %d.%02d\n", (uint32_t) min_temp,     (uint32_t) ((int) (min_temp     * 100) % (int) min_temp     * 100) / 100);
		UARTprintf("Max       Temp = %d.%02d\n", (uint32_t) max_temp,     (uint32_t) ((int) (max_temp     * 100) % (int) max_temp     * 100) / 100);
		UARTprintf("Mean      Temp = %d.%02d\n", (uint32_t) avg_temp,     (uint32_t) ((int) (avg_temp     * 100) % (int) avg_temp     * 100) / 100);
		UARTprintf("Std. Dev. Temp = %d.%03d"  , (uint32_t) std_dev_temp, (uint32_t) ((int) (std_dev_temp * 100) % (int) std_dev_temp * 100) / 100);

		ROM_IntMasterEnable();


	}
	//ROM_SysCtlDelay(g_ui32SysClock / 3 / 400000); //Delay for 2us
	if (HWREGBITW(&g_ui32PrintFlags, 2)) { //Proximity readings are ready
		HWREGBITW(&g_ui32PrintFlags, 2) = 0; //Reset flag
		//Print proximity data

		for (i = 0; i < 20; i++) { //Calculate min, max, and mean
			avg_prox += g_prox_data[i];
			min_prox = g_prox_data[i] < min_prox ? g_prox_data[i] : min_prox;
			max_prox = g_prox_data[i] > max_prox ? g_prox_data[i] : max_prox;
		}
		avg_prox = (float) avg_prox / 20;
		for (i = 0; i < 20; i++) { //Calculate standard deviation
			std_dev_prox += ((float) g_prox_data[i] - avg_prox) * ((float) g_prox_data[i] - avg_prox);
		}
		std_dev_prox = std_dev_prox / 20;
		std_dev_prox = sqrt(std_dev_prox);

		ROM_IntMasterDisable();
		UARTprintf("\033[11;1H\033[0JMin       Prox = %d\n", min_prox);
		UARTprintf("Max       Prox = %d\n", max_prox);
		UARTprintf("Mean      Prox = %d.%02d\n", (uint32_t) avg_prox, (uint32_t) ((int) (avg_prox * 100) % (int) avg_prox * 100) / 100);
		UARTprintf("Std. Dev. Prox = %d.%03d"  , (uint32_t) std_dev_prox, (uint32_t) ((int) (std_dev_prox * 100) % (int) std_dev_prox * 100) / 100);
		ROM_IntMasterEnable();
	}
	if (HWREGBITW(&g_ui32PrintFlags, 3)) { //light readings are ready
		HWREGBITW(&g_ui32PrintFlags, 3) = 0; //Reset flag
		//Print light data
		for (i = 0; i < 100; i++) { //Calculate min, max, and mean
					avg_light += g_light_data[i];
					min_light = g_light_data[i] < min_light ? g_light_data[i] : min_light;
					max_light = g_light_data[i] > max_light ? g_light_data[i] : max_light;
		}
		avg_light = (float) avg_light / 100;
		for (i = 0; i < 100; i++) { //Calculate standard deviation
			std_dev_light += ((float) g_light_data[i] - avg_light) * ((float) g_light_data[i] - avg_light);
		}
		std_dev_light = std_dev_light / 100;
		std_dev_light = sqrt(std_dev_light);

		ROM_IntMasterDisable();
		UARTprintf("\033[16;1H\033[0JMin       Light = %d\n", min_light);
		UARTprintf("Max       Light = %d\n", max_light);
		UARTprintf("Mean      Light = %d.%02d\n", (uint32_t) avg_light, (uint32_t) ((int) (avg_light * 100) % (int) avg_light * 100) / 100);
		UARTprintf("Std. Dev. Light = %d.%03d"  , (uint32_t) std_dev_light, (uint32_t) ((int) (std_dev_light * 100) % (int) std_dev_light * 100) / 100);
		ROM_IntMasterEnable();
	}


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
// Configure the timer and its pins for measuring the length of
// ultrasonic sensor echo pulse.
//
//*****************************************************************************
void ConfigureDistancePulseTimer()
{
	//
	// Enable Timer 4
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);

	//
	// Configure timer 4A as a 16-bit event capture up-counter
	//
	ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

	//
	// Set prescaler to 255. This essentially makes
	// the 16-bit timer a 24-bit timer.
	//
	ROM_TimerPrescaleSet(TIMER4_BASE, TIMER_A, 0xFF);

	//
	// The timer should capture events on both rising and falling edges
	//
	ROM_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

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



	// Setup for ultrasonic ranger

	//
	// Enable GPIO M
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	//
	// Enable GPIO pin for timer event capture (M4).
	//
	ROM_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4);

	//
	// Initialize timer for distance pulse measurement.
	//
	ConfigureDistancePulseTimer();

    ROM_FPULazyStackingEnable(); //Enable lazy stacking for faster FPU performance
    ROM_FPUEnable(); //Enable FPU


    TimerACount = 0;
    TimerBCount = 0;
    TimerCCount = 0;
    TimerDCount = 0;

    g_temp_index = 0;
    g_prox_index = 0;
    g_light_index = 0;

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();
    UARTprintf("\033[2J"); //Clear screen

    //UARTprintf("\033[2JFinal Project Timers Example\n");

    configureADC();

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Real Time Clock
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //Temperature
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); //Proximity
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); //Light

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5); //Timer5 is used to measure the leading edge pulses from the signal generator

	// Enable the A peripheral used by the Timer3 pin PA6, PA7
	// Enable the B peripheral used by the Timer5 pin PB2, PB3
	//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enable GPIO A
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Enable GPIO B

	//Configure the pins for its Timer functionality
	ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2); //Enable Timer5 on PB2

	//Configures the alternate function of a GPIO pin for Timer5
	ROM_GPIOPinConfigure(GPIO_PB2_T5CCP0);

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
    ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT); // Timer5

    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 10); //RTC       100ms
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 5);  //Temp      200ms
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock / 2);  //Proximity 500ms
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock / 10); //Light     100ms
    ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, 65000); // Timer5

    //Configure the signal edges that triggers the timer when in capture mode
    ROM_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

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
    ROM_TimerEnable(TIMER5_BASE, TIMER_A); //Timer5

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
