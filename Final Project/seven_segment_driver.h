/*
 * seven_segment_driver.h
 *
 *  Created on: Nov 25, 2014
 *      Author: Keith Lueneburg
 */

/*
 * This driver is for the TI Tiva TM4C1294XL development
 * board, using the following pinout. The 7-segment being
 * driven is LSD322X-XX (Jameco PN: 24740)
 *
 * Display is a common anode type, so pins should be driven
 * low to turn on corresponding segment
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

#ifndef SEVEN_SEGMENT_DRIVER_H_
#define SEVEN_SEGMENT_DRIVER_H_

#define A_BASE GPIO_PORTC_BASE
#define A_PIN GPIO_PIN_4

#define B_BASE GPIO_PORTC_BASE
#define B_PIN GPIO_PIN_5

#define C_BASE GPIO_PORTC_BASE
#define C_PIN GPIO_PIN_6

#define D_BASE GPIO_PORTC_BASE
#define D_PIN GPIO_PIN_7

#define E_BASE GPIO_PORTE_BASE
#define E_PIN GPIO_PIN_0

#define F_BASE GPIO_PORTE_BASE
#define F_PIN GPIO_PIN_1

#define G_BASE GPIO_PORTE_BASE
#define G_PIN GPIO_PIN_2

#endif /* SEVEN_SEGMENT_DRIVER_H_ */


void displayDigit(uint8_t digit);
void sevenSegSetup();
