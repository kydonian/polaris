/*
 * control.h
 *
 *  Header file of ADC data control routine.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "typedefs.h"
#include "storage.h"
#include "math.h"


/** Variables specific to control methods **/
typedef struct
{
	volatile bool	tempEstimationIsInitialized;	// True if temperature estimation has been initialized by controlInitTemperatureEstimation().
	volatile bool	plowConnected;			// Plow connected to winch?
	volatile bool	isOvercurrent;			// Overcurrent detected
	volatile bool	isHot;					// The winch is currently too hot to operate
	volatile uint8	opMode;					// Operation mode of type operationMode_t
	volatile uint8	LCDMode;				// Display mode of LCD array from LCDMode_t
	volatile uint16	counter;				// Counter to regulate main interrupt routine
	volatile uint16 counterMax;				// Value at which 'counter' loops
	volatile uint16	ADC12MinReadingCurrent;	// Minimum ADC12 reading for current sensor that will be accepted in ohmic calcs
											// (done to avoid 'heating' from mere noise)
	volatile uint16	ADC12OvercurrentThreshold;	// ADC12 reading corresponding to the overcurrent threshold
	volatile uint32	absoluteCounter;		// Absolute counter, increments from startup at freq of basic_timer_ISR
	volatile double	OAT;					// Outside air temperature
	volatile double	motorTemp;				// Estimated temperature of motor windings
	volatile double motorCurrent;			// Motor current read in from sensors
	volatile double motorVoltage;			// Motor voltage read in from sensors
} controlMemory_t;
// Total RAM allocated: 2310 bytes


/** Definitions **/

// Modes based on external selector switch
typedef enum operationMode
{
	CONTROL_OPERATION_MODE_2500 = 0,
	CONTROL_OPERATION_MODE_3500,
	CONTROL_OPERATION_MODE_4500,
	CONTROL_OPERATION_MODE_OFF,
	CONTROL_OPERATION_MODE_NUM
} operationMode_t;

// LCD display modes
typedef enum LCDDisplayMode
{
	CONTROL_LCD_DISPLAY_MODE_MOTOR_TEMP = 0,
	CONTROL_LCD_DISPLAY_MODE_OAT,
	CONTROL_LCD_DISPLAY_MODE_CURRENT,
	CONTROL_LCD_DISPLAY_MODE_VOLTAGE,
	CONTROL_LCD_DISPLAY_MODE_NUM
} LCDDisplayMode_t;


#define CONTROL_WDT_WRONG_PASSWORD			(0xFF00)

// Signals to controller to flip the ADC reading as it comes in.  ADC12 readings go from
// 0 to 4095 .  If flipped:
// 0 -> 4095
// 4095 -> 0
#define CONTROL_FLIP_CURRENT_ADC			1

// To read, <<  myBool = CONTROL_READ_... >>

// Input ports & pins
#define CONTROL_READ_PLOW_CONNECTED			(P3IN & BIT2)
#define CONTROL_READ_OVERRIDE_ENGAGED		(P3IN & BIT4)
#define CONTROL_READ_USER_CONTROL_SWITCH	(P3IN & BIT7)
#define CONTROL_READ_SWITCH_1				(P1IN & BIT0)
#define CONTROL_READ_SWITCH_2				(P1IN & BIT1)
#define CONTROL_READ_SELECTOR_0				(P6IN & BIT0)
#define CONTROL_READ_SELECTOR_1				(P6IN & BIT1)
#define CONTROL_READ_SELECTOR_2				(P6IN & BIT2)


// Define external selector positions
#define CONTROL_READ_SELECTOR_2500		 	CONTROL_READ_SELECTOR_0
#define CONTROL_READ_SELECTOR_3500		 	CONTROL_READ_SELECTOR_1
#define CONTROL_READ_SELECTOR_4500		 	CONTROL_READ_SELECTOR_2


// To write output = 1, CONTROL_PORT_... |= CONTROL_PIN_...
// To write output = 0, CONTROL_PORT_... &= ~CONTROL_PIN_...

// Output ports
#define CONTROL_PORT_ENABLE_WINCH			P3OUT
#define CONTROL_PORT_BEEPER					P3OUT
#define CONTROL_PORT_SYSTEM_LED				P2OUT
#define CONTROL_PORT_LED0					P3OUT
#define CONTROL_PORT_LED1					P3OUT
#define CONTROL_PORT_LED2					P2OUT
#define CONTROL_PORT_LED3					P3OUT
#define CONTROL_PORT_LED4					P2OUT
#define CONTROL_PORT_LED5					P2OUT
#define CONTROL_PORT_LED6					P2OUT
#define CONTROL_PORT_LED7					P2OUT
#define CONTROL_PORT_LED8					P2OUT
#define CONTROL_PORT_LED9					P2OUT

// Output pins
#define CONTROL_PIN_ENABLE_WINCH			BIT3
#define CONTROL_PIN_BEEPER					BIT5
#define CONTROL_PIN_SYSTEM_LED				BIT1
#define CONTROL_PIN_LED0					BIT0
#define CONTROL_PIN_LED1					BIT1
#define CONTROL_PIN_LED2					BIT0
#define CONTROL_PIN_LED3					BIT6
#define CONTROL_PIN_LED4					BIT2
#define CONTROL_PIN_LED5					BIT3
#define CONTROL_PIN_LED6					BIT4
#define CONTROL_PIN_LED7					BIT5
#define CONTROL_PIN_LED8					BIT6
#define CONTROL_PIN_LED9					BIT7


// Specific numbers
#define CONTROL_INTERRUPT_CYCLES_INRUSH				41		// 0.08 sec at 512 Hz
#define CONTROL_OVERCURRENT_COUNT_MAX				39		// 95% of the length of CONTROL_INTERRUPT_CYCLES_INRUSH
#define CONTROL_OVERCURRENT_THRESHOLD				25		// (A) Limit current in plow mode to <var>.
#define CONTROL_MINIMUM_OVERCURRENT_DISABLE_TIME	0.25	// (sec) Minimum time to disable winch upon overcurrent


// LED array thresholds
#define CONTROL_LED1_THRESHOLD				0.111		// LED1 will light up when the estimated temperature is
														// 11.1% of the way between the OAT and the motor critical
														// temperature
#define CONTROL_LED2_THRESHOLD				0.222
#define CONTROL_LED3_THRESHOLD				0.333
#define CONTROL_LED4_THRESHOLD				0.444
#define CONTROL_LED5_THRESHOLD				0.555
#define CONTROL_LED6_THRESHOLD				0.666
#define CONTROL_LED7_THRESHOLD				0.777
#define CONTROL_LED8_THRESHOLD				0.888





/** Public functions **/

// Initializes control variables
void controlInit( controlMemory_t* pMemory );

// Interprets outside air temperature as the average of all values in the
// temperature array, which, at 512Hz sample and 128 length temperature array,
// equates to 0.25 sec average.  Stores the averaged OAT in the controlMemory
// struct supplied.
void controlUpdateOAT( 	controlMemory_t* pControlMemory,
						storageMemory_t* pStorageMemory );

// Check if plow is connected.
void controlUpdatePlow( controlMemory_t* pControlMemory );

// Enables or disables user winch control.
void controlEnableWinch( 	controlMemory_t* pControlMemory,
							storageMemory_t* pStorageMemory );

// Updates item in control block that indicates overcurrent
void controlUpdateOvercurrent( 	controlMemory_t* pControlMemory,
								storageMemory_t* pStorageMemory );

// Updates item in control block that indicates that winch is too hot
void controlUpdateIsHot( 	controlMemory_t* pControlMemory,
							storageMemory_t* pStorageMemory );

/* Toggle system monitoring LED
 * counter		Counter in main ISR.  Nominally, counts to max once per second
 * counterMax	Maximum value that counter obtains.
 */
void controlUpdateSystemLED( controlMemory_t* pControlMemory );

// Update LED output array
void controlUpdateLEDArray( controlMemory_t* pControlMemory );

// Update motor winding temperature estimate
void controlUpdateMotorTemp( 	controlMemory_t* pControlMemory,
								storageMemory_t* pStorageMemory );

// Update LCD output
void controlUpdateLCD( controlMemory_t* pControlMemory );

// Perform software reset if indicated
void controlCheckForReset( void );

/*
 * Cycle LCD display mode if switch 2 is pressed.  Cycle through displaying OAT,
 * estimated motor temp, current, and voltage.
 */
void controlCheckForLCDCycle( controlMemory_t* pControlMemory );

// Update operation mode based on external selector position.  This function
// operates on the inputs on the pins specified by CONTROL_READ_SELECTOR_xxxx.
// If the pin corresponding to a particular winch type is high, the winch will
// enter the mode of operation corresponding to that winch type.  Otherwise
// (if no pins are high), the winch controller enters 'off' mode and does nothing.
void controlUpdateSelector( controlMemory_t* pControlMemory );

// Initialize control estimation parameters.  Only to be run once when ADC
// has been running long enough to correctly initialize temperature estimation
// params.
void controlInitTemperatureEstimation( controlMemory_t* pControlMemory );

// Update the motor temperature with respect to temperature decay for a given
// period of time dt.
void controlDecayEstimatedTemp( controlMemory_t* pControlMemory, double dt );

// Update the motor temperature with respect to motor use.  That is, when the
// motor is running, calculate the temperature increase.  Only factor in current,
// not voltage (assumed constant coil resistance).
void controlRaiseEstimatedTempCurrentOnly( 	controlMemory_t* pControlMemory,
											storageMemory_t* pStorageMemory,
											uint16 numReadings );

/* Update the motor temperature with respect to motor use.  That is, when the
 * motor is running, calculate the temperature increase.  Factor in total power,
* not just current (energy defect).
* pControlMemory	Pointer to control memory block
* pStorageMemory	Pointer to storage memory block
* numReadings		The number of current, voltage readings to be factored into
* 					the temperature rise calculation.  This should be the number \
* 					of readings since this function was last run.
* readingFreq		Frequency with which current / voltage readings are taken.
* 					Nominally equal to MAIN_SELECTED_BT_FREQUENCY.
*/
void controlRaiseEstimatedTempPower(	controlMemory_t* pControlMemory,
										storageMemory_t* pStorageMemory,
										uint16 numReadings,
										uint16 readingFreq);

/* Update the motor temperature with respect to motor use.  That is, when the
 * motor is running, calculate the temperature increase.  Calculate by correlating
 * a specific total power over a period of time into a temperature rise (not
 * energy defect!  Based on empirical power vs. temperature rise measurements only).
 * pControlMemory	Pointer to control memory block
 * pStorageMemory	Pointer to storage memory block
 * numReadings		The number of current, voltage readings to be factored into
 * 					the temperature rise calculation.  This should be the number \
 * 					of readings since this function was last run.
 * readingFreq		Frequency with which current / voltage readings are taken.
 * 					Nominally equal to MAIN_SELECTED_BT_FREQUENCY.
 */
void controlRaiseEstimatedTempPowerInferred(controlMemory_t* pControlMemory,
											storageMemory_t* pStorageMemory,
											uint16 numReadings,
											uint16 readingFreq);

/*
 * Update instantaneous current & voltage readings stored in a given control struct
 *  with the most recently read values from sensors.
 */
void controlUpdateCurrentAndVoltage(controlMemory_t* pControlMemory,
									storageMemory_t* pStorageMemory);

#endif /* __CONTROL_H__ */
