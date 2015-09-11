/*
 * 	control.c
 *
 *  Implementation of winch control algorithms.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#include <msp430xG46x.h>
#include <math.h>

#include "main.h"
#include "typedefs.h"
#include "control.h"
#include "string.h"
#include "LCD.h"
#include "tables.h"


/****** Private function prototypes *******/

/* Description		Converts an ADC12 reading into a voltage
 * ADC12Reading		ADC12 reading to be converted
 * return			THe converted voltage value based on ADC parameters.
 */
double controlADC2Voltage( uint16 ADC12Reading );

/* Description		Translates a winch current into an estimate power out.
 * current			Steady-state current that winch draws.
 * return			Power out of the winch (work transmitted to the cable) if winch
 * 					is at steady-state drawing <current> amps.
 */
double controlCalculatePowerOut( double current );

/*
 * Description		Calculate temperature rise in a given time from the energy into
 * 					the winch in that given time.  This energy is not calculated
 * 					based on equations, but empirical data only.  In testing, it was
 * 					seen that a given power in corresponded consistently to a given
 * 					temperature rise, and this relationship is exploited here.
 * energy			The energy in in a given time.
 * return			The temperature rise in that given time.
 */
double controlCalculateTempRiseFromEnergyIn( double energy );


/****** Public function implementations ******/

/** Implementation of Public Functions **/
void controlInit( controlMemory_t* pMemory )

{
	double voltage;
	uint16 ADC12Reading;

	// bool's
	pMemory->tempEstimationIsInitialized = FALSE;
	pMemory->plowConnected = TRUE;
	pMemory->isOvercurrent = FALSE;
	pMemory->isHot = FALSE;

	// uint8's
	pMemory->opMode = CONTROL_OPERATION_MODE_OFF;
	pMemory->LCDMode = CONTROL_LCD_DISPLAY_MODE_MOTOR_TEMP;

	// uint16's
	pMemory->counter = 0;
	pMemory->counterMax = 0;

	// Calculate minimum ADC12 current reading
	// Voltage corresponding to minimum acceptable current
	voltage = MAIN_CURRENT_SENSOR_V0 + MAIN_CURRENT_SENSOR_SLOPE
			* MAIN_CURRENT_MINIMUM_READING;
	// ADC12 reading corresponding to vMin
	ADC12Reading = (uint16)( (voltage - MAIN_ADC_VREF_MINUS) /
			(MAIN_ADC_VREF_PLUS - MAIN_ADC_VREF_MINUS) * MAIN_ADC_RES );
	pMemory->ADC12MinReadingCurrent = ADC12Reading;

	// ADC12 reading corresponding to winch overcurrent threshold
	// Voltage corresponding to current threshold
	voltage = MAIN_CURRENT_SENSOR_V0 + MAIN_CURRENT_SENSOR_SLOPE
			* CONTROL_OVERCURRENT_THRESHOLD;
	ADC12Reading = (uint16)( (voltage - MAIN_ADC_VREF_MINUS) /
				(MAIN_ADC_VREF_PLUS - MAIN_ADC_VREF_MINUS) * MAIN_ADC_RES );
	pMemory->ADC12OvercurrentThreshold = ADC12Reading;

	// uint32's
	pMemory->absoluteCounter = 0x00000000;

	// double's
	pMemory->OAT = 0;
	pMemory->motorTemp = 0;
	pMemory->motorCurrent = 0;
	pMemory->motorVoltage = 0;
}

void controlUpdateOAT( 	controlMemory_t* pControlMemory,
						storageMemory_t* pStorageMemory )
{
	uint16 iter;
	uint32 sum;
	uint16 average;
	uint16 count;
	double voltage;
	double degC;

	// Average all nonzero values store in temperature array.  Nonzeo
	// to avoid summing non-measured values at beginning of measurement routine.
	sum = 0;
	count = 0;
	for( iter=0; iter<STORAGE_TEMPERATURE_LENGTH; iter++)
	{
		if( pStorageMemory->storageTemp[iter] > 0)
		{
			sum += (uint32)pStorageMemory->storageTemp[iter];
			count++;
		}
	}
	average = (uint16) ( sum / count );

	// Convert ADC integer to voltage amount
	voltage = controlADC2Voltage( average );

	// Convert voltage to deg C
	degC = (voltage - MAIN_THERMOCOUPLE_V0) / MAIN_THERMOCOUPLE_SLOPE;

	// Store in control struct
	pControlMemory->OAT = degC;


}

void controlUpdatePlow( controlMemory_t* pControlMemory )
{
	// Checks if plow is connected, sets appopriate value in control memory block.

	if( CONTROL_READ_PLOW_CONNECTED || MAIN_PLOW_ALWAYS_CONNECTED )
	{
		pControlMemory->plowConnected = TRUE;
	}
	else
	{
		pControlMemory->plowConnected = FALSE;
	}


}


void controlEnableWinch( 	controlMemory_t* pControlMemory,
							storageMemory_t* pStorageMemory )
{
	static uint16 overcurrentCounter = 0;
	static uint16 minimumOvercurrentCycles = (uint16) (CONTROL_MINIMUM_OVERCURRENT_DISABLE_TIME*MAIN_SELECTED_BT_FREQUENCY);

	/*** OVERCURRENT SECTION ***/
	// If overcurrented, wait for a specified time CONTROL_MINIMUM_OVERCURRENT_DISABLE_TIME to
	// de-signal overcurrent.
	if( pControlMemory->isOvercurrent )
	{
		if( overcurrentCounter < minimumOvercurrentCycles )
			overcurrentCounter++;

		if( (overcurrentCounter >= minimumOvercurrentCycles) && (CONTROL_READ_USER_CONTROL_SWITCH != FALSE) )
		{
			pControlMemory->isOvercurrent == FALSE;
			overcurrentCounter = 0;
		}
	}

	// If not overcurrented, check for overcurrent if plow connected.
	else
	{
		// If plow connected, check for overcurrent.
		if( pControlMemory->plowConnected)
		{
			controlUpdateOvercurrent( pControlMemory, pStorageMemory );
		}

		// else
		// If plow not connected, pControlMemory->isOvercurrent will always be false.
	}

	/*** TEMPERATURE SECTION ***/

	// Always check temperature
	controlUpdateIsHot( pControlMemory, pStorageMemory );



	/*** CHECK AND ENABLE / DISABLE SECTION ***/

	// Check if winch is either overcurrent or too hot.  If either, cut power.
	// Otherwise, enable winch power.
 	if( pControlMemory->isOvercurrent || pControlMemory->isHot)
		CONTROL_PORT_ENABLE_WINCH &= ~CONTROL_PIN_ENABLE_WINCH;		// Drive pin low
	else
		CONTROL_PORT_ENABLE_WINCH |= CONTROL_PIN_ENABLE_WINCH;		// Drive pin high
}

void controlUpdateOvercurrent( 	controlMemory_t* pControlMemory,
								storageMemory_t* pStorageMemory )
{
	uint16 iter;
	uint8  overcurrentCount;
	uint16 ADC12OvercurrentThreshold;
	int16 readIdx;

	// Initialize locals
	ADC12OvercurrentThreshold = pControlMemory->ADC12OvercurrentThreshold;

	// If isOvercurrent is TRUE, assume that it continues TRUE.
	// (NOTE: this prevents the winch from reenabling before the user
	// has depressed the control switch).
	if( pControlMemory->isOvercurrent == FALSE )
	{
		// Check how many of recent ADC measurements have measured over
		// the maximum allowed current.
		overcurrentCount = 0;
		for(iter = 0; iter<CONTROL_INTERRUPT_CYCLES_INRUSH;  iter++ )
		{
			// Read backwards through storage array, accounting for wrapping
			readIdx = pStorageMemory->currIdx - iter;
			readIdx += STORAGE_CURRENT_LENGTH;
			readIdx %= STORAGE_CURRENT_LENGTH;

			if( pStorageMemory->storageCurr[readIdx] > ADC12OvercurrentThreshold)
			{
				overcurrentCount++;
			}
		}

		// If enough have measured over the
		// maximum value to indicate a steady-state high current (not
		// just inrush), cut power to the winch.
		if( overcurrentCount >= CONTROL_OVERCURRENT_COUNT_MAX )
			pControlMemory->isOvercurrent = TRUE;

	}
}

void controlUpdateIsHot( 	controlMemory_t* pControlMemory,
							storageMemory_t* pStorageMemory )
{
	// SUMMARY: Effectively a state machine that switches back and forth
	// between motor hot and motor not hot.

	// If the motor is hot, only say it's not hot once it's dropped
	// below the threshold temperature.
	if( pControlMemory->isHot )
	{
		if( pControlMemory->motorTemp < MAIN_THRESHOLD_TEMPERATURE )
			pControlMemory->isHot = FALSE;
	}

	// If the motor is not hot, only say it's hot once it's exceeded
	// the critical temperature.
	else
	{
		if( pControlMemory->motorTemp > MAIN_CRITICAL_TEMPERATURE )
					pControlMemory->isHot = TRUE;
	}
}

void controlUpdateSystemLED( controlMemory_t* pControlMemory )
{
	// LED will be on while variable <counter> is less than onLimit, off while
	// <counter> is greater than onLimit.  Once the counter reaches offLimit, it'll
	// reset to zero locally ( variable <counter> ) but will not be changed within
	// the control memory struct.  That way, the blinker can be made to blink more
	// than once per second, as specified by MAIN_SYSTEM_LED_BLINK_PER_SEC.  Remember
	// that pControlMemory->counter resets at 1 Hz and counts at
	// MAIN_SELECTED_BT_FREQUENCY Hz.

	static uint16 onLimit = (uint16) (MAIN_SYSTEM_LED_BLINK_TIME * MAIN_SELECTED_BT_FREQUENCY);
	static uint16 offLimit = (uint16) (MAIN_SELECTED_BT_FREQUENCY / MAIN_SYSTEM_LED_BLINK_PER_SEC);

	uint16 counter = pControlMemory->counter % offLimit;

	// If the main ISR counter is within the first fraction
	// MAIN_SYSTEM_LED_BLINK_TIME of the main ISR counter cycle, turn
	// the LED on.  Otherwise, off.
	CONTROL_PORT_SYSTEM_LED =
		(counter < onLimit)									?	// logical test
		(CONTROL_PORT_SYSTEM_LED | CONTROL_PIN_SYSTEM_LED) 	:	// if true
		(CONTROL_PORT_SYSTEM_LED & ~CONTROL_PIN_SYSTEM_LED)	;	// if false

	// HACK
	// Blink LED0 on LED array with the system LED
	CONTROL_PORT_LED0 =
		(counter < onLimit)						?	// logical test
		(CONTROL_PORT_LED0 | CONTROL_PIN_LED0) 	:	// if true
		(CONTROL_PORT_LED0 & ~CONTROL_PIN_LED0)	;	// if false
}


void controlUpdateLEDArray( controlMemory_t* pControlMemory )
{
	// This function updates the 10-LED LED array

	static double 	fraction = 0;		// Fraction of LED array to be lit
	static uint8 	LED9_counter = 0;	// Counter used to regulate operation of LED9

	if( MAIN_LED_CYCLE )
	// Just cycle through the LEDs instead of using them to display temp
	{
		fraction += 0.1;
		fraction = modf(fraction,NULL);
	}
	else
	{
		fraction = (pControlMemory->motorTemp - pControlMemory->OAT)
			/ (MAIN_CRITICAL_TEMPERATURE - pControlMemory->OAT);
	}

	// Blink LED0
	// HACK - this is done in controlUpdateSystemLED(), just blinks with the
	// system LED
	__no_operation();

	// LED1 - LED8 - linearly correlated with motor temperature.
	// None lit up if temp = control block variable OAT
	// All lit up if temp = MAIN_CRITICAL_TEMPERATURE
	CONTROL_PORT_LED1 = ( fraction > CONTROL_LED1_THRESHOLD )
			? CONTROL_PORT_LED1 | CONTROL_PIN_LED1
			: CONTROL_PORT_LED1 & ~CONTROL_PIN_LED1;

	CONTROL_PORT_LED2 = ( fraction > CONTROL_LED2_THRESHOLD )
			? CONTROL_PORT_LED2 | CONTROL_PIN_LED2
			: CONTROL_PORT_LED2 & ~CONTROL_PIN_LED2;

	CONTROL_PORT_LED3 = ( fraction > CONTROL_LED3_THRESHOLD )
			? CONTROL_PORT_LED3 | CONTROL_PIN_LED3
			: CONTROL_PORT_LED3 & ~CONTROL_PIN_LED3;

	CONTROL_PORT_LED4 = ( fraction > CONTROL_LED4_THRESHOLD )
			? CONTROL_PORT_LED4 | CONTROL_PIN_LED4
			: CONTROL_PORT_LED4 & ~CONTROL_PIN_LED4;

	CONTROL_PORT_LED5 = ( fraction > CONTROL_LED5_THRESHOLD )
			? CONTROL_PORT_LED5 | CONTROL_PIN_LED5
			: CONTROL_PORT_LED5 & ~CONTROL_PIN_LED5;

	CONTROL_PORT_LED6 = ( fraction > CONTROL_LED6_THRESHOLD )
			? CONTROL_PORT_LED6 | CONTROL_PIN_LED6
			: CONTROL_PORT_LED6 & ~CONTROL_PIN_LED6;

	CONTROL_PORT_LED7 = ( fraction > CONTROL_LED7_THRESHOLD )
			? CONTROL_PORT_LED7 | CONTROL_PIN_LED7
			: CONTROL_PORT_LED7 & ~CONTROL_PIN_LED7;

	CONTROL_PORT_LED8 = ( fraction > CONTROL_LED8_THRESHOLD )
			? CONTROL_PORT_LED8 | CONTROL_PIN_LED8
			: CONTROL_PORT_LED8 & ~CONTROL_PIN_LED8;


	// LED 9 - Blink frequency determined
	// by how often this routine is called (at this point, 4 Hz).


	// If winch is too hot, blink continuously
	if( pControlMemory->isHot )
	{
		CONTROL_PORT_LED9 ^= CONTROL_PIN_LED9;
	}


	// If winch overcurrents, blink only 3 times.
	if( pControlMemory->isOvercurrent && pControlMemory->isHot==FALSE)
	{
		switch( LED9_counter )
		{

		case 0:
		case 2:
		case 4:
			CONTROL_PORT_LED9 |= CONTROL_PIN_LED9;
			LED9_counter++;
			break;

		case 1:
		case 3:
		case 5:
			CONTROL_PORT_LED9 &= ~CONTROL_PIN_LED9;
			LED9_counter++;
			break;

		default:
			break;

		}
	}

	else
		LED9_counter = 0;
}

void controlUpdateMotorTemp( 	controlMemory_t* pControlMemory,
								storageMemory_t* pStorageMemory )
{
	// For now, set temperature manually to be halfway in between critical temp
	// and OAT.
	// pControlMemory->motorTemp = (MAIN_CRITICAL_TEMPERATURE - pControlMemory->OAT)/2;

}

void controlUpdateLCD(	controlMemory_t* pControlMemory )
{
	// Declare local variables
	uint8 LCDMode = pControlMemory->LCDMode;
	double number;		// Number to be displayed
	uint8 hundreds;	// Value of hundreds place
	uint8 tens;		// Etc.
	uint8 ones;
	uint8 tenths;
	uint8 hundreths;
	uint8 LCDCharArray[7];	// Set of character indices from LCD_Char_Map
	uint8 i;
	bool isNegative;
	uint8 char0;
	uint8 char1;
	uint8 arrowMask;

	// Clear LCD segments
	LCDClear();

	// Set locals
	switch( LCDMode )
	{
	case CONTROL_LCD_DISPLAY_MODE_MOTOR_TEMP:
		char0 = LCD_CHAR_C;
		char1 = LCD_CHAR_DEGREE;
		arrowMask = LCD_ARROW_UP;
		number = pControlMemory->motorTemp;
		break;

	case CONTROL_LCD_DISPLAY_MODE_OAT:
		char0 = LCD_CHAR_C;
		char1 = LCD_CHAR_DEGREE;
		arrowMask = LCD_ARROW_RIGHT;
		number = pControlMemory->OAT;
		break;

	case CONTROL_LCD_DISPLAY_MODE_CURRENT:
		char0 = LCD_CHAR_A;
		char1 = LCD_CHAR_BLANK;
		arrowMask = LCD_ARROW_DOWN;
		number = pControlMemory->motorCurrent;
		break;

	case CONTROL_LCD_DISPLAY_MODE_VOLTAGE:
		char0 = LCD_CHAR_U;
		char1 = LCD_CHAR_BLANK;
		arrowMask = LCD_ARROW_LEFT;
		number = pControlMemory->motorVoltage;
		break;

	default:
		number = 0;
		break;
	}


	// If negative, reverse the sign so the number can be deciphered into digits
	// and set the negative flag.
	if( number < 0 )
	{
		isNegative = TRUE;
		number = -number;
	}
	else
	{
		isNegative = FALSE;
	}

//	// Round to nearest tenth
//	number *= 10;
//	number = floor( number + 0.5 );
//	number /= 10;

	/********* Display 'number' as a temperature in deg C *********/

	// Determine 100's place, 10's, etc.
	hundreds 	= (uint8) ( number/100	);
	number 		= fmodf( number, 100 );
	tens		= (uint8) ( number/10 	);
	number 		= fmodf( number, 10 );
	ones		= (uint8) ( number/1  	);
	number 		= fmodf( number, 1 );
	tenths		= (uint8) ( number*10 	);
	number 		= fmodf( number, 0.1 );
	hundreths	= (uint8) ( number*100 	);

	// Create array specifying which characters to display on LCD.
	// First element is rightmost character (of 7).
	LCDCharArray[0] = char0;
	LCDCharArray[1] = char1;
	LCDCharArray[2] = hundreths;
	LCDCharArray[3] = tenths;
	LCDCharArray[4] = ones;
	LCDCharArray[5] = tens;
	LCDCharArray[6] = hundreds;

	// Display characters
	for( i=0; i<7; i++ )
			LCDDisplayCharacter( i, LCDCharArray[i] );

	// Display decimal point
	LCDDisplaySpecialCharacter( LCD_DP4 );

	// Display minus sign, if necessary
	if( isNegative )
		LCDDisplaySymbol(LCD_SYM_MINUS);



	/********* Indicate operational states *********/

	// If plow connected, indicate F1
	if( pControlMemory->plowConnected )
		LCDDisplayFunction( LCD_F1 );

	// If motor has 'overcurrented,' indicated F2
	if( pControlMemory->isOvercurrent )
		LCDDisplayFunction( LCD_F2 );

	// If motor is 'too hot,' indicated F3
	if( pControlMemory->isHot )
		LCDDisplayFunction( LCD_F3 );


	/******** Indicate LCD display state **********/

	// Display which variable is being displayed on the LCD with arrows.
	LCDDisplayArrow( arrowMask );
}


void controlCheckForReset( void )
{
	// If switch 1 is pressed, reset microcontroller by writing the
	// wrong password to the watchdog timer control register.

	// If switch is pressed, will read 0.  Else, will read 1.
	if( CONTROL_READ_SWITCH_1 == FALSE )
		WDTCTL = CONTROL_WDT_WRONG_PASSWORD;
}

void controlCheckForLCDCycle( controlMemory_t* pControlMemory )
{
	static bool isPressed = FALSE;
	uint8 LCDMode;
	bool readSwitch;

	// Initialize locals as necessary
	LCDMode = pControlMemory->LCDMode;
	readSwitch = CONTROL_READ_SWITCH_2;

	// 2 state state machine - switches back and forth between pressed
	// and not pressed.  Done so that for each 1 press the controller will
	// only cycle through 1 LCD mode.  Remember that if switch is pressed,
	// switch input line will read 0.  Else, 1.
	if( isPressed )
	{
		// Check if switch 2 has been released.
		if( readSwitch != 0 )
		{
			isPressed = FALSE;	// Indicate that switch has been released
		}
	}
	else
	{
		// Check if switch 2 is pressed.
		if( readSwitch == FALSE)
		{
			isPressed = TRUE;	// Indicate that switch has been pressed

			// Cycle LCD mode stored in control struct
			LCDMode++;
			if( LCDMode == CONTROL_LCD_DISPLAY_MODE_NUM )
				LCDMode = CONTROL_LCD_DISPLAY_MODE_MOTOR_TEMP;
			pControlMemory->LCDMode = LCDMode;
		}
	}
}

void controlUpdateSelector( controlMemory_t* pControlMemory )
{
	// Determine selector position and update operation mode in control
	// memory appropriately if control mode is not forced with the
	// configuration MAIN_FORCED_OPERATION_MODE.  See main.h.

	// Determine operation mode from selector switch pins
	if( MAIN_FORCED_OPERATION_MODE == CONTROL_OPERATION_MODE_NUM )
	{
		if( CONTROL_READ_SELECTOR_2500 )
			pControlMemory->opMode = CONTROL_OPERATION_MODE_2500;
		else if( CONTROL_READ_SELECTOR_3500 )
			pControlMemory->opMode = CONTROL_OPERATION_MODE_3500;
		else if( CONTROL_READ_SELECTOR_4500 )
			pControlMemory->opMode = CONTROL_OPERATION_MODE_4500;
		else
			pControlMemory->opMode = CONTROL_OPERATION_MODE_OFF;
	}

	// Set operation mode to mode specified in main.h by MAIN_FORCED_OPERATION_MODE.
	else
	{
		pControlMemory->opMode = MAIN_FORCED_OPERATION_MODE;
	}

}


void controlInitTemperatureEstimation( controlMemory_t* pControlMemory )
{
	// Initialize temperature estimation parameters

	// Initialize motor temperature to OAT or manually specified temperature, as
	// specified by MAIN_INIT_TEMPERATURE_MANUAL.
	pControlMemory->motorTemp = (MAIN_INIT_TEMPERATURE_MANUAL) ?
			MAIN_INIT_TEMPERATURE : pControlMemory->OAT;

	// Set flag that temp estimation initialization has occured
	pControlMemory->tempEstimationIsInitialized = TRUE;
}


void controlDecayEstimatedTemp( controlMemory_t* pControlMemory, double dt )
{
	// Estimate using Euler approximation, first order decay:
	// dT = ( (OAT-motorTemp) / tau ) * dt;

	// Declare locals
	//double motorTemp;	// Motor temperature
	volatile double OAT;			// Outside air temperature
	//double dT;			// Change in motor temp over time dt
	volatile double T1;			// Temperature before decay
	volatile double T2;			// Temperature after decay

	// Initialize locals
	//motorTemp 	= pControlMemory->motorTemp;
	T1			= pControlMemory->motorTemp;
	OAT 		= pControlMemory->OAT;

//	// Decay motor temperature with euler approximation of 1st order decay
//	dT 			= (OAT - motorTemp) / MAIN_THERMAL_DECAY_TAU * dt;
//	dT 			= (OAT - motorTemp) * ( 1 - exp( -dt / MAIN_THERMAL_DECAY_TAU))
//	motorTemp 	+= dT;

	// Decay motor temperature with exact solution of 1st order decay
	// Follows form:
	//		T2 = T1 * exp(-t/tau)
	//	or, calculating with respect to a non-zero T_inf
	//	->	(T2-OAT) = (T1-OAT) * exp(-t/tau)

	T2 = (T1 - OAT) * exp(-dt / (double)MAIN_THERMAL_DECAY_TAU) + OAT;

	pControlMemory->motorTemp = T2;
}


void controlRaiseEstimatedTempCurrentOnly(	controlMemory_t* pControlMemory,
											storageMemory_t* pStorageMemory,
											uint16 numReadings )
{
	// Summary:
	//	1) Add the ADC12 readings from the current sensor since the last time
	//		this function was called (when written, this function was called at
	//		32 Hz by the main ISR (basic timer) ).
	//	2) Convert this sum to a sum of currents instead of a sum of 12-bit ADC
	//		readings.
	//	3) Calculate how much temperature rise that accounts for.
	//  4) Add to current temperature estimate.

	uint16 iter;
	uint32 ADC12Sum;		// Sum of 12-bit ADC readings in the last <numReadings>
	uint16 ADC12Reading;	// Temporary storage for a single ADC12 reading from current sensor
	uint16 minADC12Reading;	// Minimum allowable ADC reading from current sensor.
	double currentSum;		// ADC12Sum converted to a current (sum of last
							// <numReadings> current readings).
	double dT;				// Total temperature rise caused by all range of readings.
	double motorTemp;
	uint16 initialCurrentIdx;
	uint16 finalCurrentIdx;

	// Initialize locals
	ADC12Sum 	= 0;
	currentSum 	= 0;
	minADC12Reading = pControlMemory->ADC12MinReadingCurrent;
	motorTemp	= pControlMemory->motorTemp;

	// Determine index bounds of relevant readings within current storage array
	finalCurrentIdx		= pStorageMemory->currIdx;
	initialCurrentIdx	= finalCurrentIdx - numReadings;
	// If initial index is negative, then add STORAGE_CURRENT_LENGTH to get the
	// the correct initial array index (the storage array loops around).
	initialCurrentIdx	= (initialCurrentIdx + STORAGE_CURRENT_LENGTH) % STORAGE_CURRENT_LENGTH;

	// Find ADC12 reading sum since the last time this function was called
	// (temperature was estimated).
	iter = initialCurrentIdx;
	while(iter != finalCurrentIdx)
	{
		iter++;
		ADC12Reading = pStorageMemory->storageCurr[iter];
		// If current does not exceed MAIN_CURRENT_MINIMUM_READING, set to 0.  This
		// prevents noise in the current sensor from inflating the estimated
		// temperature while the motor isn't actually running.
		ADC12Reading = (ADC12Reading > minADC12Reading) ? ADC12Reading : 0;
		ADC12Sum += (uint32)ADC12Reading;
		// If the end of the storage array is reached, loop around to the
		// beginning.
		iter %= STORAGE_CURRENT_LENGTH;
	}

	// Convert ADC12 reading sum to a current sum
	// 	EQNS: 	(V-V0) / (sensor slope) = I ->
	//			(sum(V) - sum(V0)) / (sensor slope) = sum(I)
	currentSum = ( controlADC2Voltage(ADC12Sum) - MAIN_CURRENT_SENSOR_V0*numReadings )
					/ MAIN_CURRENT_SENSOR_SLOPE;

	// Convert current sum to a temperature rise
	// 	EQNS:	dT = I * (time of a sample) * (degC rise / second / amp)
	//			sum(dT) = sum(I) * (time of one sample) * ( degC rise / second / amp )
	dT = currentSum / MAIN_SELECTED_BT_FREQUENCY * MAIN_DEGC_RISE_PER_SEC_PER_AMP;
	motorTemp += dT;

	// Store change in control variable
	pControlMemory->motorTemp = motorTemp;
}


void controlRaiseEstimatedTempPower(	controlMemory_t* pControlMemory,
										storageMemory_t* pStorageMemory,
										uint16 numReadings,
										uint16 readingFreq)
{
	// Summary:
	//	1) Sum the product of the ADC12 readings from the current sensor and voltage
	//		sensor since the last time this function was called (when written, this
	//		function was called at 32 Hz by the main ISR (basic timer) ).
	//	2) Convert this sum to a sum of powers instead of a sum of 12-bit ADC
	//		reading products using the ADC->current or ADC->voltage conversions for each.
	//	3) Calculate how much temperature rise that would cause over the period of time
	//		specified by the number of readings received.
	//  4) Add to current temperature estimate.

	/******* Mathematical summary of calculations *******/

	// Variables
	// P 		= power delivered to winch
	// P_a		= power dissipated in armature coils through ohmic heating
	// P_w		= power leaving the motor work delivered to the shaft
	// E		= energy dissipated through ohmic heating in motor coils in a given time dt
	// I_a 		= armature current
	// V_a 		= armature voltage
	// V_sI		= voltage sensed by armature current sensor
	// V_sV		= voltage sensed by armature voltage sensor (not the same thing because
	//				armature voltage ~12V and must be divided to be sensed by the microcontroller
	// V_0I		= voltage bias of current sensor (V_sI when I_a = 0)
	// V_0V		= voltage bias of voltage sensor (V_sV when V_a = 0)
	// V_ADC	= voltage input to any given ADC channel
	// ADC_read	= ADC reading on a given sensor channel
	// ADC_I	= ADC reading from I_a sensor channel
	// ADC_V	= ADC reading from V_a sensor channel
	// ADC_res	= ADC resolution.  In our case, 2^12 = 4096.
	// V_plus	= ADC positive reference voltage
	// V_minus	= ADC negative reference voltage
	// K_I		= Slope of current sensor (V/A), MAIN_CURRENT_SENSOR_SLOPE
	// K_V		= Slope of voltage sensor (V sensed / V armature), MAIN_CURRENT_SENSOR_SLOPE
	// dt		= Length of time for which each current / voltage measurement is valid.
	//				Since our ADC takes measurements at MAIN_SELECTED_BT_FREQUENCY Hz (512),
	//				dt = 1 / MAIN_SELECTED_BT_FREQUENCY.
	// omega	= Angular velocity of motor
	// tau		= Torque delivered by motor
	// m		= motor mass
	// c		= motor specific heat capacity (J/kg*C)
	// C		= motor heat capacity (J/C)

	// Power equation
	// P 		= I_a * V_a
	// P 		= P_a + P_w
	// P_a + P_w = I_a * V_a
	// P_a 		= I_a*V_a  -  P_w
	// P_a 		= I_a*V_a  -  omega*tau								(1) Luckily, omega and tau are both roughly inferrable from I_a
	//																	based on motor characteristics.

	// Calculating a sensed voltage from a given ADC reading
	// V_ADC 	= (ADC_read/ADC_res) * (V_plus - V_minus) + V_minus	(2) Convert ADC reading to voltage reading

	// Calculate current & voltage from ADC reading
	// I_a 		= (V_sI - V_0I) / K_I								(3) Current sensor equation
	// 		Substitute (2) into (3), where  V_sI -> V_ADC
	// 		and  ADC_I -> ADC_read
	// I_a 	= ( (ADC_I/ADC_res) * (V_plus - V_minus)
	//			+ V_minus - V_0I ) / K_I							(4) Armature current from ADC
	// Similarly,
	// V_a 	= ( (ADC_V/ADC_res) * (V_plus - V_minus)
	//			+ V_minus - V_0V ) / K_V							(5) Armature voltage from ADC

	// Calculate energy dissipated in coils in a given amount of
	// time.
	// E 	= P_a * dt
	// 		Sub from (1)
	// 		= (I_a*V_a  -  omega*tau) * dt							(6) Final eq.  I_a and V_a can be obtained from (4), (5)

	// Calculate temperature rise
	// E	= m*c*dT
	//   	= C*dT
	// dT	= E / C													(7)


	volatile uint16 iter;
	volatile uint16 ADC12VoltageReading;		// ADC12 reading of armature voltage sensor
	volatile uint16 ADC12CurrentReading;		// ADC12 reading of armature current sensor
	volatile uint16 minADC12CurrentReading;			// Minimum allowed ADC12 reading from current sensor
	volatile double nonConvertedPowerSumIn;	// Sum across all readings in last <numReadings> of the
									// power at each reading.  This sum is not a sum of the
									// true power, because the voltage READINGS from the current
									// and voltage SENSORS have not yet been converted using
									// MAIN_CURRENT_SENSOR_SLOPE and MAIN_VOLTAGE_SENSOR_SLOPE,
									// respectively.  This conversion is done on the entire sum
									// All at once.  See mathematical summary above.
	volatile double totalEnergyIn;			// The total amount of energy that enters the winch
	volatile double totalEnergyOut;			// The total amount of energy that exits the winch in the form
									// of useful work (omega*tau*dt)
	volatile double powerOutSum;				// Sum of powers out at each dt before the sum is multiplied
									// by dt to yield totalEnergyOut.
	volatile double energyDefect;			// (energy in) - (energy out) = energy dissipated in winch coils
	volatile double dT;						// Total temperature rise caused by all range of readings.
	volatile double motorTemp;
	volatile uint16 initialStorageIdx;		// Initial index to be read from current / voltage storage
									// arrays.
	volatile uint16 finalStorageIdx;			// Final index to be read from current / voltage storage arrays.
	volatile double voltageVoltage;			// V_sI - V_0I
	volatile double currentVoltage;			// V_sV - V_0V
	volatile double current;			// I_a
	volatile double product;				// Used for multiplication calculations to reduce floating point calcs.


	// Initialize locals
	nonConvertedPowerSumIn 	= 0;
	motorTemp				= pControlMemory->motorTemp;
	minADC12CurrentReading 	= pControlMemory->ADC12MinReadingCurrent;

	// Determine index bounds of relevant readings within current storage array
	finalStorageIdx		= pStorageMemory->currIdx;
	initialStorageIdx	= finalStorageIdx - numReadings + 1;
	// If initial index is negative, then add STORAGE_CURRENT_LENGTH to get the
	// the correct initial array index (the storage array loops around).
	initialStorageIdx	= (initialStorageIdx + STORAGE_CURRENT_LENGTH) % STORAGE_CURRENT_LENGTH;

	// Find ADC12 reading product sum since the last time this function was called
	// (temperature was estimated).  Note that both current and voltage have identical
	// indices in their storage arrays.
	nonConvertedPowerSumIn = 0;
	iter = initialStorageIdx - 1;
	do
	{
		// Iterate through each current / voltage reading index.  If the physical end of the
		// storage array is reached, loop around to the beginning.
		iter++;
		iter %= STORAGE_CURRENT_LENGTH;
		// Convert a single set of voltage / current readings to a non-converted power
		// (see variable description above) and add them to the non-converted power sum.
		ADC12VoltageReading = storageRetrieveValue(STORAGE_VARIABLE_VOLTAGE, iter, pStorageMemory);
		ADC12CurrentReading = storageRetrieveValue(STORAGE_VARIABLE_CURRENT, iter, pStorageMemory);
		// If current does not exceed MAIN_CURRENT_MINIMUM_READING, set to 0.  This
		// prevents noise in the current sensor from inflating the estimated
		// temperature while the motor isn't actually running.
		ADC12CurrentReading = (ADC12CurrentReading > minADC12CurrentReading)
				? ADC12CurrentReading : 0;
		voltageVoltage		= controlADC2Voltage( ADC12VoltageReading ) - MAIN_VOLTAGE_SENSOR_V0;
		currentVoltage		= controlADC2Voltage( ADC12CurrentReading ) - MAIN_CURRENT_SENSOR_V0;

		product = voltageVoltage * currentVoltage;

		// If power product is positive, add to <nonConvertedPowerSumIn>.  This prevents
		// reducing motor temperature by accident.
		nonConvertedPowerSumIn += (product > 0) ? product : 0;

	} while(iter != finalStorageIdx);


	// Convert nonConvertedPowerSumIn to a true energy sum (in joules)
	// 	EQNS: 	dE_in	= P_a * dt
	//					= V_a * I_a * dt
	//					= (V_sV - V_0V) / K_V  *  (V_sI - V_0I) / K_I  *  dt
	//					= (voltageVoltage) / K_V  *  (currentVoltage) / K_I  *  dt
	//					= nonConvertedPowerIn * dt/(K_V*K_I)
	//			E_in 	= sum( dE_in )
	//			 		= nonConvertedPowerSumIn * dt/(K_V*K_I)
	//					= nonConvertedPowerSumIn / (readingFreq*K_V*K_I)
	//					= total power in
	totalEnergyIn	= nonConvertedPowerSumIn
			/ (readingFreq * MAIN_VOLTAGE_SENSOR_SLOPE * MAIN_CURRENT_SENSOR_SLOPE);


	// Find total energy out.  Do this by estimating power out (useful that exits winch
	// through cable).
	// P_w		= power out in work
	//			= omega * tau
	// E_out	= total energy out in work in total time dt*numReadings
	// 			= sum( omega * tau * dt )
	// 			= dt * sum( omega * tau )
	//			= sum(omega * tau) / reading_frequency
	// Omega and tau can be estimated by filtering the incoming winch current (to exclude
	// spikes, e.g., inrush current) and inferring omega & tau from the known steady-state
	// values of omega and tau at that current.  Remember, current, motor angular velocity,
	// and motor torque are non-independent in steady state.  If you know one, and you know
	// the winch motor is at steady state, you automatically know the others.  Now, when
	// the winch is in operation, it's never truly in steady state, but we assume that the
	// rise time of the physical system is small enough such that we can estimate omega &
	// tau relatively well, and thus work or power out.

	// In this implementation, we've skipped filtering incoming current and have assumed that
	// calculation-detrimental transients (e.g., inrush spikes) are short enough to not make that big of a difference.
	powerOutSum = 0;
	iter = initialStorageIdx - 1;
	do
	{
		// Iterate through each current / voltage reading index.  If the physical end of the
		// storage array is reached, loop around to the beginning.
		iter++;
		iter %= STORAGE_CURRENT_LENGTH;

		// Retrieve current value
		ADC12CurrentReading
			= storageRetrieveValue( STORAGE_VARIABLE_CURRENT, iter, pStorageMemory );

		// If current does not exceed MAIN_CURRENT_MINIMUM_READING, set to 0.  This
		// prevents noise in the current sensor from affecting the estimated
		// temperature while the motor isn't actually running.
		ADC12CurrentReading
			= (ADC12CurrentReading > minADC12CurrentReading)
			? ADC12CurrentReading : 0;

		// Convert to a current value.
		current = ( controlADC2Voltage( ADC12CurrentReading )
				- MAIN_CURRENT_SENSOR_V0 ) / MAIN_CURRENT_SENSOR_SLOPE;

		// Based on that current, estimate the power the motor is delivering to its shaft.
		// If current is not large enough, ignore (was obtained from noise when at very
		// low current.  Only occurs when winch is not running).
		if( current > MAIN_CURRENT_MINIMUM_READING )
		{
			powerOutSum += controlCalculatePowerOut( current );
		}

	} while(iter != finalStorageIdx);

	// Calculate total energy deliverd to the shaft
	totalEnergyOut = powerOutSum / MAIN_SELECTED_BT_FREQUENCY;

	// Calculate energy delivered to coils, which is the difference between the energy
	// into the motor and the energy delivered to the motor shaft.
	energyDefect = totalEnergyIn - totalEnergyOut;

	// Calculate temperature rise in coils based on that energy defect
	dT = energyDefect / MAIN_MOTOR_HEAT_CAPACITY;
	motorTemp += dT;

	// Store change in control variable
	pControlMemory->motorTemp = motorTemp;
}



void controlRaiseEstimatedTempPowerInferred(controlMemory_t* pControlMemory,
											storageMemory_t* pStorageMemory,
											uint16 numReadings,
											uint16 readingFreq)
{
	// Summary:
	//	1) Sum the product of the ADC12 readings from the current sensor and voltage
	//		sensor since the last time this function was called (when written, this
	//		function was called at 32 Hz by the main ISR (basic timer) ).
	//	2) Convert this sum to a sum of powers instead of a sum of 12-bit ADC
	//		reading products using the ADC->current or ADC->voltage conversions for each.
	//	3) Calculate how energy in, and then calculate the corresponding energy in (based
	//		on empirical energy in vs. temperature rise calculations only, not on energy
	//		defect.
	//	4) Calculate how much temperature rise that would cause over the period of time
	//		specified by the number of readings received.
	//  5) Add to current temperature estimate.

	/******* Mathematical summary of calculations *******/

	// Variables
	// P 		= power delivered to winch
	// P_a		= power dissipated in armature coils through ohmic heating
	// P_w		= power leaving the motor work delivered to the shaft
	// E		= energy dissipated through ohmic heating in motor coils in a given time dt
	// I_a 		= armature current
	// V_a 		= armature voltage
	// V_sI		= voltage sensed by armature current sensor
	// V_sV		= voltage sensed by armature voltage sensor (not the same thing because
	//				armature voltage ~12V and must be divided to be sensed by the microcontroller
	// V_0I		= voltage bias of current sensor (V_sI when I_a = 0)
	// V_0V		= voltage bias of voltage sensor (V_sV when V_a = 0)
	// V_ADC	= voltage input to any given ADC channel
	// ADC_read	= ADC reading on a given sensor channel
	// ADC_I	= ADC reading from I_a sensor channel
	// ADC_V	= ADC reading from V_a sensor channel
	// ADC_res	= ADC resolution.  In our case, 2^12 = 4096.
	// V_plus	= ADC positive reference voltage
	// V_minus	= ADC negative reference voltage
	// K_I		= Slope of current sensor (V/A), MAIN_CURRENT_SENSOR_SLOPE
	// K_V		= Slope of voltage sensor (V sensed / V armature), MAIN_CURRENT_SENSOR_SLOPE
	// dt		= Length of time for which each current / voltage measurement is valid.
	//				Since our ADC takes measurements at MAIN_SELECTED_BT_FREQUENCY Hz (512),
	//				dt = 1 / MAIN_SELECTED_BT_FREQUENCY.
	// omega	= Angular velocity of motor
	// tau		= Torque delivered by motor
	// m		= motor mass
	// c		= motor specific heat capacity (J/kg*C)
	// C		= motor heat capacity (J/C)

	// Power equation
	// P 		= I_a * V_a											(1) Power into winch

	// Calculating a sensed voltage from a given ADC reading
	// V_ADC 	= (ADC_read/ADC_res) * (V_plus - V_minus) + V_minus	(2) Convert ADC reading to voltage reading

	// Calculate current & voltage from ADC reading
	// I_a 		= (V_sI - V_0I) / K_I								(3) Current sensor equation
	// 		Substitute (2) into (3), where  V_sI -> V_ADC
	// 		and  ADC_I -> ADC_read
	// I_a 	= ( (ADC_I/ADC_res) * (V_plus - V_minus)
	//			+ V_minus - V_0I ) / K_I							(4) Armature current from ADC
	// Similarly,
	// V_a 	= ( (ADC_V/ADC_res) * (V_plus - V_minus)
	//			+ V_minus - V_0V ) / K_V							(5) Armature voltage from ADC

	// Calculate energy into winch in a given amount of time.
	// E 	= P_a * dt
	//		= I_a * V_a												(6) I_a and V_a can be obtained from (4), (5)
	// dT	= f(E)													(7) Empirical relationship obtained from meas's


	uint16 iter;
	uint16 ADC12VoltageReading;		// ADC12 reading of armature voltage sensor
	uint16 ADC12CurrentReading;		// ADC12 reading of armature current sensor
	uint16 minADC12CurrentReading;			// Minimum allowed ADC12 reading from current sensor
	double nonConvertedPowerSumIn;	// Sum across all readings in last <numReadings> of the
									// power at each reading.  This sum is not a sum of the
									// true power, because the voltage READINGS from the current
									// and voltage SENSORS have not yet been converted using
									// MAIN_CURRENT_SENSOR_SLOPE and MAIN_VOLTAGE_SENSOR_SLOPE,
									// respectively.  This conversion is done on the entire sum
									// All at once.  See mathematical summary above.
	double totalEnergyIn;			// The total amount of energy that enters the winch
	double dT;						// Total temperature rise caused by all range of readings.
	double motorTemp;
	uint16 initialStorageIdx;		// Initial index to be read from current / voltage storage
									// arrays.
	uint16 finalStorageIdx;			// Final index to be read from current / voltage storage arrays.
	double voltageVoltage;			// V_sI - V_0I
	double currentVoltage;			// V_sV - V_0V


	// Initialize locals
	nonConvertedPowerSumIn 	= 0;
	motorTemp				= pControlMemory->motorTemp;
	minADC12CurrentReading 	= pControlMemory->ADC12MinReadingCurrent;

	// Determine index bounds of relevant readings within current storage array
	finalStorageIdx		= pStorageMemory->currIdx;
	initialStorageIdx	= finalStorageIdx - numReadings + 1;
	// If initial index is negative, then add STORAGE_CURRENT_LENGTH to get the
	// the correct initial array index (the storage array loops around).
	initialStorageIdx	= (initialStorageIdx + STORAGE_CURRENT_LENGTH) % STORAGE_CURRENT_LENGTH;

	// Find ADC12 reading product sum since the last time this function was called
	// (temperature was estimated).  Note that both current and voltage have identical
	// indices in their storage arrays.
	nonConvertedPowerSumIn = 0;
	iter = initialStorageIdx - 1;
	do
	{
		// Iterate through each current / voltage reading index.  If the physical end of the
		// storage array is reached, loop around to the beginning.
		iter++;
		iter %= STORAGE_CURRENT_LENGTH;
		// Convert a single set of voltage / current readings to a non-converted power
		// (see variable description above) and add them to the non-converted power sum.
		ADC12VoltageReading = storageRetrieveValue(STORAGE_VARIABLE_VOLTAGE, iter, pStorageMemory);
		ADC12CurrentReading = storageRetrieveValue(STORAGE_VARIABLE_CURRENT, iter, pStorageMemory);
		// If current does not exceed MAIN_CURRENT_MINIMUM_READING, set to 0.  This
		// prevents noise in the current sensor from inflating the estimated
		// temperature while the motor isn't actually running.
		ADC12CurrentReading = (ADC12CurrentReading > minADC12CurrentReading)
				? ADC12CurrentReading : 0;
		voltageVoltage		= controlADC2Voltage( ADC12VoltageReading ) - MAIN_VOLTAGE_SENSOR_V0;
		currentVoltage		= controlADC2Voltage( ADC12CurrentReading ) - MAIN_CURRENT_SENSOR_V0;

		nonConvertedPowerSumIn += voltageVoltage * currentVoltage;

	} while(iter != finalStorageIdx);


	// Convert nonConvertedPowerSumIn to a true energy sum (in joules)
	// 	EQNS: 	dE_in	= P_a * dt
	//					= V_a * I_a * dt
	//					= (V_sV - V_0V) / K_V  *  (V_sI - V_0I) / K_I  *  dt
	//					= (voltageVoltage) / K_V  *  (currentVoltage) / K_I  *  dt
	//					= nonConvertedPowerIn * dt/(K_V*K_I)
	//			E_in 	= sum( dE_in )
	//			 		= nonConvertedPowerSumIn * dt/(K_V*K_I)
	//					= nonConvertedPowerSumIn / (readingFreq*K_V*K_I)
	//					= total power in
	totalEnergyIn	= nonConvertedPowerSumIn
			/ (readingFreq * MAIN_VOLTAGE_SENSOR_SLOPE * MAIN_CURRENT_SENSOR_SLOPE);


	// Calculate temperature rise in coils based empirical power in vs.
	// temperature rise meas's
	dT = controlCalculateTempRiseFromEnergyIn( totalEnergyIn );
	motorTemp += dT;

	// Store change in control variable
	pControlMemory->motorTemp = motorTemp;
}

void controlUpdateCurrentAndVoltage(controlMemory_t* pControlMemory,
									storageMemory_t* pStorageMemory)
{
	uint16 ADCReading;
	double sensorVoltage;
	double sensorReading;

	// Update instantaneous current
	ADCReading = storageRetrieveValue( STORAGE_VARIABLE_CURRENT, pStorageMemory->currIdx, pStorageMemory );
	sensorVoltage = controlADC2Voltage( ADCReading );
	sensorReading = (sensorVoltage - MAIN_CURRENT_SENSOR_V0) / MAIN_CURRENT_SENSOR_SLOPE;
	pControlMemory->motorCurrent = sensorReading;

	// Update instantaneous voltage
	ADCReading = storageRetrieveValue( STORAGE_VARIABLE_VOLTAGE, pStorageMemory->voltIdx, pStorageMemory );
	sensorVoltage = controlADC2Voltage( ADCReading );
	sensorReading = (sensorVoltage - MAIN_VOLTAGE_SENSOR_V0) / MAIN_VOLTAGE_SENSOR_SLOPE;
	pControlMemory->motorVoltage = sensorReading;
}


/******  Implementation of private functions   ********/
double controlADC2Voltage( uint16 ADC12Reading )
{
	return ( (double)ADC12Reading / MAIN_ADC_RES )
			* (MAIN_ADC_VREF_PLUS - MAIN_ADC_VREF_MINUS);
}

double controlCalculatePowerOut( double current )
{
//	double omega;		// Motor angular velocity
//	double tau;			// Motor torque
//	double powerOut;	// Power motor delivers to shaft at given current.
//
//	// Calculate motor angular velocity omega with the given current.
//	//omega = tablesReadFromTable( TABLES_VARIABLE_OMEGA, current );
//	omega = 0;
//
//	// Calculate motor angular velocity tau with the given current.
//	//tau = tablesReadFromTable( TABLES_VARIABLE_TAU, current );
//	tau = 0;
//
//	// Calculate power to motor shaft
//	powerOut = omega*tau;

	return tablesReadFromTable( TABLES_VARIABLE_POWER_OUT, current );

}

double controlCalculateTempRiseFromEnergyIn( double energy )
{
	// Temperature rise equation based on empirical energy in vs. temp rise
	// dT = m*dE + T0
	// dT = temperature increase
	// dE = amount of total energy in to motor
	// m  = slope
	// T0 = y-intercept

	return MAIN_TEMP_RISE_BY_ENERGY_IN_SLOPE*energy
			+ MAIN_TEMP_RISE_BY_ENERGY_IN_T0;
}
