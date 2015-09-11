/***************************************************************

main.h
Nathan Honka
1/13/2012

Definitions for winch saver project principal file (main.c).

***************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__

#include "typedefs.h"



/*** RUN OPTIONS, EXTERNAL CALIBRATIONS, MAGIC NUMBERS, ETC  ***/

// Run as if plow is always connected (i.e., always limit current)?
#define MAIN_PLOW_ALWAYS_CONNECTED		FALSE

// How many temperature ADC measurements will be necessary to initialize
// temperature measurement.
// Shorter times -> enable winch sooner, longer times -> more temperature
// measurements to use for more accurate initialization.  However, the OAT
// is just an estimate of the motor temperature anyway, so it shouldn't need
// more than a few ADC measurements.
#define MAIN_ADC_MEAS_FOR_TEMPERATURE_INIT	128

// Initialize initial motor temperature manually or automatically from OAT
#define MAIN_INIT_TEMPERATURE_MANUAL	FALSE
#define MAIN_INIT_TEMPERATURE			150	// DegC

// If desired, force to run in a certain operation mode (instead of reading
// the mode off the selector switch pins).  To just read the mode off the
// selector switch pins, specify CONTROL_OPERATION_MODE_NUM.
#define MAIN_FORCED_OPERATION_MODE		CONTROL_OPERATION_MODE_3500

// Use LED array to display temperature or just cycle through
#define MAIN_LED_CYCLE					FALSE

// How long & often system LED should blink.
#define MAIN_SYSTEM_LED_BLINK_TIME		0.05			// Will blink for 0.05 seconds
#define MAIN_SYSTEM_LED_BLINK_PER_SEC	2.0				// Will blink 2 times per second

// ADC specifications
#define MAIN_ADC_VREF_PLUS				2.5173
#define MAIN_ADC_VREF_MINUS				0.0
#define MAIN_ADC_RES					4096

// internal temperature sensor calibration, assuming slope-intercept
#define MAIN_THERMOCOUPLE_V0			1.0056176	// (V)  	- Obtained from (FG4618 documentation, 53),
													// then externally calibrated.  Note: This value can drift.
#define MAIN_THERMOCOUPLE_SLOPE			0.00355		// (V/degC)	- Obtained from (FG4618 documentation, 53),
													// not externally calibrated

// Current sensor calibration, assuming slope-intercept
#define MAIN_CURRENT_SENSOR_V0_NONINVERTED	2.428	// (V) 		- Calibrated (voltage at I = 0).  Actual voltage,
													// not inverted (see control.h).
#define MAIN_CURRENT_SENSOR_V0			(MAIN_ADC_VREF_PLUS - MAIN_CURRENT_SENSOR_V0_NONINVERTED)
													// (V) 		- Calibrated (voltage at I = 0).  Inverted per control.h.
													//			See CONTROL_FLIP_CURRENT_ADC.
													// Note: this voltage is inverted, i.e., 2.5 - V0.  See control.h for explanation.
#define MAIN_CURRENT_SENSOR_SLOPE		5.4016e-003	// (V/A)	- Calibrated (average of slightly nonlinear function)
#define MAIN_CURRENT_MINIMUM_READING	5.0			// (A)		- Current below this value will be ignored in ohmic
													//			- heating calcs.  At no load, any winch should run a
													//				current well above this value (10+).

// Voltage sensor calibration, assuming slope-intercept
// Va = armature voltage
// Vs = sensor voltage
#define MAIN_VOLTAGE_SENSOR_V0			0.0			// (V)		- Not calibrated (Vs @ Va = 0)	??
#define MAIN_VOLTAGE_SENSOR_SLOPE		0.176022	// (Vs/Va)  - Calibrated

// Operation temperature limits
#define MAIN_CRITICAL_TEMPERATURE		100.0			// (degC) 	- Temperature at which MC disengages winch to prevent burnout
#define MAIN_THRESHOLD_TEMPERATURE		85.0			// (degC)	- Temperature at which MC reengages winch after temp-signaled cutoff.


// Motor thermal characteristics
#define MAIN_MOTOR_HEAT_CAPACITY		397.0	// (J/degC) - Heat capacity of motor core in degrees C per joule

/*

2-term exponential version

// T = C1 * e^( -t / tau1 )  +  C2 * e^( -t / tau2 )
#define MAIN_THERMAL_DECAY_C1			76.61
#define MAIN_THERMAL_DECAY_TAU1			1699.5		// (sec) 	- Time constant of primary decay mode
#define MAIN_THERMAL_DECAY_C2			34.41
#define MAIN_THERMAL_DECAY_TAU2			25549.3		// (sec)	- Time constant of secondary decay mode

*/

// 1-term exponential version
#define MAIN_THERMAL_DECAY_TAU			700			// (sec) 	- Time constant of thermal decay


// Motor ohmic heating equation: based on a single degC / (sec*A)
// ?? Needs to be updated to estimate based on armature voltage
#define MAIN_DEGC_RISE_PER_SEC_PER_AMP	0.0167		// (DegC / s-A) - NOT calibrated, rough guess	??

// Temperature rise equation based on empirical energy in vs. temp rise
// dT = m*dE + T0
#define MAIN_TEMP_RISE_BY_ENERGY_IN_SLOPE	5e-4	// (degC/J) -
#define MAIN_TEMP_RISE_BY_ENERGY_IN_T0		0		// (degC) -

/**********  Definitions ***************************/

// Basic timer frequencies

// fBT = fACLK / 64  = 32768 Hz / 64  = 512 Hz
#define BT_512HZ    (BT_fCLK2_ACLK + BT_fCLK2_DIV64)
// fBT = fACLK / 256 = 32768 Hz / 256 = 128 Hz
#define BT_128HZ 	(BT_fCLK2_ACLK + BT_fCLK2_DIV256)
// fBT = fACLK / 256 / 64 = 32768 Hz / 256 / 64 = 2 Hz
#define BT_2HZ 		(BTDIV + BT_fCLK2_DIV64)

#define MAIN_SELECTED_BT_FREQUENCY			512




/**********  Declarations *************************/


#endif //__MAIN_H__
