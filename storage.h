/*
 * storage.c
 *
 *  Header file of ADC data storage routine.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#ifndef __STORAGE_H__
#define __STORAGE_H__

#include "typedefs.h"
#include "main.h"

/** Storage array parameters **/
#define STORAGE_TEMPERATURE_LENGTH		(MAIN_SELECTED_BT_FREQUENCY/4)	// 128
#define STORAGE_CURRENT_LENGTH			(MAIN_SELECTED_BT_FREQUENCY/4)	// 128
#define STORAGE_VOLTAGE_LENGTH			(MAIN_SELECTED_BT_FREQUENCY/4)	// 128
#define STORAGE_FILTERED_CURRENT_LENGTH	(MAIN_SELECTED_BT_FREQUENCY/4)	// 128
	// needed to digitally filter the current signal for power-out inference.
	// This filter is detailed in storageFilterCurrent().

typedef enum storageVariable
{
	STORAGE_VARIABLE_TEMPERATURE = 0,
	STORAGE_VARIABLE_CURRENT,
	STORAGE_VARIABLE_CURRENT_FILTERED,
	STORAGE_VARIABLE_VOLTAGE,
	STORAGE_VARIABLE_NUM
} storageVariable_t ;

/** Local variables **/

/* storageMemory_t struct
 * storageTemp			Array that stores AD12 values read from temperature sensor.
 * storageCurr			Array that stores AD12 values read from current sensor.
 * storageVolt			Array that stores AD12 values read from voltage sensor.
 * storageCurrFiltered	Array that stores AD12 values read from lowpass filtered
 * 						current sensor.  The primary current sensor is already
 * 						lowpass filtered, but is filtered at a lower frequency that
 * 						precludes sensing inrush current.
 * tempIdx				The array index (starting at 0) of the last value stored
 * 						in the temperature storage array.
 * currIdx				The array index (starting at 0) of the last value stored
 * 						in the current storage array.
 * voltIdx				The array index (starting at 0) of the last value stored
 * 						in the voltage storage array.
 * currFilteredIdx		The array index (starting at 0) of the last value stored
 * 						in the filtered current storage array.

 */
typedef struct
{
	volatile uint16 storageTemp[STORAGE_TEMPERATURE_LENGTH];
	volatile uint16 storageCurr[STORAGE_CURRENT_LENGTH];
	volatile uint16 storageVolt[STORAGE_VOLTAGE_LENGTH];
	volatile uint16 storageCurrFiltered[STORAGE_FILTERED_CURRENT_LENGTH];
	volatile uint16 tempIdx;
	volatile uint16 currIdx;
	volatile uint16 voltIdx;
	volatile uint16 currFilteredIdx;
} storageMemory_t;
// Total RAM allocated: 2310 bytes


/** Public functions **/

// Initializes storage variables
void storageInit( storageMemory_t* pMemory );

// Stores a specified value in the appropriate array and increments that array's index.
void storageStoreValue( uint16 value, storageVariable_t variable, storageMemory_t* pMemory);

// Retrieve a specified value from a place in the array.
uint16 storageRetrieveValue( 	storageVariable_t variable,
								uint16 index,
								storageMemory_t* pMemory);

#endif /* __STORAGE_H__ */
