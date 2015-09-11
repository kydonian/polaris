/*
 * storage.c
 *
 *  Implementation of ADC data storage routine.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#include "typedefs.h"
#include "storage.h"
#include "string.h"
#include "control.h"



/** Implementation of Public Functions **/
void storageInit( storageMemory_t* pMemory )
{
	memset( (uint16*)pMemory->storageTemp, 0x00, sizeof(pMemory->storageTemp) );
	memset( (uint16*)pMemory->storageCurr, 0x00, sizeof(pMemory->storageCurr) );
	memset( (uint16*)pMemory->storageVolt, 0x00, sizeof(pMemory->storageVolt) );
	memset( (uint16*)pMemory->storageCurrFiltered, 0x00, sizeof(pMemory->storageVolt) );
	pMemory->tempIdx = 0;
	pMemory->currIdx = 0;
	pMemory->voltIdx = 0;
	pMemory->currFilteredIdx = 0;
}


void storageStoreValue( uint16 value, storageVariable_t variable,
						storageMemory_t* pMemory)
{
	switch( variable )
	{
		case STORAGE_VARIABLE_TEMPERATURE:
			{
				// Increment index and loop if max reached
				pMemory->tempIdx++;
				pMemory->tempIdx %= STORAGE_TEMPERATURE_LENGTH;

				// Store given value at specified array index
				pMemory->storageTemp[pMemory->tempIdx] = value;
			}
			break;

		case STORAGE_VARIABLE_CURRENT:
			{
				// Increment index and loop if max reached
				pMemory->currIdx++;
				pMemory->currIdx %= STORAGE_CURRENT_LENGTH;

				// Invert ADC reading.  See control.h definitions.
				if( CONTROL_FLIP_CURRENT_ADC )
					value = MAIN_ADC_RES - value;

				// Store given value at specified array index
				pMemory->storageCurr[pMemory->currIdx] = value;
			}
			break;

		case STORAGE_VARIABLE_CURRENT_FILTERED:
			{
				// Increment index and loop if max reached
				pMemory->currFilteredIdx++;
				pMemory->currFilteredIdx %= STORAGE_FILTERED_CURRENT_LENGTH;

				// Store given value at specified array index
				pMemory->storageCurrFiltered[pMemory->currFilteredIdx] = value;
			}
			break;

		case STORAGE_VARIABLE_VOLTAGE:
			{
				// Increment index and loop if max reached
				pMemory->voltIdx++;
				pMemory->voltIdx %= STORAGE_VOLTAGE_LENGTH;

				// Store given value at specified array index
				pMemory->storageVolt[pMemory->voltIdx] = value;
			}
			break;
	}
}


uint16 storageRetrieveValue( 	storageVariable_t variable,
								uint16 index,
								storageMemory_t* pMemory)
{
	switch( variable )
	{
		case STORAGE_VARIABLE_TEMPERATURE:
			{
				// Return specified value
				return pMemory->storageTemp[index];
			}
			break;

		case STORAGE_VARIABLE_CURRENT:
			{
				// Return specified value
				return pMemory->storageCurr[index];
			}
			break;

		case STORAGE_VARIABLE_VOLTAGE:
			{
				// Return specified value
				return pMemory->storageVolt[index];
			}
			break;

		case STORAGE_VARIABLE_CURRENT_FILTERED:
			{
				// Return specified value
				return pMemory->storageCurrFiltered[index];
			}
			break;
	}
	return 0;
}
