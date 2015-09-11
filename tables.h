/*
 * tables.h
 *
 *  Header file of table-based methods.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#include "typedefs.h"

/******** Macro definitions ********/

#define TABLES_LENGTH_CURRENT_V_OMEGA	3
#define TABLES_LENGTH_CURRENT_V_TAU		3
#define TABLES_LENGTH_CURRENT_V_POWER_OUT	30

typedef enum
{
	TABLES_VARIABLE_OMEGA,
	TABLES_VARIABLE_TAU,
	TABLES_VARIABLE_POWER_OUT,
	TABLES_VARIABLE_NUM
} tablesVariable_t;



/******** Public functions ********/
double tablesReadFromTable( tablesVariable_t variable, double independent );
