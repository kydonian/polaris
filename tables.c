/*
 * tables.c
 *
 *  Declaration of constant tables stored in ROM along with methods for
 *  retrieving information from those tables.
 *
 *  Nathan Honka
 *  Polaris Capstone Team
 *  1/24/2012
 *
 */

#include "tables.h"
#include "typedefs.h"


/******* Declaration of tables **********/

// General format for tables:
// Two columns:
//	(0) - independent variable
//	(1) - dependent variable
//
// Variable name = tablesIndependentVDependent

// Example:
// 	ind1	dep1
//	ind2	dep2
//	ind3	dep3

// Table for inferring omega (winch motor angular velocity) from steady state
// motor current.
const double tablesCurrentVOmega[TABLES_LENGTH_CURRENT_V_OMEGA][2] =
	{	{	1,	2},
		{	3,	4},
		{	5,	6}
	};

// Table for inferring tau (winch motor torque) from steady state motor current.
const double tablesCurrentVTau[TABLES_LENGTH_CURRENT_V_TAU][2] =
	{	{	1,	2},
		{	3,	4},
		{	5,	6}
	};


// Table for inferring motor power out from current
const double tablesCurrentVPowerOut[TABLES_LENGTH_CURRENT_V_POWER_OUT][2] =
{	{0,			0},			// Not engaged
	{8.416,   46.71},
    {8.476,   46.71},
    {8.906,   46.59},
    {9.723,   54.82},
    {11.334,  70.99},
    {13.754,  89.01},
    {16.189,  115.0},
    {19.195,  147.3},
    {22.786,  184.1},
    {27.268,  227.0},
    {32.651,  277.5},
    {38.893,  328.7},
    {46.376,  386.4},
    {54.139,  435.9},
    {63.448,  505.9},
    {73.391,  560.3},
    {84.157,  592.4},
    {95.260,  634.1},
    {106.197, 672.9},
    {119.844, 699.9},
    {131.517, 728.7},
    {143.601, 736.6},
    {157.387, 716.9},
    {171.074, 683.2},
    {186.614, 618.1},
    {198.775, 531.2},
    {206.805, 440.0},
    {214.586, 330.0},
    {230.0000,	0.000}		// Stall

};

// Table for 4500-lb
//{	{0,			0},
//    {8.7430,   35.8600},
//    {8.6760,   35.8800},
//    {9.0750,   43.7000},
//    {10.1000,   51.2500},
//    {11.8840,   69.9600},
//    {13.8510,   84.2700},
//    {16.1200,  113.2000},
//    {18.9810,  144.8000},
//    {22.4060,  186.0000},
//    {26.7690,  228.3000},
//    {31.7230,  280.4000},
//    {37.3710,  324.7000},
//    {43.7270,  379.3000},
//    {50.1210,  433.0000},
//    {58.6250,  509.6000},
//    {69.7660,  563.2000},
//    {80.3170,  607.1000},
//    {90.5580,  654.7000},
//    {101.3690,  698.2000},
//    {111.9360,  725.6000},
//    {123.0600,  764.6000},
//    {134.9060,  764.7000},
//    {148.1230,  764.2000},
//    {160.6300,  745.6000},
//    {174.1020,  704.0000},
//    {185.0140,  640.6000},
//    {194.9920,  571.7000},
//    {204.5520,  472.7000},
//    {211.7580,  356.5000},
//    {230.0000,	0.000}
//};


/****** Declaration of private functions ******/
/*
 * Summary		Returns the interpolated value of the dependent variable of
 * 				a 2D table based on the input independent variable.
 * independent	The independent variable to base interpolation on.
 * table		The table interpolation is based on.
 * length		The length of <table>.
 * return		The interpolated value of the dependent variable.
 */
double tablesInterpolate( const double table[][2], uint16 length, double independent );


/******* Implementation of public functions********/
double tablesReadFromTable( tablesVariable_t variable, double independent )
{
	switch( variable )
	{
	case TABLES_VARIABLE_OMEGA:
		return tablesInterpolate( tablesCurrentVOmega, TABLES_LENGTH_CURRENT_V_OMEGA, independent );

	case TABLES_VARIABLE_TAU:
		return tablesInterpolate( tablesCurrentVTau, TABLES_LENGTH_CURRENT_V_TAU, independent );

	case TABLES_VARIABLE_POWER_OUT:
		return tablesInterpolate( tablesCurrentVPowerOut, TABLES_LENGTH_CURRENT_V_POWER_OUT, independent );

	default:
		break;
	}

	// Shouldn't reach this point
	return -99999.0;
}



/******* Implementation of private functions********/
double tablesInterpolate( const double table[][2], uint16 length, double independent )
{
	double fraction;
	uint16 iter;

	// Perform interpolation.  Interpolate dependent variable based on independent.

	// If input <independent> is less than the first (lowest) independent entry in the
	// table, return the dependent value corresponding to that first entry.
	if( independent < table[0][0] )
		return table[0][1];

	// Do same for last (highest) independent value of table.
	if( independent > table[length-1][0] )
		return table[length-1][1];

	// Cycle through the independent values of the table to find which independent
	// values <independent> lies within. Return a corresponding linearly interpolated
	// dependent value.
	for( iter=1; iter<length-1; iter++)
	{
		if( independent < table[iter+1][0] )
		{
			fraction = (independent - table[iter][0]) / (table[iter+1][0] - table[iter][0]);
			return table[iter][1] + fraction * (table[iter+1][1] - table[iter][1]);
		}
	}

	// This should never be reached.
	return -99999.0;
}
