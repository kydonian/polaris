//******************************************************************************
//
// Principal source code for TOROS capstone team.
// To be loaded onto MSP430FG4618 microcontroller.
//
// Nathaniel Honka
// BYU Capstone team 21
// 1.18.2012

// To do:
// Buzzer output routine			- not started
// Write documentation				- started, in docs

// Other's to do:
// Analog super lowpass for current



//*****************************************************************************
// Intrinsic headers
#include  <msp430xG46x.h>
#include  <math.h>
#include  <string.h>

// Explicitly defined headers
#include  "typedefs.h"
#include  "main.h"
#include  "storage.h"
#include  "control.h"
#include  "LCD.h"



/**********  Private functions prototypes **********/
void configureTiming(void);
void configurePins(void);
void configureADC(void);
void initializeGlobalVariables(void);
void configureLCD(void);


/********** Variable declarations *********/
// Note: 8KB available RAM.  Pre-allocate up to 4K, leaving 4K for the stack.
storageMemory_t storageMemory; 	// 2310 bytes
controlMemory_t controlMemory;  // ~16 bytes



/************ Entry point ************/
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;				// Disable watchdog timer

	// Initializations - private to main.c
	initializeGlobalVariables();			// Must execute first
	configureTiming();
	configurePins();
	configureADC();							// Note: must occur after pin configuration.
	configureLCD();							// Note: must occur after pin configuration.

	LCDDispayAllSegments();					// Display all LCD segments

	IE2 |= BTIE;                            // EnablInternal temperature sensore BT interrupt (main interrupt routine)
	_BIS_SR(GIE);							// Enable general interrupts (for ADC interrupts)

	while(1)
	{
		_BIS_SR(LPM0_bits + GIE);               // Enter LPM0, enable interrupts

	}

	// Note: At this point in the program, the processor enters low-power mode 3
	// and only returns to active mode when an interrupt is requested by the timer or the ADC interrupt vector.
	// The interrupts are based on the auxiliary clock, which is timed on a watch
	// crystal, so the interrupt frequency is known.  When an interrupt is signaled,
	//
	// Low power mode 3: All clocks except ACLK (32 kHz external watch crystal) disabled.

	// Main loop - hang out in low-power mode while waiting for interrupts

}





/************  Interrupt routines ************/

// Main interrupt routine

// Interrupts based on basic timer, which is timed off of the auxiliary clock (ACLK).
// NOTE: The timing of this function was constructed assuming counterMax = interrupt
// frequency - that is, the counter resets to 0 once per second.
#pragma vector=BASICTIMER_VECTOR
__interrupt void basic_timer_ISR(void)
{
	// If external selector switch is set to 'off' mode:
	if( controlMemory.opMode == CONTROL_OPERATION_MODE_OFF)
	{
		// Update operation mode based on external selector switch
		controlUpdateSelector( &controlMemory );
	}

	// If external selector switch is set to a winch mode:
	else
	{
		// Immediately start ADC12 conversion, so that measurements are taken at the specified
		// basic timer frequency.
		ADC12CTL0 |= ADC12SC;


		// Increment interrupt counter up to counterMax, then reset to 1
		controlMemory.counter %= controlMemory.counterMax;
		controlMemory.counter++;

		// Increment absolute counter
		controlMemory.absoluteCounter++;

		// Operations performed at 1 Hz
		if(  (controlMemory.counter % (controlMemory.counterMax/1) ) == 0 )
		{
			double dtTempDecay = 1.0;

			// Check if plow is connected
			controlUpdatePlow( &controlMemory );

			// Perform temperature decay estimation
			if( controlMemory.tempEstimationIsInitialized )
				controlDecayEstimatedTemp( &controlMemory, dtTempDecay );
		}


		// Operations performed at 4 Hz
		if(  (controlMemory.counter % (controlMemory.counterMax/4) ) == 0 )
		{
			// Update OAT
			controlUpdateOAT( &controlMemory, &storageMemory );

			// Update LED array
			controlUpdateLEDArray( &controlMemory );
		}


		// Operations performed at 8 Hz
		if(  (controlMemory.counter % (controlMemory.counterMax/8) ) == 0 )
		{
			// Update LCD
			controlUpdateLCD( &controlMemory );
		}


		// Operations performed at 32 Hz
		if(  (controlMemory.counter % (controlMemory.counterMax/32) ) == 0 )
		{
			static uint16 numReadings = MAIN_SELECTED_BT_FREQUENCY/32;
					// The number of readings between temperature estimate updates.

			// Raise the motor winding temperature estimate based on current
			// measurements in the last ( <numReadings> / MAIN_SELECTED_BT_FREQUENCY ) seconds.
			if( controlMemory.tempEstimationIsInitialized )
			{
				//controlRaiseEstimatedTempCurrentOnly( &controlMemory, &storageMemory, numReadings );
				//controlRaiseEstimatedTempPowerInferred( &controlMemory, &storageMemory, numReadings,  MAIN_SELECTED_BT_FREQUENCY);
				controlRaiseEstimatedTempPower( &controlMemory, &storageMemory, numReadings,  MAIN_SELECTED_BT_FREQUENCY);
			}

			// Function to blink system monitoring LED
			controlUpdateSystemLED( &controlMemory );
		}

		/*** Operations performed continuously (512 Hz) ***/

		// Enables winch use if not too hot or overcurrented.  Only perform once temperature
		// estimation routines have initialized.  If they have not initialized, disable winch.
		if( controlMemory.tempEstimationIsInitialized )
			controlEnableWinch( &controlMemory, &storageMemory );
		else
			CONTROL_PORT_ENABLE_WINCH &= ~CONTROL_PIN_ENABLE_WINCH;

		// Perform software reset if user has pressed switch 1
		controlCheckForReset();

		// Cycle LCD display mode if switch 2 is pressed
		controlCheckForLCDCycle( &controlMemory );

		// Update operation mode based on external selector switch
		controlUpdateSelector( &controlMemory );

		// Update instantaneous current & voltage readings stored in control struct
		controlUpdateCurrentAndVoltage( &controlMemory, &storageMemory );




		/*** One-time operations ***/

		// At MAIN_ADC_MEAS_FOR_TEMPERATURE_INIT executions of main ISR (basic timer) after
		// initialization.  Thus, the ADC will have measured temperature that many times.
		if(  controlMemory.absoluteCounter == (MAIN_ADC_MEAS_FOR_TEMPERATURE_INIT + 1) )
		{
			// Ensure OAT is updated ahead of time.  Possibly redundant, but just in case.
			controlUpdateOAT( &controlMemory, &storageMemory );
			controlInitTemperatureEstimation( &controlMemory );
		}
	}

	// At end of main interrupt, return to low-power mode 0
	__bic_SR_register_on_exit(LPM0_bits+ GIE);	// Exit to LPM0
}


// ADC storage interrupt routine

// At the beginning of the main interrupt routing, an ADC conversion sequence is triggered.  The
// ADC takes readings and stores them in its own memory registers.  Once the last register is ready
// to be read, this interrupt is triggered.  This interrupt transfers the information in the ADC
// registers to the heap for later usage.
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
	uint16 readADC[4];		// Stores ADC values before they are properly stored in 'storageMemory'

	// Move results, which clears ADC interrupt flags
	readADC[0] = ADC12MEM0;		// Channel 3  - Filtered armature current sensor
	readADC[1] = ADC12MEM1;		// Channel 4  - Armature voltage
	readADC[2] = ADC12MEM2;		// Channel 5  - Armature current sensor
	readADC[3] = ADC12MEM3;		// Channel 10 - Internal temperature sensor

	// Store values appropriately
	storageStoreValue( readADC[3], STORAGE_VARIABLE_TEMPERATURE, 	&storageMemory);
	storageStoreValue( readADC[2], STORAGE_VARIABLE_CURRENT, 		&storageMemory);
	storageStoreValue( readADC[1], STORAGE_VARIABLE_VOLTAGE, 		&storageMemory);
	storageStoreValue( readADC[0], STORAGE_VARIABLE_CURRENT_FILTERED, &storageMemory);

	__bic_SR_register_on_exit(LPM0_bits+ GIE);	// Exit to LPM0
}



/************  Low level initialization routines ************/

// NOTE: This code executes before variable initialization
void __low_level_init(void)
{
	WDTCTL = WDTPW + WDTHOLD;
	// Stop WDT - prevents WDT from resetting controller during variable
	// initialization (globals).  For example, if a very large array is declared
	// globally, the watchdog timer may time out before the processor
	// has time to allocate the array.

}

int _system_pre_init(void)
{
	WDTCTL = WDTPW + WDTHOLD;				// Disable watchdog timer
    return 1;
}



/************ Private function definitions ************/

// Configure clocks
void configureTiming(void)
{
	// Configure watch crystal capacitor
	FLL_CTL0 |= XCAP18PF;           // Set load cap for 32k xtal

	// Control sourcing of MCLK from DCO - set MCLK at ~8 MHz
	FLL_CTL0 |= DCOPLUS;            // DCO+ set, freq = xtal x D x N+1
	SCFI0 |= FN_4;                  // x2 DCO freq, 8MHz nominal DCO
	SCFQCTL = 121;                  // (121+1) x 32768 x 2 = 7.99 MHz

	// Setup basic timer interrupts
	if( MAIN_SELECTED_BT_FREQUENCY == 512 )
	{
		BTCTL = BT_512HZ;		// Set frequency of basic timer interrupt to 512 Hz.
		controlMemory.counterMax = 512;	// Set counter reset limit.  Program will be able to
										// perform periodic operations a minimum speed of
										// freq / max (Hz).
	}

	// If you've selected another frequency (besides 512 Hz), a lot of other things probably need to be changed.
}


// Configure pins as input, output as necessary
void configurePins(void)
{
	// Port		I/O/ADC		Description

	// 6.3		ADC			External OAT sensor voltage (i.e., thermistor)
	// 6.4		ADC			Armature voltage (divided)
	// 6.5		ADC			Current sensor voltage
	// 3.2		I 			Plow connected?
	// 3.3		O			Enable winch?
	// 3.4		I			External override engaged?
	// 3.5		O			Beeper
	// 3.7		I			User control switch on?
	// 2.1		O			System monitoring LED (blinks if processor is running)
	// 3.0		O			LED0	(LED array)	 |
	// 3.1		O			LED1				 |
	// 2.0		O			LED2				\ /
	// 3.6		O			LED3
	// 2.2		O			LED4
	// 2.3		O			LED5
	// 2.4		O			LED6
	// 2.5		O			LED7
	// 2.6		O			LED8
	// 2.7		O			LED9
	// 1.0		I			Switch 1	(low if pressed)
	// 1.1		I			Switch 2	(low if pressed)
	// 6.0		I			Winch selector 0 	(high to select 2500 lb winch)
	// 6.1		I			Winch selector 1	(high = 3500)
	// 6.2		I			Winch selector 2	(high = 4500)


	/*** Select port functions  	***/
	// Enable specific ports to be used for alternative functions (in this case, ADC).  If the
	// corresponding PxSEL bit for a port is set to 0, processor assumes that port will be used
	// for digital I/O.  If 1, processor assumes that port will be used for an alternative
	// function, such as ADC or LCD.  All ports are set to 0 by default.

	// Designate analog pins
	// Note: Voltage reference pins do not need to be designated because they are
	// dedicated (i.e., cannot be designated as digital I/O).
	P6SEL  |=	BIT3 | BIT4 | BIT5;


	// Designate LCD pins as necessary
	// Configure COM0-COM3 and R03-R33 pins
	P5SEL  |= 	BIT4 | BIT3 | BIT2;
	P5DIR  |= 	BIT4 | BIT3 | BIT2;

	// Designate digital I/O pins
	P1SEL = 0x00;
	P2SEL = 0x00;
	P3SEL = 0x00;

	/*** Configure digital I/O by specifying the direction of each port.  	***/
	/*** 1 = output, 0 = input.												***/

	P1DIR = 0x00;								// Only pins 0 & 1 are in use, and both are inputs
	P2DIR = 0xFF;								// All pins are outputs
	P3DIR = BIT0 | BIT1 | BIT3 | BIT5 | BIT6;	// Pins 2,4,7 are inputs
	P6DIR = 0x00;								// P6 I/O ports all inputs, special function, or unused
}


// Initialize global variables
void initializeGlobalVariables(void)
{
	storageInit( &storageMemory );
	controlInit( &controlMemory );
}


// Configure 12-bit ADC
void configureADC(void)
{
	// Declarations
	uint32 iterator;

	// TO DO:
	// When a set of conversions is processed, do I need to reset the conversion start address
	// specified by CSTARTADDx of ADC12CTL1?


	// ADC timing calculations
	// Input capacitance:				40 pF		(FG4618 documentation, 49)
	// Input resistance: 				2  kOhm		(FG4618 documentation, 49)
	// Maximum allowed load resistance:	34 kOhm		(user-specified, determines timing characteristics)
	// VREF+ internal voltage reference) external capacitance: 	10.1 uF (Experimenter board circuit diagram, pin 7)
	// REFON settling time:				17ms (MSP430x4xx User Guide, 28.2.3)
	// Oscillator:			Minimum 3.7 MHz, Max 6.3 MHz, nominally 5 Mhz (FG4618 documentation, 52)
	//						Note: assume 6.3 MHz for timing calculations to give appropriate timing margins.
	// Input sampling time (most ports)	:	ln(2^(n+1))*(Rs + Ri)*Ci + 800 ns (FG4618 documentation, 52)
	//									:	for current sensor: 13.776 us
	//									:	for voltage sensor: 91.911 us
	// Input sampling time (thermo)		: 30 us (FG4618 documentation, 53)
	// Input sampling cycles:			freq * duration = cycles
	//									6.3e6 * 91.9e-6 = 579 cycles (choose greatest time of all ADC inputs)
	//									=579 -> 768 cycles	(smallest of available clock cycle counts.  See MSP430x4xx_guser_guide.pdf, pg. 807)
	// Input conversion cycles:			13 cycles (FG4618 documentation, 52)
	// Total conversion time:			->  NOT SURE if this is true anymore... check docs: 23 ms (32 + 13*4 cycles, 3.7 MHz)

	// ADC input channel
	// Current sensor:			P6.5 / A5
	// Armature voltage:		P6.4 / A4
	// External temp sensor: 	P6.3 / A3
	// Internal temp sensor:	A10


	/**** Set parameters in ADC control registers ADC12CTL0 and ADC12CTL1 ****/
	// ADC12ON:		Turns ADC on.  Note, requires 100ns to settle.
	// MSC:			Multiple sample and conversion enabled. This makes it so once the ADC is
	// 				triggered, it will perform all of the requested conversions automatically
	//				in rapid succession.  In our case, it means that the ADC will sample all
	//				ADC channels as close to simultaneous as possible.
	// SHTx_11		Sampling period of 768 ADC12CLK cycles (see timing calculations above).
	// REFON		Enable internal voltage reference.
	// REF2_5V		Specify internal voltage reference to be 2.5V from AVss.
	// ~ENC			(inferred) Clear ENC (enable conversion) bit so ADC12 control bits can be
	//				modified.
	ADC12CTL0 = ADC12ON | MSC | SHT0_11 | SHT1_11 | REFON | REF2_5V;

	// SHP			Sampling trigger signal (SAMPCON) sourced from the sampling timer (which, in
	//				case, is specified manually by setting the ADC12SC (start conversion) bit.
	//				Optionally, this could be sourced from Timer A.
	// SHS_0		Sample-and-hold source selected to be ADC12SC bit (start conversion).  That is,
	// 				sampling is triggered automatically.
	// CSTARTADD0	Conversion start address = ADC12MEM0.  That is, the first conversion will be
	//				stored in ADC12MEM0, and subsequent conversions will stored in ADC12MEM1,
	//				ADC12MEM2, etc.
	// ADC12SSEL0	Select source of ADC12 clock (ADC12CLK signal) to be ADC12OSC, which is the
	//				ADC's own internal oscillator.
	// CONSEQ_1 	Specify conversion mode to be single conversion, sequence of channels.  That is,
	//				the ADC will convert a series of channels in rapid succession so that measurements
	//				are effectively simultaneous.
	ADC12CTL1 = SHP | SHS_0 | CSTARTADD_0 | ADC12SSEL_0 | CONSEQ_1;



	/**** Configure ADC12 conversion memory control registers ****/
	// INCH_xx: 	Analog channel ( 10 = internal temperature sensor) that register will read from
	// SREF_1:		Specifies which voltages are used as references in conversion.
	//				ref+ = VREF+ (internal voltage reference), ref- = AVss
	// EOS 			Specifies that this ADC memory register is the last in a conversion sequence.  When
	//				the ADC is in the process of capturing in sequence and register with this bit set
	//				is reached, the sequence is ended.
	ADC12MCTL0 = INCH_3 	| SREF_1;		// External temp sensor
	ADC12MCTL1 = INCH_4 	| SREF_1;		// Voltage
	ADC12MCTL2 = INCH_5 	| SREF_1;		// Armature current
	ADC12MCTL3 = INCH_10 	| SREF_1 | EOS;	// Internal temperature sensor


	/**** Configure ADC12 interrupt enable register ****/
	// ADC12IEx		Allows specified memory register to request an interrupt by setting its
	// 				corresponding ADC12IFGx flag.  The ADC12IFGx flag is set with the corresponding
	//				ADC12MEMx is loaded with a conversion result.  The ADC12FGx flag is cleared once
	//				ADC12MEMx is accessed.
	ADC12IE = BIT3;		// Since ADC12MEM3 is the last memory register to be loaded, once it is converted,
						// all ADC12MEM is ready to be read & stored.  Therefore, it signals the interrupt
						// that stores off the data in the ADC registers (ADC12ISR, see above).

	// Set ENC to enable conversions.  Once ENC is set, ADC12 co ntrol bit cannot be modified.
	// ENC is not reset automatically (ADC12SC, start conversion, is reset automatically).
	ADC12CTL0 |= ENC;	// Note the OR so as not to reset the other bits.

	// Final step
	// Delay 20 ms for internal voltage reference and ADC to settle on (ADC only needs 100 ns).
	for(iterator = 0;  iterator > 500000; iterator++)
	{
		//160000 clock cycles at 8MHz = 63ms
	}

}

// Configure LCD_A
void configureLCD(void)
{
	// Clear LCD memory
	LCDClear();

	// Configure LCD_A
	// Not really sure what all these do, but it's in the MSP430x4xx user
	// guide. I just copied them from an LCD example.
	LCDACTL = LCDFREQ_128 | LCDMX1 | LCDMX0 | LCDSON | LCDON;
	LCDAPCTL0 = LCDS4 | LCDS8 | LCDS12 | LCDS16 | LCDS20 | LCDS24;
	LCDAPCTL1 = 0;
	LCDAVCTL0 = LCDCPEN;
	LCDAVCTL1 = VLCD_2_60;
}
