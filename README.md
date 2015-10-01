## Winch motor burnout preventer with TI MSP430 ##

This was a senior design project created for Polaris Industries (manufacturer of ATVs, snowmobiles, etc).  The aim of the project was to prevent inexpert users from inadvertently burning out a medium-power (120W) DC motor by estimating motor temperature, disabling the user's on/off switch when the motor overheated, and reenabling the switch when the motor had cooled sufficiently.  Temperature was estimated by measuring motor current, voltage, and ambient air temp and inferring heat generation/transfer.

The solution was implemented on an MSP430 microprocessor (16-bit, 8MHz clock freq, 8K RAM) dev board with a small external circuit for measurements, power supply, and physical control.

I did all firmware & control design for the project.

The project is a good example of firmware design.  The code I wrote demonstrates:
* what I consider "low-level" implmentation in firmware of robotics-related technologies
    * determinisitic (real-time) execution timing, accomplished by tying the processor clock speed to an external crystal oscillator then using a hardware timer to trigger an interrupt
    * ADC register timing calculations, configuration, & verification - ADC input register must be given sufficient time to settle within bit precision, but must be sampled quick enough for the application (MSP430 permits software selection of ADC capacitance).
    * management of microcontroller power modes
* good coding practices
    * well-documented code
    * consistent coding standards
    * object-oriented design insofar as the language \(C\) permitted

You'll find relevant code in `main.c/h` and `control.c/h`.

I should point out that this project is *not* a very good example of control design.  Knowing what I do now about control, estimation, & sys ID I would update the control scheme signficantly.  For example, the physical system was clearly a dominantly second-order plant with significant nonlinear dynamics, but at the time I modeled it as a simple first-order system, knowing it was pretty inaccurate but hoping it would be "good enough".  A current redesign would likely incorporate a UKF or at least EKF for more accurate state estimation, extra sensing of a measurable state, and more extensive physical model verification.

When I made this project I used Code Composer Studio (an Eclipse-based IDE from TI) its built-in toolchain to compile and on-chip debug.  I believe the toolchain was a version of GCC tailored for TI's chips, which you can find [here](http://www.ti.com/tool/msp430-gcc-opensource).
