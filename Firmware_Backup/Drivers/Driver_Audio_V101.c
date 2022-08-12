//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2013-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Driver_Audio_V101.c
// Author(s)		: Fabian Kung
// Last modified	: 7 Feb 2019
// Toolsuites		: Microchip MPLAB-X IDE v5.10 or above
//                	  MPLAB-x XC16 C-Compiler v1.33 or above

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//
INT16 gnAudioTone[9];                       // Set the tone, from 1 to 15.  0 or otherwise will stop the speaker.
                                            // We can have up to 8 different tones.  The last tone setting should
                                            // be 0.  See example of usage for Proce_Audio_Driver.
unsigned int gunAudioVolume;                // From 0 to 5 (loudest).

//
// --- PRIVATE FUNCTION PROTOTYPES ---
//

void SetAudioProperty(int nPeriod, int nDutyCycle);

//
// --- PRIVATE VARIABLES ---
//
//
// --- Process Level Constants Definition ---
//
#define T2_PS	64							// TIMER2 Prescalar.

#define DUTY_CYCLE_CONST    0.01*T2_PS

#define	SOUND_0_5KHZ	(1/(0.5*T2_PS*0.001*__TCLK_US))-1		// TIMER2 count to generate 0.5kHz.
                                                                // NOTE: This driver doesn't work well
                                                                // with 0.5kHz setting, 22 Jan 2013.
#define	SOUND_1KHZ	(1/(1.0*T2_PS*0.001*__TCLK_US))-1		// TIMER2 count to generate 1kHz.
#define	SOUND_1_5KHZ	(1/(1.5*T2_PS*0.001*__TCLK_US))-1		// TIMER2 count to generate 1.5kHz.
#define	SOUND_2KHZ	(1/(2.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 2kHz.
#define	SOUND_2_5KHZ	(1/(2.5*T2_PS*0.001*__TCLK_US))-1		// TIMER2 count to generate 2.5kHz.
#define	SOUND_3KHZ	(1/(3.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 3kHz.
#define	SOUND_3_5KHZ	(1/(3.5*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 3.5kHz.
#define	SOUND_4KHZ	(1/(4.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 4kHz.
#define	SOUND_4_5KHZ	(1/(4.5*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 4.5kHz.
#define	SOUND_5KHZ	(1/(5.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 5kHz.
#define	SOUND_5_5KHZ	(1/(5.5*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 5.5kHz.
#define	SOUND_6KHZ	(1/(6.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 6kHz.
#define	SOUND_6_5KHZ	(1/(6.5*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 6.5kHz.
#define	SOUND_7KHZ	(1/(7.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 7kHz.
#define	SOUND_8KHZ	(1/(8.0*T2_PS*__TCLK_US*0.001))-1		// TIMER2 count to generate 8kHz.


//
// --- Microcontroller Pin Usage ---
//

///
/// Function name	: SetAudioProperty
///
/// Author		: Fabian Kung
///
/// Last modified	: 22 Jan 2013
///
/// Code Version	: 1.00
///
/// Description		: Subroutine to set the properties of the Audio Driver.  Which uses the Output Compare 
///                   Module 3 (OC3) and Timer 2 of the dsPIC33E controller to generate a periodic square
///                   wave with variable frequency and dutycycle. The OCM is driven by peripheral clock (Fp).
///                   See further explanation below.
///
/// Arguments		: nPeriod = Period of the square, from 200 to 65000.  Actual duration depends on the
///                   CPU clock speed.
///			          nDutyCycle = from 0 to 100 (%). A value of 0 will shutdown the audio driver.
///
/// Further Explanation:
/// Unlike the Output Compare modules in dsPIC33F, the OCx (x = 1 to 4) module in dsPIC33E has
/// an internal timer, here it is OCxTMR, and the values in registers OCxR and
/// OCxRS are compared to OCxTMR.  This internal timer can be reset by external event,
/// called the syncronization source. Here the synchronization source is TIMER2.
/// Whenever TIMER2 overflows the internal timer OCxTMR will be reset.
/// Here we set OC3 module to "Dual Compare Continuos Pulse" mode.  The CPU peripheral timer
/// Fp is used to provide clock source to drive OC3TMR, where Fp = Fosc/2.  
/// OC3TMR is incremented from CPU peripheral clock and it's value is compared with OCR3 register.
/// When matched, OC3 pin is set to high.  When OC3TMR matches OC3RS, OC3 pin is set low.
///
/// Initially:
/// 1) The output pin of OC3 is set Low.
/// 2) OC3R and OC3RS are initialized such that the values in OC3RS > OC3R. Here we will
///    set OC3R = 0, and OC3RS > 0.
/// 3) TIMER2 is loaded with a pre-determined value (as in PIR2).
/// 4) OC3TMR is reset.
///
/// Since OC3TMR = OC3R = 0 initially, pin OC3 will be set High.  OC3TMR will then
/// be incremented every clock cycle, when OC3TMR = OC3RS, pin OC3 will be set Low.
/// OC3TMR will be incremented every clock cycle until it is reset (by TIMER2 overflow),
/// and the cycle repeats.
///
/// The pulse period will be: (T2_Value+1) x T2_prescalar x Tclk.
/// The duty cycle will be: Pulse_Period x T2_prescalar x Duty_Cycle


void SetAudioProperty(int nPeriod, int nDutyCycle)
{
    int nTemp;

    if (nDutyCycle > 100)	// Check for over limits.
    {
        nDutyCycle = 100;
    }
    if (nDutyCycle < 1)
    {
        T2CONbits.TON = 0;              // Turn off TIMER2.
        OC3CON1bits.OCM = 0x0;          // Shut down audio driver.
    }
    else if (nPeriod > 0)
    {
                                        // The period of one pulse = nPeriod (from 100 to 65000).
                                        // Thus we convert the duty cycle from 0-100 to a fraction
                                        // of nPeriod.
        nTemp = nPeriod*(DUTY_CYCLE_CONST*nDutyCycle);        
                                         
        // --- Turn on OC3 and TIMER2 module ---
        //OC3RS = 1000;
        OC3RS = nTemp;                   // Update OC3 secondary Register.
        OC3R = 0;                        // Update OC3 main Register.

        T2CONbits.TON = 1;               // Turn on TIMER2.
        PR2 = nPeriod;                   // Reload TIMER2 compare register.
        TMR2 = 0x0000;                   // Reset TIMER2.

        OC3TMR = 0x0000;                 // Clear OC3 internal counter.
        OC3CON1bits.OCM = 0x5;           // Set to dual compare continuous pulse mode.
    }
}




///
/// Function name	: Proce_Audio_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 9 March 2019
///
/// Code Version	: 1.04
///
/// Processor/System Resource 
/// PINS            : 1. Pin RE7 = OC3 (Remappable I/O RP87), output.
///  				 
///
/// MODULES         : 1. TIMER2 (Internal, 16-bit).
///                   2. OC3 (Internal).
///
/// RTOS            : Ver 1 or above, round-robin scheduling.
///
/// Global Variables : gnAudioTone[]
///                    gunAudioVolume

#ifdef __OS_VER			// Check RTOS version compatibility.
	#if __OS_VER < 1
		#error "Proce_Audio_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_Audio_Driver: An RTOS is required with this function"
#endif

///
/// Description	: Speaker control.
///               This routine works by using the Output Compare Module (OCM) of the dsPIC controller to generate
///               a periodic square wave with variable duty cycle and frequency.  The OCM output in turn drives
///               an electromagnetic or piezoelectric transducer. The audio mode is determined by the global 
///		          variable gnAudioTone[] array.
///               Initially the index is 0, and this refers to the first value in the gnAudioTone[] array.
///               Valid values is from 1 to 15. For each value, a beep is generated, and the
///               frequency of the audio beep is dependent on the value store in gnAudioTone[].  A value of 16 or larger
///               will pause the speaker for a duration of a beep.  The loudness of the tone is controlled by 
///               global variable gunAudioVolume, a value of 0 turn off the speaker, and 1 to 5 indicating the volume level,
///               with 5 being the loudest.
///               After this beep is generated, the index is incremented and refers to the next value in the array.  This repeats
///               until the maximum number of element in the array is reached or a value of 0 or negative is encountered.
///               The index is then reset to 0 and the routine waits until a non-zero is written in the first element
///               of gnAudioTone[] array to start the sequence again.
///
/// Example of usage:
///               gnAudioTone[0] = 1;
///               gnAudioTone[1] = 3;
///               gnAudioTOne[2] = 100;
///               gnAudioTone[3] = 1;
///               gnAudioTone[4] = 0;
///               gunAudioVolume = 4;
///
/// The example above will produce two beep corresponding to the value 1 and 3, then pause for a short while,
/// then beep for another time corresponding to the value 1, and finally the audio module will be
/// disabled, until a non-zero value is written into gnAudioTone[0] again.


void Proce_Audio_Driver(TASK_ATTRIBUTE *ptrTask)
{
    static int nIndex = 0;
    static int nDutyCycle = 50;
    
    if (ptrTask->nTimer == 0)
    {
	switch (ptrTask->nState)
	{
            case 0: // State 0 - Initialization of OC3 module and pins.
                    // Setup IO pins mode.
                    // Settings of remappable output pin. The cluster of bits RPnR
                    // control the mapping of the pin internal peripheral output.
                    // Note n = integer.  If we refer to the datasheet, pin RE7
                    // is also called RP87, and RP87R bits are contained in
                    // the special function register RPOR6 in the controller.
                
                RPOR6bits.RP87R = 0b010010;         // RP87 or RE7 connected to OC3's output.
                OC3CON1 = 0x0000;                   // According to the datasheet it is a
                OC3CON2 = 0x0000;                   // good habit to set all control bits to 0 first.
                OC3CON1bits.OCSIDL = 1;             // Output Compare 3 halts when in idle mode.
                OC3CON1bits.OCTSEL = 0x7;           // Use peripheral clock (Tclk/2) as clock source.
                OC3CON2bits.SYNCSEL = 0b01100;      // TIMER2 synchronizes or triggers OC3.
                T2CONbits.T32 = 0;                  // TIMER2 and TIMER3 act as 2 16-bits timers.
                T2CONbits.TSIDL = 1;                // Discontinue TIMER2 operation with in idle mode.
                T2CONbits.TCKPS = 0b10;             // TIMER2 clock input divide by 64 prescaler.
                T2CONbits.TCS = 0;                  // Internal clock source.
                OSSetTaskContext(ptrTask, 1, 1000); // Next state = 1, timer = 1000.
                nIndex = 0;                         // Reset the index.
                gnAudioTone[0] = 0;
                gunAudioVolume = 5;
            break;
			
            case 1: // State 1 - Check the first element of the gnAudioTone array.
                if ((gnAudioTone[nIndex] > 0) && (nIndex < 9))
                {   
                    if (gunAudioVolume > 5)         // Limit the maximum volume.
                    {
                        gunAudioVolume = 5;
                    }
                    nDutyCycle = gunAudioVolume * 10;
                    switch (gnAudioTone[nIndex])
                    {
                        case 1:
                            SetAudioProperty(SOUND_0_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 2:
                            SetAudioProperty(SOUND_1KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 3:
                            SetAudioProperty(SOUND_1_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 4:
                            SetAudioProperty(SOUND_2KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 5:
                            SetAudioProperty(SOUND_2_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 6:
                            SetAudioProperty(SOUND_3KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 7:
                            SetAudioProperty(SOUND_3_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 8:
                            SetAudioProperty(SOUND_4KHZ,nDutyCycle);	// Generate a beep.
                        break;

                       case 9:
                            SetAudioProperty(SOUND_4_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 10:
                            SetAudioProperty(SOUND_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 11:
                            SetAudioProperty(SOUND_5_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 12:
                            SetAudioProperty(SOUND_6KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 13:
                            SetAudioProperty(SOUND_6_5KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 14:
                            SetAudioProperty(SOUND_7KHZ,nDutyCycle);	// Generate a beep.
                        break;

                        case 15:
                            SetAudioProperty(SOUND_8KHZ,nDutyCycle);    // Generate a beep
                        break;

                        default:
                            SetAudioProperty(SOUND_8KHZ,0);    // Off for the duration of a beep.
                    }
                    nIndex++;                                   // Next beep.
                } 
                else
                {
                    nIndex = 0;                                 // Reset index.
                    SetAudioProperty(SOUND_8KHZ,0);             // Off audio generator.
                    gnAudioTone[0] = 0;                         // Disable beep.
                }
                OSSetTaskContext(ptrTask, 1, 700);              // Back to state = 1, timer = 700.
            break;
			
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}

 