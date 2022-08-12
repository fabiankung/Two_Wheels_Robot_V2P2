//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_ADC_DAC_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 9 March 2019
// Tool-suites		: Microchip MPLAB-X IDE v5.10 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above
//			

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//
// ADC Module:
typedef struct StructProceADCDriver
{
    UINT16   unADC1_mV;         // ADC Channel 1 output.
    UINT16   unADC2_mV;         // ADC Channel 2 output.
    UINT16   unADC3_mV;         // ADC Channel 3 output.
    UINT16   unSampleCount;     // Counter to keep track of ADC sample.
    UINT16   unEnADC;           // Set greater than 0 to enable the ADC module.
    UINT16   unSamplingInterval_us;   // Sampling interval of ADC in micro-seconds.
} DSPIC33E_ADC_DRIVER;

DSPIC33E_ADC_DRIVER     gobjDriverADC;
UINT16 gunADChanSel = 1;		// Public data.	 Select ADC Channel (1 to 16 depending on driver support).

// DAC Module
typedef struct StructProceDACDriver
{
    UINT16   unDAC1_mV;         // ADC Channel 1 output.
    UINT16   unDAC2_mV;         // ADC Channel 1 output.
    UINT16   unEnDAC;           // Set greater than 0 to enable the DAC module.
    UINT16   unSamplingInterval_us;   // Sampling interval of DAC in micro-seconds.
} DSPIC33E_DAC_DRIVER;

DSPIC33E_DAC_DRIVER     gobjDriverDAC;

//UINT16 gunRunDAC = 0;           // Public data, 0 - DAC disabled.
                                //              otherwise - DAC enabled.
//UINT16 gunDACIn1_mV = 0;		// Public data.  DAC-A output in milivolts.
//UINT16 gunDACIn2_mV = 0;		// Public data.	 DAC-B output in milivolts.

//
// --- PRIVATE FUNCTION PROTOTYPES ---
//

//
// --- PRIVATE VARIABLES ---
//

//
// --- Process Level Constants Definition --- 
//
#define	_VREF	2.048	// Reference voltage for ADC module, in Volts.
// Note: The ADC output register ADC1BUF0, under 12-bits output it is given by:
//       	 ADC1BUF0 = (Vin/Vref)x4096. 
//
//       For Vref = 2.048V, the following procedures will convert the ADC1BUF0 value into
//		 milivolts, where gunADCOutx_mV is the external register storing the result.
//       	gunADCOutx_mV = ADC1BUF0/2.    

///
/// Process name	: Proce_ADC_Driver
///
/// Author			: Fabian Kung
///
/// Last modified   : 9 March 2019
///
/// Code Version	: 1.022
///
/// Processor		: dsPIC33EP256MU80X family.
///                   dsPIC33EP512MC80X family.
///
/// Processor/System Resources 
/// PINS			: 1. Pin RB0 = Vref+ input (optional).
///                   2. Pin RB11 = Analog input (AN11), ADC input Channel 0 (Not used for now)
///                   2. Pin RB12 = Analog input (AN12), ADC input Channel 1.
/// 				  3. Pin RB13 = Analog input (AN13), ADC input Channel 3.
///                   4. Pin RB14 = Analog input (AN14), ADC input Channel 2.
/// 
/// MODULES			: 1. ADC1 (Internal).
///                   2. Precision reference voltage source for _VREF (External).
///                      At present the reference is provided 2.048V bandgap reference
///                      chip.
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gobjDriverADC
///                       gunADChanSel - ADC analog input channel select.


#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_ADC_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_ADC_Driver: An RTOS is required with this function"
#endif

/// Description	: Subroutine to setup and drive the built-in ADC module in the dsPIC33EP family
///		  microcontroller.
///               ADC Output Specifications:
///               1) 12 bits result in unsigned integer format.
///               2) 3 channels - AN12 (ADC1), AN11 (ADC2) and AN10 (ADC3).
///               3) Input voltage limits from 0V to Vref (see constant _VREF above).
///               4) Sampling period for each channel -  6 x _SYSTEMTICK_US, or 1 msec for 166.667usec system tick.
///               5) Output registers - unADC1_mV (ADC1), unADC2_mV (ADC2) and unADC3_mV (ADC3).
///               The value in output registers are in milivolts.
///                    
/// Example of usage: 
/// 1) To start the sampling and conversion, we first set gobjDriverADC.unEnADC to 1.  The process will then
/// automatically samples the analog input channels ADC1, ADC2 and ADC3 alternately. The result of each sample
/// will be stored in the respective global variables, unADCx_mV, where x = 1, 2 or 3.  
/// A global variable unSampleCount will also be incremented for each sample, so this can be used to track 
/// if a new sample has arrived or not. 
/// 2) The result in unADCx_mV will be scaled to milivolts, between 0V to Vref. unADCx_mV is a 16-bits 
/// unsigned integer. 
///


void Proce_ADC_Driver(TASK_ATTRIBUTE *ptrTask)
{
 unsigned long uLTemp;
 
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization and configure the 12-bits Analog-to-Digital Converter, called ADC1.
                TRISBbits.TRISB0 = 1;	// Set RB0 as input (to be used as external Vref+ input if needed).
                TRISBbits.TRISB11 = 1;  // Set RB11 as input.
                TRISBbits.TRISB12 = 1;	// Set RB12 as input.
                TRISBbits.TRISB13 = 1;	// Set RB13 as input.
                TRISBbits.TRISB14 = 1;	// Set RB14 as input.
                ANSELBbits.ANSB0 = 1;   // RB0 as analog input.
                ANSELBbits.ANSB11 = 1;  // RB11 as analog input.
                ANSELBbits.ANSB12 = 1;  // RB12 as analog input (1 = analog, 0 = digital).
                ANSELBbits.ANSB13 = 1;  // RB13 as analog input.
                ANSELBbits.ANSB14 = 1;  // RB14 as analog input.
                AD1CON1 = 0x2404; 	// Discontinue ADC module when device enters idle mode.	
					// DMA buffers (if used) are written in scatter/gather mode.
					// 12 bits, 1 sample-and-hold ADC operation.
					// Output in unsigned integer form.
					// Manual conversion mode, clearing SAMP bit ends sampling and 
					// starts conversion.
					// Sampling begins immediately after last conversion (ASAM bit=1).
                //AD1CON2 = 0x0000; // ADC + reference = AVdd.
                AD1CON2 = 0x2000; 	// ADC + reference = External Vref+ pin.
					// ADC -ve reference = AVSS pin.
					// No input scanning.
					// Increment DMA address (if used) after every sampling/conversion.
					// Always start filling buffer at address 0x00.

					// Configure ADC conversion clock: Tad=(ADCS+1)*Tcyc							

		#define	 _TAD_US	0.15                // Minimum Tad for dsPIC33EP is 117.6nsec or 0.1176usec.
		#define	 _ADCS_		_TAD_US/__TCLK_US   // ADC clock.
			
                AD1CON3 = _ADCS_; 	// Set ADC1 clock.
					// Sampling time = 0 Tad.
					// For Tcyc=16.7nsec, this translates into Tad = 150.0nsec.
					// For 12 bits ADC mode, a total conversion time of 14Tad=2.1usec is needed.
				 	// NOTE: I discovered that a sufficient long delay between start of 
				 	// sampling and start of conversion.  Not mentioned in the datasheet,
				 	// but I believe it to be 1xTad.  If this is not adhered to the 
				 	// ADC will not work properly, e.g. the DONE and ADIF bits will 
				 	// not be set.
                AD1CON4	= 0x0000;	// Allocate 1 word of DMA buffer (if used) to each analog input.
                AD1CHS0 = 0x000C;  	// Configure Sample-and-Hold (S/H) channel: Channel 0 (CH0)
					// MUXA: positive input is AN12, CH0 negative input is VREFL (AVss).
					// MUXB is not used (unless ALTS bit = 1).
					// See datasheet.  There is only 1 S/H channel, called CH0 for 12-bits
                                        // ADC conversion.
                //	AD1CON2bits.ALTS = 1;
                AD1CSSL = 0x0000; 	// Skip all analog inputs for scanning.
                IFS0bits.AD1IF = 0;     // Clear ADC1 convert complete interrupt flag.				
                OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.						 
            break;

            case 1: // State 1 - Turn on ADC and begin sampling.
                gobjDriverADC.unSampleCount = 0;	// Reset sample counter.
                gobjDriverADC.unSamplingInterval_us = 1000; // Set sampling interval.
                AD1CON1bits.ADON = 1; // Turn on ADC.
                AD1CON1bits.SAMP = 1; // Start sampling.
                OSSetTaskContext(ptrTask, 2, 1); // Next state = 2, timer = 1.					 
            break;

            case 2: // State 2 - Wait for start conversion signal and start sampling of analog channels.
                if (gobjDriverADC.unEnADC > 0)
                {
					
                    AD1CON1bits.SAMP = 0; 	// End sampling and start conversion.
				  		// Note that 14Tad is needed for 12-bits conversion and 
				  		// 12Tad is needed for 10-bits conversion.
                    OSSetTaskContext(ptrTask, 3, 1); // Next state = 3, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 2, 1); // Next state = 2, timer = 1.
                }						 
            break;

            case 3: // State 3 - Perform conversion sequence on analog input.  Here Vref+ is 2.048V,
	  	    // and Vref- is 0V.  Sample THREE analog channels alternately.
		    //if (IFS0bits.AD1IF == 1)
                if (AD1CON1bits.DONE == 1)
                {
                    IFS0bits.AD1IF = 0; 	// Clear ADC convert complete interrupt flag.

                    // --- Insert user routines here ---
                    uLTemp = ADC1BUF0>>1;               // Get ADC buffer content, divide by 2 to normalize to 
                                                        // millivolts.
                    //uLTemp = ADC1BUF0; 
                                                        // For Vref = 2.048V, 12-bits ADC:
                                                        // deltaV = Vref/(2^12 - 1) = 2.048/4095 = 0.5 milivolts per unit.
                                                        // Thus if input is 1V, we would get an ADC output of 2000.
                                                        // This needs to be divided by 2 to yield 1000, or 1000 mV.
                                                        // If Vref = 4.096V, then we do not need to divide by 2.
                    uLTemp = uLTemp & 0x0FFF;			// Only the first 12 bits are valid.
                    if (gunADChanSel == 1)
                    {
                        gobjDriverADC.unADC1_mV = uLTemp;			// Read Channel 1 conversion result. 
								// (only first 12 bits are valid).
                    }
                    else if (gunADChanSel == 2)
                    {
                        gobjDriverADC.unADC2_mV = uLTemp;			// Read Channel 2 conversion result. 
								// (only first 12 bits are valid).
                    }
                    else
                    {
                        gobjDriverADC.unADC3_mV = uLTemp;			// Read Channel 3 conversion result. 
								// (only first 12 bits are valid).
                    }

                    // Select new input channel.
                    if (gunADChanSel == 1)
                    {
                        AD1CHS0 = 0x000D;	// Sample channel 2: AN13 (RB13).
                        gunADChanSel = 2;	// Set channel indicator to channel 2.
                    }
                    else if (gunADChanSel == 2)
                    {
                        AD1CHS0 = 0x000E;	// Sample channel 3: AN14 (RB14).
                        gunADChanSel = 3;	// Set channel indicator back to channel 3.
                    }
                    else // Channel 3, AN14.
                    {
                        AD1CHS0 = 0x000C;	// Sample channel 1: AN12 (RB12).
                        gunADChanSel = 1;	// Set channel indicator back to channel 1.
                    }

                    AD1CON1bits.DONE = 0; 	// Clear DONE bit (actually the hardware will do this automatically
                                         	// when we clear the SAMP bit.
                    AD1CON1bits.SAMP = 1; 	// Start new sampling (on AN11 or AN12).

                    gobjDriverADC.unSampleCount++;	// Increment sampling counter.
                    OSSetTaskContext(ptrTask, 2, 1); // Next state = 2, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 3, 1); // Next state = 3, timer = 1.
                }	
            break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}


//
// --- Microcontroller Pin Usage ---
// Pin Name                         uC Pin                External Module Pin Function             uC Pin Function
// --------                         ------                ----------------------------             --------------
#define PIN_12DAC_Driver_LDAC		_RD9	// Pin RD9 = Load Data, output.
#define PIN_12DAC_Driver_SCK		_RD7	// Pin RD7 = SCK (Remappable I/O RP71), output.      Note: uC pin set to SCK, output.
#define	PIN_12DAC_Driver_CS         _RD8	// Pin RD8 = &CS& (Remappable I RPI72), output.
#define PIN_12DAC_Driver_SDI		_RD6	// Pin RD6 = SDI (Remappable I/O RP70), input.       Note: uC pin set to SDO, output.

//
// --- Process Level Constants Definition --- 
//
//#define	_12DAC_Vo_MAX	2047	// Allowable maximum output voltage (in mV for x1 gain).  
#define	_12DAC_Vo_MAX	4095	// Allowable maximum output voltage (in mV for x2 gain).
//#define         _12DAC_Vo_MAX	3100	// This is also power supply dependent, for instance if Vcc = 3.3V, it
					// is then not possible for the output to be higher than this level.
#define         _12DAC_Vo_MIN	0	// Allowable minimum output voltage (in mV).

///
/// Function name	: Proce_12DAC_MCP4822_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 9 March 2019
///
/// Code Version	: 1.02
///
/// RTOS            : Ver 1 or above, round-robin scheduling.
///
/// Processor		: dsPIC33EP256MU806 family.
///                   dsPIC33EP512MC80X family.
///
/// Processor/System Resources 
/// PINS             : See above.
///
/// MODULES			 : 1. SPI1 (Internal).
///                    2. MCP4822 12 bits DAC chip (External)
///
/// RTOS			 : Ver 1 or above, round-robin scheduling.
///
/// Global variables : gobjDriverDAC

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_12DAC_MCP4822_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_12DAC_MCP4822_Driver: An RTOS is required with this function"
#endif

///
/// Description	: SPI driver for external 12 bits DAC chip - MCP4822 from Microchip Technologies.
///
/// Specifications:
/// 1) MCP4822 contains two 12-bits DAC modules, DAC-A and DAC-B.  Each DAC is controlled by a 
/// 16-bits register.  The lower 12 bits set the DAC output, while the upper 4 bits set the mode.
/// This chip has an internal voltage reference set at 2.048 Volts, thus with x1 gain, the output 
/// 	
///	   Vout = (D/4096)Vref, where D = data word (12 bits) 0-4095, Vref = 2.048V.
///
///    With x2 gain the output range is 0-4.096V. 
///	   Vout = (D/4096)(2Vref)
///
///    In summary with gain = x1, the resolution is 500uV.  With gain = x2, the resolution is 1mV.
///
/// 2) Here the internal module SPI1 is used to drive the DAC chip.
/// 3) SPI max bit rate = 10 MBps.
///
/// 16 bit data word for MCP4822:
/// Bit 15: 1 = Select DAC-B, 0 = Select DAC-B.
/// Bit 14: Not used.
/// Bit 13: 1 = Gain x1, 0 = Gain x2.
/// Bit 12: 1 = active, 0 = shutdown.
/// Bit 11-0: DAC data.
/// See the up-to-date datasheet for more information.
///
/// Since there are two DAC modules, a total of 32-bits (2 words) needs to be shifted into the chip
/// via SPI interface.  After this is done the LDAC pin on the chip is pulsed to transfer the command
/// words from the holding register into the DAC modules.
///
/// Example of Usage:
/// Global registers gunDAC1_mV and unDAC2_mV, in unsigned 16 bit format provide the inputs to the
/// DAC Channel A and B, in mV units.  Note that only 12 bits are used, the upper 4 bits are set to zero.  
/// Thus the
/// DAC output goes from 0 to 4.095V (b'11111111111' = decimal 4095).  We can treat the values in both
/// registers as containing the DAC output in milivolts.  However the max and min output voltage will 
/// be determined by the internal constants _12DAC_Vo_MAX and _12DAC_Vo_MIN to prevent overflow.
///
/// Setting outputs of DAC-A and DAC-B to 1.5V:
///         gobjDriverDAC.unEnDAC = 1;      // Enable the DAC module.
///         unDAC1_mV = 1500;
///         unDAC1_mV = 1500;
///

void Proce_12DAC_MCP4822_Driver(TASK_ATTRIBUTE *ptrTask)
{
 static int nDAC1, nDAC2;	// Command words for DAC-A and DAC-B modules.
 int nTemp;
 unsigned int unDAC1, unDAC2;
 
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Checking the state of gunRunDAC and initialization of SPI2.
                    // Here we are setting the bit rate to 7.5 MBps.
                if (gobjDriverDAC.unEnDAC == 0)     // Skip if unEnDAC = 0.
                {
                    gobjDriverDAC.unSamplingInterval_us = 1000; // Set sampling interval.
                    OSSetTaskContext(ptrTask, 0, 1); // Next state = 0, timer = 1.
                    break;
                }
                nDAC1 = 0xF000;
                nDAC2 = 0xF000;
                                        // Settings of remappable I/O. Here we will to map RP71 (pin RD7) to
                                        // SCK1 and RP70 (pin RD6) to SDO1.  The mapping of RP71 is controlled
                                        // by cluster of bits named RP71R (<13:8>) in the special function register 
                                        // (SFR) RPOR3.  The mapping of RP70 is controlled by cluster of bits named
                                        // RP70R (<5:0>), also in SFR RPOR3.
					// the special function register RPOR7 in the controller.
                RPOR3bits.RP71R = 0b000110;	// RD7 connected to SPI1's SCK.
                RPOR3bits.RP70R = 0b000101;	// RD6 connected to SPI1's SDO.
                SPI2CON1bits.MSTEN = 1;         // Set to master mode.  This bit needs to be set first
                                                // before setting SMP bit (see datasheet of dsPIC33EP256MU806).
                SPI1CON1 = 0x073A;              // SPI1 Master, 16 bits operation.
                                                // CKE=1, CKP=0, i.e. clock is active high, positive going edge.
                                                // SS pin is not used.
                                                // Set the SPI1 clock.  For this:
                                                // Primary pre-scalar = 4:1, secondary pre-scalar = 2:1.
                                                // Assuming CPU clock frequency of 60MHz, this will give
                                                // 60/(4x2) = 7.5MHz SPI clock.
                                                // For CPU clock frequency of 64MHz, this will give 
                                                // 64/(4x2) = 8.0MHz SPI clock, which is still within the limit
                                                // of 10 MHz.
                SPI1STAT = 0xA000;              // Enable SPI1 module.
                nTemp = SPI1BUF;                // Read the received buffer, to clear SPIRBF flag.
                PIN_12DAC_Driver_CS = 1;        // Deaasert &CS& pin.
                                                // Abort operation when device enters Idle mode.
                OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.						 
            break;
			
            case 1: // State 1 - Send command to DAC chip.  Effective rate is 1/_SYSTEMTICK_US Hz.
                    // Assuming _SYSTEMTICK_US = 100 usec, then DAC conversion rate is 10 kHz.
                unDAC1 = gobjDriverDAC.unDAC1_mV;
                if (unDAC1 > _12DAC_Vo_MAX) 		// Limit the maximum output voltage
                {                                   // for DAC channel B.
                    unDAC1 = _12DAC_Vo_MAX;
                }
                unDAC2 = gobjDriverDAC.unDAC2_mV;
                if (unDAC2 > _12DAC_Vo_MAX) 		// Limit the maximum output voltage.
                {                                   // for DAC channel A.
                    unDAC2 = _12DAC_Vo_MAX;
                }
                nDAC1 = 0xD000 | (0x0FFF & unDAC1);         // Get data and select DAC B.
                                                            // Internal gain = x2.
                //nDAC1 = 0xF000 | (0x0FFF & gunDACIn1_mV); // Get data and select DAC B.
                                                            // Internal gain = x1.
                nDAC2 = 0x5000 | (0x0FFF & unDAC2);         // Get data and select DAC A.
                                                            // Internal gain = x2.
                //nDAC2 = 0x7000 | (0x0FFF & gunDACIn2_mV); // Get data and select DAC A.
                                                            // Internal gain = x1.

                // Transmit command word to DAC-B.
                PIN_12DAC_Driver_CS = 0;			// Assert &CS& pin.
                SPI1BUF = nDAC1;                    // Shift out nDAC1. Note that data is shifted out via MSb.
                while (!SPI1STATbits.SPIRBF);       // Wait until 16 bits are shifted out, this is
                                                    // equivalent to 16 bits being shifted in, triggering
                                                    // the SPI receive buffer full flag, SPIRBF.
                PIN_12DAC_Driver_CS = 1;			// De-assert &CS& pin.
                nTemp = SPI1BUF;                    // Read the received buffer, to clear SPIRBF flag.
				
                // Transmit command word to DAC-A.
                PIN_12DAC_Driver_CS = 0;            // Assert &CS& pin.
                SPI1BUF = nDAC2;                    // Shift out nDAC2. Note that data is shifted out via MSb.
                while (!SPI1STATbits.SPIRBF);       // Wait until 16 bits are shifted out, this is
                                                    // equivalent to 16 bits being shifted in, triggering
                                                    // the SPI receive buffer full flag, SPIRBF.
                PIN_12DAC_Driver_CS = 1;            // De-aasert &CS& pin.
                nTemp = SPI1BUF;                    // Read the received buffer, to clear SPIRBF flag.
				
                PIN_12DAC_Driver_LDAC = 0;			// Pulse LDAC line (active low).

                if (gobjDriverDAC.unEnDAC == 0)         // Check gunRunDAC, whether to enable or disable
                                                        // the DAC module.
                {                                       // Disable DAC module.
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);    // Next state = 1, timer = 1.
                }
	
                PIN_12DAC_Driver_LDAC = 1;			// Note that the output of both DACs will reflect the new values
								// loaded simultaneously, i.e. synchronized.
            break;

            case 2: // State 2 - Check the status of gunRunDAC.
                SPI1STAT = 0x2000;                              // Disable SPI1 module.
                OSSetTaskContext(ptrTask, 0, 1);                // Next state = 0, timer = 1.
            break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1);    // Back to state = 0, timer = 1.
            break;
        }
    }
}