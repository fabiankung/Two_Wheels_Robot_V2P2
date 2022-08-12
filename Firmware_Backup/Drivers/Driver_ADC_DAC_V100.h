// Author			: Fabian Kung
// Date				: 7 Dec 2017
// Filename			: Driver_ADC_DAC_V100.h

#ifndef _DRIVER_ADC_DAC_dsPIC33_H
#define _DRIVER_ADC_DAC_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
// ADC Module:
typedef struct StructProceADCDriver
{
    UINT16   unADC1_mV;         // ADC Channel 1 output.
    UINT16   unADC2_mV;         // ADC Channel 1 output.
    UINT16   unADC3_mV;         // ADC Channel 1 output.
    UINT16   unSampleCount;     // Counter to keep track of ADC sample.
    UINT16   unEnADC;           // Set greater than 0 to enable ADC module.
    UINT16   unSamplingInterval_us;   // Sampling interval of ADC in micro-seconds.
} DSPIC33E_ADC_DRIVER;

extern  DSPIC33E_ADC_DRIVER    gobjDriverADC;
			
// DAC Module
typedef struct StructProceDACDriver
{
    UINT16   unDAC1_mV;         // ADC Channel 1 output.
    UINT16   unDAC2_mV;         // ADC Channel 1 output.
    UINT16   unEnDAC;           // Set greater than 0 to enable the DAC module.
    UINT16   unSamplingInterval_us;   // Sampling interval of DAC in micro-seconds.
} DSPIC33E_DAC_DRIVER;

extern DSPIC33E_DAC_DRIVER     gobjDriverDAC;
					
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_ADC_Driver(TASK_ATTRIBUTE *);		// ADC module driver.
													// Here the ADC module is built-in,
													
void Proce_12DAC_MCP4822_Driver(TASK_ATTRIBUTE *);	// DAC module driver.
													// Here the DAC module is external,
													// using Microchips MCP4822 and driven
													// via SPI bus.	
#endif