///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  APPLICATION PROGRAM INTERFACE ROUTINES FOR dsPIC33EXXXX MICROCONTROLLER
//
//  (c) Copyright 2016-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename         : os_dsPIC33E_APIsB.c
// Author           : Fabian Kung
// Last modified	: 10 March 2019
// Version          : 1.06
// Description		: This file contains the implementation of all the important processor specific
//                    routines used by the OS and the user processes. Most of the routines deal with
//                    microcontroller specifics resources, thus the functions have to be
//                    rewritten for different microcontroller family. Only the function
//                    prototype (call convention) of the routine is to be maintained.  In this
//                    way the OS can be ported to different microcontroller family.
//                    
//                    NOTE:
//                    Except for the interrupt service routine, the other functions in this
//                    file are essentially similar to OS_dsPIC33E_APIs.c
//
// Toolsuites		: Microchip MPLAB X IDE v5.10 or above
//                	  MPLAB XC-16 C-Compiler v1.33 or above
// Microcontroller	: dsPIC33E families.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "osmain.h"

// --- GLOBAL AND EXTERNAL VARIABLES DECLARATION ---

// --- FUNCTIONS' PROTOTYPES ---
void ClearWatchDog(void);
void dsPIC33E_PORInit(int);

// --- REGISTER FOR STORING PROCESSOR'S INTERRUPT CONTEXT ---
unsigned char bytIEC0bak;
unsigned char bytIEC1bak;
unsigned char bytIEC2bak;

// --- RAM BUFFER FOR STORING 1 PAGE OF PROGRAM MEMORY ---
int gnRamBuffer[DSPIC33E_PAGELENGTH*2];

// --- FUNCTIONS' BODY ---

//////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	//////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


// Configuration bits setting for the microcontroller in the header file "p33EP256MU806.h".

#pragma config FCKSM = CSDCMD   // Disable Fail-safe Clock Monitor and Clock-switching.
#pragma config IOL1WAY = OFF    // Allow multiple peripheral pin select configuration.
#pragma config OSCIOFNC = OFF   // OSC2 pin as oscillator output.
#pragma config POSCMD = XT     // Primary oscillator mode is XT (crylstal oscillator).
#pragma config FNOSC = PRIPLL   // Primary oscillator, XT mode with phase-locked loop.
							   // Note: the actual clock frequency is determined by setting the 
							   // feedback divider M, pre-divider N1 and post-divider N2 of the 
							   // PLL system in the Power-On Reset initialization routines.
							   // On power-up the processor will runs at the oscillator frequency,
							   // which is 4MHz in this case.

#pragma config FWDTEN = ON                          // Enable Watch-dog Timer.
//#pragma config FWDTEN = OFF                          // Disable Watch-dog Timer.
#pragma config BOREN = ON //& BOREN_ON              // Enable brown-out reset.
#pragma config FPWRT = PWR32                        // Set power-on reset timer to 32 msec.
//#pragma config GSS = ON                           // User program memory is code-protected.
                                                    // Note: whenever either GSS or GWRP is ON, the GSSK must be ON to
                                                    // ensure proper operation.
#pragma config GSS = ON                             // User program memory is code-protected but still writable.
#pragma config GWRP = OFF                           // Here we enable general section (GS) code protection,
#pragma config GSSK = ON                            // but still allows the user program to modify the general section,
                                                    // e.g. write protection OFF.

// Interrupt Service Routine 
// Author		: Fabian Kung
// Last modified	: 12 Dec 2011
// Purpose		: This routine performs the following important tasks pertaining to the 
//                RTOS:
//                1. Updates the 32-bits global clock tick gulClockTick of the OS when Timer 1 overflows.
//                2. Increments the timer property of each tasks.
//                3. Check for task overflow condition (i.e. the time used to run the tasks is
//                greater the the clock tick.  When this happen the main scheduler will be
//                stalled, with the indicator LED turned on all the time. This option can be
//                disabled by remarking the relevant codes in the routine.
//
// Arguments		: None
// Return		: None
void __attribute__((__interrupt__)) _T1Interrupt( void )
{
    if (IFS0bits.T1IF)                                                          // If it is Timer 1 overflow interrupt
    { 
		OSEnterCritical();
		if ((gnRunTask > 0) || (gunOSError > 0) || (gunTaskExecutedCount != 0)) //	If task overflow occurs or OS error detected,
                                                 // trap the controller 
		{                                        // indefinitely and turn on indicator LED1.
			while (1)
			{
				ClearWatchDog();				// Clear the Watch Dog Timer.
				PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
				PIN_ILED2 = 1;					// Turn on indicator LED2.
			}
		}
        
		gnRunTask = 1;							// Assert gnRunTask.
		gulClockTick++; 						// Increment RTOS clock tick counter. 	
        
        // The codes below should be implemented in the Main() function for this version.
        // This will produce more efficient codes.
		//for (ni = 0;ni < gnTaskCount;ni++)		// Using for-loop produce more efficient 
		//										// assembly codes.
		//{	
		//	if (gstrcTaskContext[ni].nTimer > 0) // Only decrement timer if it is greater than zero.
		//	{
		//		--(gstrcTaskContext[ni].nTimer); // Decrement timer for each process.
		//	}
		//}
		
		PR1 = __TIMER1COUNT;					// Load Period Register.
 		IFS0bits.T1IF = 0;					// reset Timer 1 interrupt flag.
		OSExitCritical(); 	
    }
}     

// Interrupt Service Routine 
// Author           : Fabian Kung
// Last modified	: 9 March 2019
// Purpose          : These routine handles various hardware traps or exception.s 
//                    Basically we stalled the processor when math error occurs.
//                    To indicate to the user that math exception occurs, we 
//                    blink indicator LED1 rapidly.
// Arguments		: None
// Return           : None
void __attribute__((__interrupt__)) _MathError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _AddressError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _HardTrapError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _StackError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _SoftTrapError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _DMACError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

// Function name	: ClearWatchDog 
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Purpose          : Reset the Watch Dog Timer
// Arguments		: None
// Return           : None
void ClearWatchDog(void)
{
	asm ("clrwdt");	// Inline assembly instruction to clear the Watch Dog Timer.
}

// Function Name	: dsPIC33E_PORInit
// Author           : Fabian Kung
// Last modified	: 1 Dec 2018
// Description		: Perform power-on reset (POR) initialization on the microcontroller.
//                    Upon completion of this routine, all the microcontroller peripherals and
//                    I/O ports will be set to known state. For I/O ports, all pins will
//                    be set to
//                    (a) Digital mode,
//                    (b) Output and
//                    (c) A logic '0'.
// Arguments		: nProcessorClockMHz - Processor clock frequency, valid values are 120, 130 or 140 MHz.
//                    If not valid input is provided, the processor clock defaults to 120 MHz.
// Return           : None
// 
void dsPIC33E_PORInit(int nProcessorClockMHz)
{

	// Sets the internal phase-locked loop (PLL) for clock generation.
	// Oscillator frequency = Fin = 4MHz.
	// Fclk = Fin (M/(N1XN2).  
	// M = PLL feedback divider = PLLDIV + 2
	// N1 = Pre-divider into PLL = PLLPRE + 2
	// N2 = Post-divider from voltage-controlled oscillator (VCO) of PLL = 2(PLLPOST + 1)
	// If we are setting Fclk to 120MHz for 60MIPS operation (2 clock cycle per instruction).
    // The following selections are used:
    // Option 1: M = 120, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 120 MHz.
    // or PLLDIV = 118 = b'01110110', PLLPRE = 0, PLLPOST = 0.
	// Option 2,3 and 4: For slightly higher clock speed (with slightly higher power dissipation)
    // M = 124, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 124 MHz.
    // or PLLDIV = 122, PLLPRE = 0, PLLPOST = 0.
    // M = 128, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 128 MHz.
    // or PLLDIV = 126, PLLPRE = 0, PLLPOST = 0.
    // M = 132, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 132 MHz.
    // or PLLDIV = 130, PLLPRE = 0, PLLPOST = 0.
    
    if (nProcessorClockMHz == 140)
    {
        PLLFBD = 138; // Clock option 3, sets PLLDIV = 140, Option 3, Fclk = 140 MHz
    }
    else if (nProcessorClockMHz == 130)
    {
        PLLFBD = 128; // Clock option 2, sets PLLDIV = 130, Option 2, Fclk = 130 MHz
    }
    else
    {
        PLLFBD = 118;
        //PLLFBD = 0x0076; // Clock option 1, sets PLLDIV = 120. Option 1, Fclk = 120 MHz.
    }    
    
    CLKDIV = 0x0000; // Sets PLLPRE = 0 and PLLPOST = 0.
                     // Peripheral clock = processor clock.
                     // Internal fast RC oscillator post scaler = 1.
                     // Fcy divided by 1, e.g. processor clock = fosc/2.

	// Check for Watchdog Timer timeout.
//	if (RCONbits.WDTO == 1) // Check the flag WDTO in Reset Control Register (RCON).
//	{
//		while (1) 
//		{
//			ClearWatchDog(); // Clear Watchdog Timer.
//			LATBbits.PLED1 = 1; // Turn on indicator LED1.
//		}
//	}

	// I/O port setting:
	// Port B setting, all Port B pins are set to outputs by default to prevent floating inputs.
	LATB = 0x0000;
	TRISB = 0x0000;
    ANSELB = 0x0000;        // Set all pins of Port B to digital by default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.
                            // Port B pins are mapped to ADC inputs.

    TRISBbits.TRISB0 = 1;   // Set RB0 to input.  This is the input from voltage reference IC.

	// Port C setting, all Port C pins are set to outputs by default to prevent floating inputs.
	LATC = 0x0000;	
	TRISC = 0x0000;

    // Port D setting, all Port D pins are set to outputs by default to prevent floating inputs.
	LATD = 0x0000;
	TRISD = 0x0000;

    // Port E setting, all Port E pins are set to digital mode, and outputs by
    // default to prevent floating inputs.
	LATE = 0x0000;
	TRISE = 0x0000;
    ANSELE = 0x0000;        // Set all pins to digital be default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.

    // Port F setting, all Port F pins are set to outputs by default to prevent floating inputs.
	LATF = 0x0000;
	TRISF = 0x0000;

    // Port G setting, all Port g pins are set to outputs by default to prevent floating inputs.
	LATG = 0x0000;
	TRISG = 0x0000;
    ANSELG = 0x0000;        // Set all pins of Port G to digital by default.  Port G pins are
                                // mapped to Analog Comparator inputs.
	// Setup 16-bit Timer1:
	// Note - Timer1 will be driven by internal clock cycle.  It will increase up 
	// to the value set in PR1, then reset back to 0. An interrupt will be triggered
	// during this event, known as Timer1 overflow.  
	// Set Timer1 to default state first.
	T1CON = 0;
	// Prescaler = 1:1, valid value 0-3 (2 bits).
	T1CONbits.TCKPS = 0;    // Clock source from internal oscillator (peripheral clock).
	// Load Period Register.
	PR1 = __TIMER1COUNT;	
	// Reset Timer1 register.		
	TMR1 = 0; 
	// reset Timer 1 interrupt flag.
 	IFS0bits.T1IF = 0;
 	// set Timer1 interrupt priority level to 1 (highest priority).
	IPC0bits.T1IP = 1;
	// enable Timer 1 interrupt.
 	IEC0bits.T1IE = 1;
	// Turn on Timer1 and start counting.
	T1CONbits.TON = 1; // Turn on Timer1.
}

// Function name	: OSEnterCritical
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Description		: Disable all processor interrupts for important tasks
//                    involving Stacks, Program Counter other critical
//	                  processor registers.
void OSEnterCritical(void)
{
	bytIEC0bak = IEC0;	// Store current processor interrupt settings.
	bytIEC1bak = IEC1;
	bytIEC2bak = IEC2;
	IEC0 = 0x00;		// Disable all processor interrupts.
	IEC1 = 0x00;
	IEC2 = 0x00;
}											

// Function name	: OSExitCritical
// Author           : Fabian Kung
// Last modified	: 24 April 2007
// Description		: Enable all processor interrupts for important tasks.
void OSExitCritical(void)
{
    IEC0 = bytIEC0bak;	// Restore processor interrupt settings.
    IEC1 = bytIEC1bak;
    IEC2 = bytIEC2bak;
}											


// Function name	: OSProce1
// Author           : Fabian Kung
// Last modified	: 4 April 2016
// Description		: Blink an indicator LED1 to show that the microcontroller is 'alive'.
#define _LED1_ON_MS	500		// LED1 on period in msec, i.e. 500 msec.

void OSProce1(TASK_ATTRIBUTE *ptrTask)
{
    switch (ptrTask->nState)
    {
	case 0: // State 0 - On Indicator LED1
            PIN_OSPROCE1 = 1;                                           // Turn on indicator LED1.
            OSSetTaskContext(ptrTask, 1, _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = _LED_ON_MS.
	break;

	case 1: // State 1 - Off Indicator LED1
            PIN_OSPROCE1 = 0;                                           // Turn off indicator LED1.
            OSSetTaskContext(ptrTask, 0, _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC);    // Next state = 0, timer = _LED_ON_MS
            break;

        default:
            OSSetTaskContext(ptrTask, 0, 0);                                    // Back to state = 0, timer = 0.
    }
}

// Read a page of data in program memory into RAM buffer using table read
// instruction.
// Author       : Fabian Kung
// Last Modified: 28 Nov 2018
// Argument     : unTablePage - Page number (for TBLPAG register) of the address
//                to the first word in program memory to read.
//                unTableOffset - Offset address to the first word in program
//                memory to read.
// Return       : None
// Example of usage:
// Assuming progData[] contains an array of integer data store in the program
// memory, we thus need to provide the page and offset address of the first
// element of progData[0] to this function.  To do this we use the built-in
// function that comes with the C-compiler.  We also assume the address of progData[0]
// is already aligned to the page boundary.  If this is not the case, the nearest
// address smaller than &progData[0] will be used.
//
// ProgMemReadPage(__builtin_tblpage(progData),__builtin_tbloffset(progData));
//
void ProgMemReadPage(unsigned int unTablePage, unsigned int unTableOffset)
{
    int i, nTblpag_bak;
   
    nTblpag_bak = TBLPAG;       // The datasheet recommends backing up TBLPAG before 
                                // modifying it.
    TBLPAG = unTablePage;
    unTableOffset = unTableOffset & 0xF800;   
                                // Set to base of page.
                                // For page size of 1024 24-bits words 
                                // or 2x1024 16-bits words in the program memory.
                                // 2x1024 - 1 = 2047 or 0x7FF.  The the lower
                                // 11-bits of the address is always 0.  If the
                                // lower 11-bits of offset is not 0, it means the
                                // address of progData starts after the page
                                // boundary, so we adjust it to start at the 
                                // page boundary. 
    for (i = 0; i<(DSPIC33E_PAGELENGTH*2); i++)
    {
        gnRamBuffer[i++] = __builtin_tblrdl(unTableOffset);
        gnRamBuffer[i] = __builtin_tblrdh (unTableOffset);
        unTableOffset += 2;
    }
    TBLPAG = nTblpag_bak;       // Restore TBLPAG.
}

// Function to erase a page of data in program memory.
// Author       : Fabian Kung
// Last Modified: 28 Nov 2018
// Arguments    : unTablePage - Page number (for NVMADRU register) of the address
//                to the first word in program memory to read.
//                unTableOffset - Offset address to the first word in program
//                memory to read.
// Example of usage:
// Assuming progData[] contains an array of integer data store in the program
// memory, we thus need to provide the page and offset address of the first
// element of progData[0] to this function.  To do this we use the built-in
// function that comes with the C-compiler.  We also assume the address of progData[0]
// is already aligned to the page boundary.  If this is not the case, the nearest
// address smaller than &progData[0] will be used.
//
// ProgMemErasePage(__builtin_tblpage(progData),__builtin_tbloffset(progData));
//
void ProgMemErasePage(unsigned int unTablePage, unsigned int unTableOffset)
{
    NVMADRU = unTablePage;
    NVMADR = (unTableOffset & 0xF800); // For page size of 1024 24-bits words 
                                // or 2x1024 16-bits words in the program memory.
                                // 2x1024 - 1 = 2047 or 0x7FF.  Thus the the lower
                                // 11-bits of the starting address is always 0.  
                                // If the lower 11-bits of offset is not 0, it 
                                // means the address of progData starts after the
                                // page boundary, so we adjust it to start at the 
                                // page boundary. 
    NVMCON = 0x4003;            // Set WREN and Page Erase in NVMCON.
    __builtin_disi(6);          // Disable interrupts.
    __builtin_write_NVM();      // Initiate write process.
}

// Function to write a row of flash data from the RAM buffer into the program memory.
// Author       : Fabian Kung
// Last Modified: 28 Nov 2018
// Argument:    unTablePage - Page number (for NVMADRU register) of the address
//              to the first word in program memory to read.
//              unTableOffset - Offset address to the first word in program
//              memory to read.
//              nRowOffset, from 0, 1, 2, ... 7 for page length with 8 rows.
// Return:      None
// Example of usage:
// Assuming progData[] contains an array of integer data store in the program
// memory, and we want to update the content in progData[].  We thus need to 
// provide the page and offset address of the first element of progData[0] to 
// this function.  We also need to furnish the row offset, which is integer 
// from 0,1,2, to (DSPIC33E_PAGELENGTH/DSPIC33E_ROWLENGTH)-1
// To extract the page and offset address for progData[0], we use the built-in 
// function that comes with the C-compiler.  
//
// The statement below programs the first 128 words (0-127) starting at the address 
// of progData[0] (Assume &progData[0] is already page aligned).
// ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),0);
//
// The statement below programs the words 128-255 from the starting address 
// of progData[0] (Assume &progData[0] is already page aligned).
// ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),0);
void ProgMemRowWrite(unsigned int unTablePage, unsigned int unTableOffset, int nRowOffset)
{
    int nOffset, nTblpag_bak, i;
    
    if (nRowOffset < 8)             // 8 rows per page for dsPIC33E family.
    {
        nTblpag_bak = TBLPAG;       // The datasheet recommends backing up TBLPAG before 
                                    // modifying it.    
        TBLPAG = 0xFA;              // Base address of write latches.
        // Load row of data into write latches.
        nOffset = nRowOffset*DSPIC33E_ROWLENGTH*2;
        for (i = 0; i< DSPIC33E_ROWLENGTH*2; i++)
        {
            __builtin_tblwtl(nOffset, gnRamBuffer[i++]);
            __builtin_tblwth(nOffset, gnRamBuffer[i]);
            nOffset += 2;
        }
    
        // Set the destination address of the program memory into the
        // NVM address registers.
        NVMADRU = unTablePage;
        NVMADR = (unTableOffset & 0xF800) + (nRowOffset*2*DSPIC33E_ROWLENGTH); // For page size of 1024 24-bits PM words.
                                                            
        // Set WREN and enable row write in NVMCON.
        NVMCON = 0x4002;
    
        __builtin_disi(6);          // Disable interrupts.
        __builtin_write_NVM();      // Initiate write process.
        TBLPAG = nTblpag_bak;       // Restore TBLPAG.
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	   ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
