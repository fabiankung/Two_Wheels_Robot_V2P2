// Author			: Fabian Kung
// Date				: 24 Aug 2020
// Filename			: osmain.h

///////////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO dsPIC33E MICROCONTROLLER        ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __OSMAIN_H
#define __OSMAIN_H

// --- Microcontroller specific header file (provided by compiler manufacturer) ---

#include <xc.h>     // This header file will allow access to all include files.
                    // Recommended by Microchip, and also to conform to the practice
                    // of code portability, via the concept of Common C Interface (CCI).
                    // The header file for the microcontroller will be automatically
                    // included with this header file.

#define DSPIC33E_PAGELENGTH 1024    // Page length for dsPIC33E family,
                                    // 1024 24-bits instruction words in one page.
#define DSPIC33E_ROWLENGTH 128      // Row length for dsPIC33E family,
                                    // 128 24-bits instruction words in one row.

// --- Mapping of variable datatype to match the controller's architecture ---
#define	INT16 	int                 // 16-bits signed integer.
#define INT32   long                // 32-bits signed integer.
#define	UINT16	unsigned int		// 16-bits unsigned integer.
#define UINT32  unsigned long		// 32-bits unsigned integer.
#define	BYTE	unsigned char		// 8-bits unsigned integer.

// --- Microcontroller I/O Pin definitions ---
#define	PIN_OSPROCE1            _RD10 		// Indicator LED1 driver pin.
#define	PIN_ILED2               _RD11 		// Indicator LED2 driver pin.
#define PIN_PSW                 _RB15       // Analog power supply switch control.

// --- Processor Clock and Kernel Cycle in microseconds ---
// Note: Uncomment the required value for _TIMER1COUNT, and update the corresponding definition
// for the constant _SYSTEMTICK_US, in microseconds.

/*
// - For 120 MHz oscillator - 
#define	__FOSC_MHz              120             // Oscillator clock frequency in MHz.
#define __FCYC_MHz              __FOSC_MHz*0.5  // Processor frequency = 60 MHz.
#define	__TCLK_US               0.01667         // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/60000000 = 1.667E-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec
*/

/*
// - For 124 MHz oscillator -
#define	__FOSC_MHz              124             // Oscillator clock frequency in MHz.
#define __FCYC_MHz              __FOSC_MHz*0.5  // Processor frequency = 62 MHz.
#define	__TCLK_US               0.016129        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/62000000 = 1.61290-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec
*/

/*
// - For 128 MHz oscillator -
#define	__FOSC_MHz              128             // Oscillator clock frequency in MHz.
#define __FCYC_MHz              __FOSC_MHz/2    // Processor frequency = 64 MHz.
#define	__TCLK_US               0.015625        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/64000000 = 1.5625-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec
*/

// - For 132 MHz oscillator -
//#define	__FOSC_MHz              132             // Oscillator clock frequency in MHz.
//#define __FCYC_MHz              __FOSC_MHz/2    // Processor frequency = 66 MHz.
//#define	__TCLK_US               0.015152        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/66000000 = 1.5152-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec

// - For 136 MHz oscillator -
//#define	__FOSC_MHz              136             // Oscillator clock frequency in MHz.
//#define __FCYC_MHz              __FOSC_MHz/2    // Processor frequency = 68 MHz.
//#define	__TCLK_US               0.014706        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/68000000 = 1.4706-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec

// - For 140 MHz oscillator -
#define	__FOSC_MHz              140             // Oscillator clock frequency in MHz.
#define __FCYC_MHz              __FOSC_MHz/2    // Processor frequency = 70 MHz.
#define	__TCLK_US               0.014286        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/70000000 = 1.4286-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec
                                                
                                                
#if     __FOSC_MHz  == 120                       // No. of Tcyc for Timer1 to expire (for approximately 166.67us cycle).
    #define __TIMER1COUNT           10000
#elif   __FOSC_MHz  == 124
    #define __TIMER1COUNT           10333
#elif   __FOSC_MHz  == 128
    #define __TIMER1COUNT           10666
#elif   __FOSC_MHz  == 132
    #define __TIMER1COUNT           11000
#elif   __FOSC_MHz  == 136
    #define __TIMER1COUNT           11333
#elif   __FOSC_MHz  == 140
    #define __TIMER1COUNT           11666
#else
    #error  "osmain.h: Wrong frequency setting for oscillator"
#endif


#define	__SYSTEMTICK_US         166.67          // System_Tick = (_TIMER1COUNT*(Tclk))

#define     __NUM_SYSTEMTICK_MSEC         6     // Requires 6 system ticks to hit 1 msec period.

///////////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO PIC24H/dsPIC33 MICROCONTROLLER  //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


// --- RTOS CONSTANTS ---
#define	__OS_VER		2           // RTOS/Scheduler version, need to be integer (ANSI C preprocessor
                                    // expression requires integer). 

#define	__MAXTASK               20	// Maximum no. of concurrent tasks supported. Ideally should be 16.
#define __SCI_TXBUF_LENGTH      100	// SCI transmitt  buffer length in bytes.
#define __SCI_RXBUF_LENGTH      12	// SCI receive  buffer length in bytes.
#define __RFSCI_TXBUF_LENGTH	32	// RF SCI transmitt  buffer length in bytes.
#define __RFSCI_RXBUF_LENGTH	32	// RF SCI receive  buffer length in bytes.

// --- RTOS DATATYPES DECLARATIONS ---
// Type cast for a structure defining the attributes of a task,
// e.g. the task's ID, current state, counter, variables etc.
typedef struct StructTASK
{	
	INT16 nID;      // The task identification.  Also determines the sequence
                    // in which the task is executed by the Kernel.  Task with 
                    // ID = 1 will be executed first. Valid value is 1-255.
                    // ID = 0 is used to indicate empty task.  
	INT16 nState;	// The current state of the task.  Useful for implementing 
                    // an algorithmic state machine.
	INT16 nTimer;	// This variable will be decremented on every clock tick. 
                    // The Scheduler uses this variable to determine whether to execute a 
                    // task or not.  If nTimer = 0, the corresponding task will be
                    // executed, else the task will be skipped.
                    // Useful for implementing a non-critical delay within a task.
} TASK_ATTRIBUTE;

// Type cast for a pointer to a task, TASK_POINTER with argument of a pointer to TASK_ATTRIBUTE.
typedef void (*TASK_POINTER)(TASK_ATTRIBUTE *);

// Type cast for a Bit-field structure - Serial Communication Interface (SCI) status
typedef struct StructSCI
{
	unsigned bTXRDY: 	1;	// Set to indicate valid data for the wired SCI module to transmit.
	unsigned bRXRDY: 	1;	// Set if there is valid byte data in the wired SCI receive buffer.
	unsigned bRXOVF:	1;	// Set if there is data overflow in wired SCI receive buffer, i.e. old
                            // data has not been read but new data has arrived.
	unsigned bRFTXRDY:	1;	// Set to indicate valid data for the RF transceiver module to transmit.
	unsigned bRFRXRDY:	1;	// Set if there is valid byte data in the RF transceiver module receive 
                            // buffer.
	unsigned bRFRESET:	1;	// Set to reset RF transceiver.
	unsigned bRFTXERR:	1;	// Set to indicate transmission is not successful.
} SCI_STATUS;

// Type cast for Bit-field structure - I2C interface status.
typedef struct StructI2CStatus
{
    unsigned bI2CBusy:      1;      // Mutex, set when I2C module is being used.
	unsigned bCommError:    1;      // Set to indicate communication error in the I2C bus.
	unsigned bRead:         1;      // Set to initiate reading of data (Slave -> Master).
    unsigned bSend:         1;      // Set to initiate sending of data (Master -> Slave).
} I2C_STATUS;

// --- RTOS FUNCTIONS' PROTOTYPES ---
// Note: The body of the followings routines is in the file "os_APIs.c"
void OSInit(void);
BYTE OSCreateTask(TASK_ATTRIBUTE *, TASK_POINTER );
void OSSetTaskContext(TASK_ATTRIBUTE *, INT16, INT16);
BYTE OSTaskDelete(INT16);
void OSUpdateTaskTimer(void);
void OSEnterCritical(void);
void OSExitCritical(void);
void OSProce1(TASK_ATTRIBUTE *ptrTask); 	// Blink indicator LED1 process.
// Note: The body of the followings routines is in the file "os dsPIC33E_APIs.c"
void ClearWatchDog(void);
void dsPIC33E_PORInit(int);

// --- ROUTINES TO ACCESS AND REPROGRAM PROGRAM MEMORY ---
void ProgMemReadPage(unsigned int, unsigned int);
void ProgMemErasePage(unsigned int, unsigned int);   
void ProgMemRowWrite(unsigned int, unsigned int, int);

// --- GLOBAL/EXTERNAL VARIABLES DECLARATION ---
// STANDARD GLOBAL VARIABLES
// Note: The followings are defined in the file "os_dsPIC33E_APIs.c"
extern BYTE bytIEC0bak;
extern BYTE bytIEC1bak;
extern BYTE bytIEC2bak;
extern int gnRamBuffer[DSPIC33E_PAGELENGTH*2];

// Note: The followings are defined in the file "os_APIs.c"
extern volatile INT16 gnRunTask;
extern INT16 gnTaskCount;
extern volatile UINT32 gulClockTick;
extern TASK_ATTRIBUTE gstrcTaskContext[__MAXTASK-1];
extern TASK_POINTER gfptrTask[__MAXTASK-1];
extern SCI_STATUS gSCIstatus;
extern UINT16 gunTaskExecutedCount;
extern UINT16 gunOSError;
#endif
