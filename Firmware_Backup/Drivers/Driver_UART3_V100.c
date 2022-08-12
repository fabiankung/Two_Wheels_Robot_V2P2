//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File			: Drivers_UART3_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 26 Feb 2016
// Toolsuites		: Microchip MPLAB X IDE v3.10 or above
//                	  MPLAB XC16 C-Compiler v1.25 or above

#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

#define     __SCI_TXBUF_LENGTH3     15
#define     __SCI_RXBUF_LENGTH3     15
//
// --- PUBLIC VARIABLES ---
//
// Data buffer and address pointers for wired serial communications (UART).
BYTE gbytTXbuffer3[__SCI_TXBUF_LENGTH3-1];          // Transmit buffer.
BYTE gbytTXbufptr3;                                 // Transmit buffer pointer.
BYTE gbytTXbuflen3;                                 // Transmit buffer length.
BYTE gbytRXbuffer3[__SCI_RXBUF_LENGTH3-1];          // Receive buffer length.
BYTE gbytRXbufptr3;                                 // Receive buffer length pointer.

SCI_STATUS gSCIstatus3;

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//
//#define	_UART3_BAUDRATE_kBPS	19.2	// Default datarate in kilobits-per-second, for G15 servo motor.
#define	_UART3_BAUDRATE_kBPS	9.6	// Default datarate in kilobits-per-second, for HC-05 module.
//#define	_UART3_BAUDRATE_kBPS	57.6	// Default datarate in kilobits-per-second, for dynamixel servo motor.

///
/// Process name	: Proce_UART3_Driver
///
/// Author		: Fabian Kung
///
/// Last modified	: 26 Feb 2016
///
/// Code version	: 1.22
///
/// Processor		: dsPIC33EP256GMU8XX family.
///                       
///
/// Processor/System Resource 
/// PINS		: 1. Pin RB3 = U3RX (Remappable I/O RPI35), input.
///  			  2. Pin RF3 = U3TX (Remappable I/O RP99), output.
///                       3. PIN_ILED2 = indicator LED2.
///
/// MODULES		: 1. UART3 (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytRXbuffer3[]
///                       gbytRXbufptr3
///                       gbytTXbuffer3[]
///                       gbytTXbufptr3
///                       gbytTXbuflen3
///                       gSCIstatus3
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_UART3_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_UART3_Driver: An RTOS is required with this function"
#endif

///
/// Description		: 1. Driver for built-in UART3 Module.
///                       2. Serial Communication Interface (UART) transmitt buffer manager.
///                          Data will be taken from the SCI transmitt buffer gbytTXbuffer in FIFO basis
///                          and transmitted via USART module.  Maximum data length is determined by the
///			     constant _SCI_TXBUF_LENGTH in file "osmain.h".
///                       3. Serial Communication Interface (UART) receive buffer manager.
///			     Data received from the USART module of the microcontroller will be
///			     transferred from the USART registers to the RAM of the microcontroller
///			     called SCI receive buffer (gbytRXbuffer2[]).
///			     The flag bRXRDY will be set to indicate to the user modules that valid
///			     data is present.
///			     Maximum data length is determined by the constant _SCI_RXBUF_LENGTH in
///			     file "osmain.h".
///
///
/// Example of usage : The codes example below illustrates how to send 2 bytes of character,
///			'a' and 'b' via UART3.
///                    if (gSCIstatus3.bTXRDY == 0)	// Check if any data to send via UART.
///                    {
///                         gbytTXbuffer3[0] = 'a';	// Load data.
///		   	    gbytTXbuffer3[1] = 'b';
///		   	    gbytTXbuflen3 = 2;		// Set TX frame length.
///		  	    gSCIstatus3.bTXRDY = 1;	// Initiate TX.
///                    }
///
/// Example of usage : The codes example below illustrates how to retrieve 1 byte of data from
///                    the UART receive buffer.
///                    if (gSCIstatus3.bRXRDY == 1)	// Check if UART receive any data.
///		       {
///                         if (gSCIstatus3.bRXOVF == 0) // Make sure no overflow error.
///			    {
///                             bytData = gbytRXbuffer3[0];	// Get 1 byte and ignore all others.
///                         }
///                         else
///                         {
///				gSCIstatus3.bRXOVF = 0; 	// Reset overflow error flag.
///                         }
///                         gSCIstatus3.bRXRDY = 0;	// Reset valid data flag.
///                         gbytRXbufptr3 = 0; 		// Reset pointer.
///			}
///
/// Note: Another way to check for received data is to monitor the received buffer pointer
/// gbytRXbufptr.  If no data this pointer is 0, a value greater than zero indicates the
/// number of bytes contain in the receive buffer.


void Proce_UART3_Driver(TASK_ATTRIBUTE *ptrTask)
{

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - UART3 Initialization.
				// Setup IO pins mode and configure the remappable peripheral pins:

				TRISFbits.TRISF3 = 0;			// Set RF3 (RP99) to output.
				RPOR8bits.RP99R = 0b011011;		// RP99 (RF3) tied to U3TX.
									// Note: Once this is done, the controller will 
									// override the I/O mode in the original TRISx 
									// register.

                                TRISBbits.TRISB3 = 1;                   // Set RB3 (RPI35) to input.
                                RPINR27bits.U3RXR = 0b0100011;          // RPI35 tied to U3RX.

				// Setup baud rate generator register, this requires BRGH=1:
				// NOTE: This formula is specified in the manner shown to prevent overflow of the 
				// Integer (16-bits) during the arithmetic operation.
			
				U3BRG = (__FOSC_MHz/8)*(1000/_UART3_BAUDRATE_kBPS)-1;
                                
				// Setup UART3 operation mode part 1:
				// 1. Enable UART3 module.
				// 2. Continue module operation in idle, no wake-up enabled.
				// 3. Loopback mode disabled.
				// 4. No auto-baud rate detect.	
				// 5. Only U3TX and U3RX pins are used, no flow control.
				// 6. 8 bits data, even parity, 1 stop bit.
				// 7. Sync break transmission disabled.
				// 8. Interrup when any character is received (the U1RSR can take in 4 characters).
				// 9. Address detect is disabled.
				U3MODEbits.PDSEL = 0b01;        // 8 bits, even parity.
				U3MODEbits.BRGH = 1;            // High-speed clock generation mode (better).
				U3MODEbits.UARTEN = 1;          // Enable UART3.
				U3MODEbits.UEN = 0b00;          // Only U3TX and U3RX pins are used. U2CTS, U2RTS and BCLK
                                                                // pins are controlled by latches.
                                U3MODEbits.PDSEL = 0b00;        // 8 bits data, no parity.
				U3MODEbits.STSEL = 0;           // 1 stop bit.
                                U3MODEbits.UARTEN = 1;          // Enable UART3 module.
				U3STAbits.UTXEN = 1;            // Enable Transmitt module in UART3.
                                //U3STAbits.URXEN = 1;          // NOTE: This bit is not available in all dsPIC33E devices.
				gbytTXbuflen3 = 0;               // Initialize all relevant variables and flags.
				gbytTXbufptr3 = 0;
				gSCIstatus3.bRXRDY = 0;
				gSCIstatus3.bTXRDY = 0;
				gSCIstatus3.bRXOVF = 0;
                                //U3STAbits.OERR = 0;
				gbytRXbufptr3 = 0;
                                PIN_ILED2 = 0;                      // Off indicator LED2.
				OSSetTaskContext(ptrTask, 10, 100);  // Next state = 10, timer = 100.
			break;
			
			case 1: // State 1 - Transmitt and receive buffer manager.
				// Check for data to send via UART3.
				// Note that the transmitt FIFO buffer is 4-level deep in dsPIC33 and PIC24H microcontrollers.
				if (gSCIstatus3.bTXRDY == 1)                     // Check if valid data in SCI buffer.
				{
                                    while (U3STAbits.UTXBF == 0)                // Check if UART transmitt buffer is not full.
										// 2 conditions to exit the load transmit buffer
                                                                                // routines, (1) When transmit butter is full or
                                                                                // (2) when transmit buffer pointer equals
                                                                                // transmit data length.
                                    {
                                        PIN_ILED2 = 1;                          // On indicator LED2.
					if (gbytTXbufptr3 < gbytTXbuflen3)
					{
                                            U3TXREG = gbytTXbuffer3[gbytTXbufptr3];  // Load 1 byte data to UART transmitt buffer.
                                            gbytTXbufptr3++;			   // Pointer to next byte in TX buffer.
                                        }
                                        else
                                        {
 					    gbytTXbufptr3 = 0;                   // Reset TX buffer pointer.
					    gbytTXbuflen3 = 0;                   // Reset TX buffer length.
					    gSCIstatus3.bTXRDY = 0;              // Reset transmitt flag.
                                            PIN_ILED2 = 0;                      // Off indicator LED2.
                                            break;
                                        }
                                    }
				}

				// Check for data to receive via UART.
				// Note that the receive FIFO buffer is 4-level deep in dsPIC33 and PIC24H microcontrollers.
                                // Here we ignore Framing and Parity error.  If overflow error is detected, the dsPIC33
                                // UART receive circuitry will hangs, we need to reset it by clearing the OERR bits in
                                // UxSTA register.
                                
                                if (U3STAbits.OERR == 0)                        // Make sure no overflow error.
                                {
                                    while (U3STAbits.URXDA == 1)                // If at least 1 data byte is
										// available at UART
                                    {
                                        PIN_ILED2 = 1;                          // On indicator LED2.
                                        if (gbytRXbufptr3 < (__SCI_RXBUF_LENGTH3-1))	// check for data overflow.
                                        {					// Read a character from USART.
                                            gbytRXbuffer3[gbytRXbufptr3] = U3RXREG; // Get received data byte.
                                            gbytRXbufptr3++;                     // Pointer to next byte in RX buffer.
                                            gSCIstatus3.bRXRDY = 1;		// Set valid data flag.
                                        }
                                        else 					// data overflow.
                                        {
                                            gbytRXbufptr3 = 0;			// Reset buffer pointer.
                                            gSCIstatus3.bRXOVF = 1;		// Set receive data overflow flag.
                                        }
                                        //PIN_ILED2 = 0;
                                    }
                                }
                                else                                            // Overflow/overrun error.
                                {
                                    U3STAbits.OERR = 0;                         // Clear overrun flag, this will also
                                                                                // clear and reset the receive FIFO in
                                                                                // the UART module.
                                    gbytRXbufptr3 = 0;                           // Reset buffer pointer.
                                    gSCIstatus3.bRXOVF = 1;                      // Set receive data overflow flag.
                                } 
				OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.
			break;

                    case 10: // State 10 - Keep on reading the receive buffer to clear the receive status flag.
                             // Note: 1 May 2015, I noticed that upon power up, the UART module may contains invalid
                             // data, and the URXDA flag will be set to 1.  Thus we should clear this intially before
                             // proceeding.
                        if (U3STAbits.URXDA == 1)
                        {
                            gbytRXbuffer3[gbytRXbufptr3] = U3RXREG;               // Read the receive buffer.
                            OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                        }
                        break;

			default:
				OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
			break;
		}
	}
}


