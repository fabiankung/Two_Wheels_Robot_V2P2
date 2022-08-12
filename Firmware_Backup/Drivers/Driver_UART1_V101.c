//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016=2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_UART1_V101.c
// Author(s)		: Fabian Kung
// Last modified	: 9 March 2019
// Tool suites		: Microchip MPLAB X IDE v5.10 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above

#include "osmain.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//
// Data buffer and address pointers for wired serial communications (UART).
BYTE gbytTXbuffer[__SCI_TXBUF_LENGTH-1];       // Transmit buffer.
BYTE gbytTXbufptr;                             // Transmit buffer pointer.
BYTE gbytTXbuflen;                             // Transmit buffer length.
BYTE gbytRXbuffer[__SCI_RXBUF_LENGTH-1];       // Receive buffer length.
BYTE gbytRXbufptr;                             // Receive buffer length pointer.

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//

//#define	_UART_BAUDRATE_kBPS	9.6	// Default datarate in kilobits-per-second, for HC-05 module.
//#define	_UART_BAUDRATE_kBPS	38.4	// Default datarate in kilobits-per-second, for HC-05 module.
#define	_UART_BAUDRATE_kBPS	115.2	// Default datarate in kilobits-per-second, for HC-05 module.

///
/// Process name	: Proce_UART1_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 9 March 2019
///
/// Code version	: 1.23
///
/// Processor		: dsPIC33EP256GMU8XX family.
///                       
///
/// Processor/System Resource 
/// PINS		: 1. Pin RE4 = U1TX (Remappable I/O RP84), output.
///                          Alternative Pin RG6 (RP118)
///  			  2. Pin RE5 = U1RX (Remappable I/O RP85), input.
///                          Alternative Pin RG7 (RPI119)
///                       3. PIN_ILED2 = indicator LED2.
///
/// MODULES		: 1. UART1 (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytRXbuffer[]
///                       gbytRXbufptr
///                       gbytTXbuffer[]
///                       gbytTXbufptr
///                       gbytTXbuflen
///                       gSCIstatus
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_UART1_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_UART1_Driver: An RTOS is required with this function"
#endif

///
/// Description		: 1. Driver for built-in UART1 Module.
///                   2. Serial Communication Interface (UART) transmit buffer manager.
///                      Data will be taken from the SCI transmit buffer gbytTXbuffer in FIFO basis
///                      and transmitted via USART module.  Maximum data length is determined by the
///			             constant _SCI_TXBUF_LENGTH in file "osmain.h".
///                   3. Serial Communication Interface (UART) receive buffer manager.
///			     Data received from the USART module of the microcontroller will be
///			     transferred from the USART registers to the RAM of the microcontroller
///			     called SCI receive buffer (gbytRXbuffer[]).
///			     The flag bRXRDY will be set to indicate to the user modules that valid
///			     data is present.
///			     Maximum data length is determined by the constant _SCI_RXBUF_LENGTH in
///			     file "osmain.h".
///
/// Version 1.00 - First stable version.
/// Version 1.01 - Add checking for framing error in received data.  Version 1.00 only checks for 
///                overflow error.
///
/// Example of usage : The codes example below illustrates how to send 2 bytes of character,
///			'a' and 'b' via UART.
///         if (gSCIstatus.bTXRDY == 0)	// Check if any data to send via UART.
///         {
///             gbytTXbuffer[0] = 'a';	// Load data.
///		   	    gbytTXbuffer[1] = 'b';
///		   	    gbytTXbuflen = 2;		// Set TX frame length.
///		  	    gSCIstatus.bTXRDY = 1;	// Initiate TX.
///         }
///
/// Example of usage : The codes example below illustrates how to retrieve 1 byte of data from
///                    the UART receive buffer.
///         if (gSCIstatus.bRXRDY == 1)	// Check if UART receive any data.
///		    {
///             if (gSCIstatus.bRXOVF == 0) // Make sure no overflow error.
///			    {
///                 bytData = gbytRXbuffer[0];	// Get 1 byte and ignore all others.
///             }
///             else
///             {
///                 gSCIstatus.bRXOVF = 0; 	// Reset overflow error flag.
///             }
///             gSCIstatus.bRXRDY = 0;	// Reset valid data flag.
///             gbytRXbufptr = 0; 		// Reset pointer.
///			}
///
/// Note: Another way to check for received data is to monitor the received buffer pointer
/// gbytRXbufptr.  If no data this pointer is 0, a value greater than zero indicates the
/// number of bytes contain in the receive buffer.


void Proce_UART1_Driver(TASK_ATTRIBUTE *ptrTask)
{

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - UART1 Initialization.
				// Setup IO pins mode and configure the remappable peripheral pins:

				//TRISEbits.TRISE5 = 1;			// Set RE5 (RP85) to input.
				//RPINR18bits.U1RXR = 0b1010101;          // RP85 tied to U1RX.
				//RPOR5bits.RP84R = 0b000001;		// RP84 (RE4) tied to U1TX.
									// Note: Once this is done, the controller will 
									// override the I/O mode in the original TRISx 
									// register.

                                TRISGbits.TRISG7 = 1;                   // Set RG7 (RPI119) to input.
                                RPINR18bits.U1RXR = 0b1110111;          // RPI119 tied to U1RX.
                                RPOR13bits.RP118R = 0b000001;            // RP118 tied to U1TX.

				// Setup baud rate generator register, this requires BRGH=1:
				// NOTE: This formula is specified in the manner shown to prevent overflow of the 
				// Integer (16-bits) during the arithmetic operation.
			
				U1BRG = (__FOSC_MHz/8)*(1000/_UART_BAUDRATE_kBPS)-1;
                                
				// Setup UART1 operation mode part 1:
				// 1. Enable UART1 module.
				// 2. Continue module operation in idle, no wake-up enabled.
				// 3. Loopback mode disabled.
				// 4. No auto-baud rate detect.	
				// 5. Only U1TX and U1RX pins are used, no flow control.
				// 6. 8 bits data, even parity, 1 stop bit.
				// 7. Sync break transmission disabled.
				// 8. No interrupt when any character is received (the U1RSR can take in 4 characters).
				// 9. Address detect is disabled.
				U1MODEbits.PDSEL = 0b01;        // 8 bits, even parity.
				U1MODEbits.BRGH = 1;            // High-speed clock generation mode (better).
				U1MODEbits.UARTEN = 1;          // Enable UART.
				U1MODEbits.UEN = 0b00;          // Only U1TX and U1RX pins are used. U1CTS, U1RTS and BCLK
                                                // pins are controlled by latches.
                U1MODEbits.PDSEL = 0b00;        // 8 bits data, no parity.
				U1MODEbits.STSEL = 0;           // 1 stop bit.
                U1MODEbits.UARTEN = 1;          // Enable UART1 module.
				U1STAbits.UTXEN = 1;            // Enable Transmit module in UART.
                //U1STAbits.URXEN = 1;          // NOTE: This bit is not available in all dsPIC33E devices.
				gbytTXbuflen = 0;               // Initialize all relevant variables and flags.
				gbytTXbufptr = 0;
				gSCIstatus.bRXRDY = 0;	
				gSCIstatus.bTXRDY = 0;
				gSCIstatus.bRXOVF = 0;
                //U1STAbits.OERR = 0;
				gbytRXbufptr = 0;
                PIN_ILED2 = 0;                      // Off indicator LED2.
				OSSetTaskContext(ptrTask, 10, 100);  // Next state = 10, timer = 100.
			break;
			
			case 1: // State 1 - Transmit and receive buffer manager.
				// Check for data to send via UART.
				// Note that the transmit FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
				if (gSCIstatus.bTXRDY == 1)                         // Check if valid data in SCI buffer.
				{
                    while (U1STAbits.UTXBF == 0)                    // Check if UART transmit buffer is not full.
                                                                    //conditions to exit the load transmit buffer
                                                                    // routines, (1) When transmit butter is full or
                                                                    // (2) when transmit buffer pointer equals
                                                                    // transmit data length.
                    {
                        PIN_ILED2 = 1;                              // On indicator LED2.
                        if (gbytTXbufptr < gbytTXbuflen)
                        {
                            U1TXREG = gbytTXbuffer[gbytTXbufptr];  // Load 1 byte data to UART transmit buffer.
                            gbytTXbufptr++;                         // Pointer to next byte in TX buffer.
                        }
                        else
                        {
                            gbytTXbufptr = 0;                       // Reset TX buffer pointer.
                            gbytTXbuflen = 0;                       // Reset TX buffer length.
                            gSCIstatus.bTXRDY = 0;                  // Reset transmit flag.
                            PIN_ILED2 = 0;                          // Off indicator LED2.
                            break;
                        }
                    }
				}

				// Check for data to receive via UART.
				// Note that the receive FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
                // Here we ignore the Parity error since by default the serial data has no parity.  
                // If overflow error is detected, the dsPIC33 UART receive circuitry will hangs, we need 
                // to reset it by clearing the OERR bits in UxSTA register.
                                
                if ((U1STAbits.OERR == 0) && (U1STAbits.FERR == 0)) // Make sure no overflow and framing error.
                {
                    while (U1STAbits.URXDA == 1)                    // If at least 1 data byte is
                                                                    // available at UART
                    {
                        PIN_ILED2 = 1;                              // On indicator LED2.
                        if (gbytRXbufptr < (__SCI_RXBUF_LENGTH-1))	// check for data overflow.
                        {                                           // Read a character from USART.
                            gbytRXbuffer[gbytRXbufptr] = U1RXREG;   // Get received data byte.
                            gbytRXbufptr++;                         // Pointer to next byte in RX buffer.
                            gSCIstatus.bRXRDY = 1;                  // Set valid data flag.
                        }
                        else                                        // data overflow.
                        {
                            gbytRXbufptr = 0;                       // Reset buffer pointer.
                            gSCIstatus.bRXOVF = 1;                  // Set receive data overflow flag.
                        }
                    }
                }
                else                                            // Overflow/overrun error.
                {
                    U1STAbits.OERR = 0;                         // Clear overrun flag, this will also
                                                                // clear and reset the receive FIFO in
                                                                // the UART module.
                    U1STAbits.FERR = 0;                         // Clear the framing error flag.
                    gbytRXbufptr = 0;                           // Reset buffer pointer.
                    gSCIstatus.bRXOVF = 1;                      // Set receive data overflow flag.
                } 
				OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.
			break;

            case 10: // State 10 - Keep on reading the receive buffer to clear the receive status flag.
                     // Note: 1 May 2015, I noticed that upon power up, the UART module may contains invalid
                     // data, and the URXDA flag will be set to 1.  Thus we should clear this before
                     // proceeding. 
                if (U1STAbits.URXDA == 1)
                {
                    gbytRXbuffer[gbytRXbufptr] = U1RXREG;               // Read the receive buffer.
                    OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                }
            break;

			default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
				OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
			break;
		}
	}
}


