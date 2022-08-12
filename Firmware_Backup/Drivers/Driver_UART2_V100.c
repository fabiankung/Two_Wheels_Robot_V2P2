  //////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_UART2_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 31 July 2019
// Toolsuites		: Microchip MPLAB X IDE v5.10 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above

#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

#define     __SCI_TXBUF_LENGTH2     16
#define     __SCI_RXBUF_LENGTH2     16
//
// --- PUBLIC VARIABLES ---
//
// Data buffer and address pointers for wired serial communications (UART).
BYTE gbytTXbuffer2[__SCI_TXBUF_LENGTH2-1];          // Transmit buffer.
BYTE gbytTXbufptr2;                                 // Transmit buffer pointer.
BYTE gbytTXbuflen2;                                 // Transmit buffer length.
BYTE gbytRXbuffer2[__SCI_RXBUF_LENGTH2-1];          // Receive buffer length.
BYTE gbytRXbufptr2;                                 // Receive buffer length pointer.

SCI_STATUS gSCIstatus2;

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//
//#define	_UART2_BAUDRATE_kBPS	38.4	// Default datarate in kilobits-per-second, for general purpose usage.
//#define	_UART2_BAUDRATE_kBPS	19.2	// Default datarate in kilobits-per-second, for G15 servo motor.
//#define	_UART2_BAUDRATE_kBPS	9.6	// Default datarate in kilobits-per-second, for HC-05 module.
//#define	_UART2_BAUDRATE_kBPS	57.6	// Default datarate in kilobits-per-second, for dynamixel servo motor.
#define	_UART2_BAUDRATE_kBPS	115.2	// Default datarate in kilobits-per-second, for dynamixel servo motor.

///
/// Process name	: Proce_UART2_Driver
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
/// PINS		: 1. Pin RB5 = U2RX (Remappable I/O RPI37), input.
///  			  2. Pin RE5 = U2TX (Remappable I/O RP85), output.
///                3. PIN_ILED2 = indicator LED2.
///
/// MODULES		: 1. UART2 (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytRXbuffer2[]
///                       gbytRXbufptr2
///                       gbytTXbuffer2[]
///                       gbytTXbufptr2
///                       gbytTXbuflen2
///                       gSCIstatus2
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_UART2_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_UART2_Driver: An RTOS is required with this function"
#endif

///
/// Description		: 1. Driver for built-in UART2 Module.
///                       2. Serial Communication Interface (UART) transmit buffer manager.
///                          Data will be taken from the SCI transmit buffer gbytTXbuffer in FIFO basis
///                          and transmitted via USART module.  Maximum data length is determined by the
///			     constant _SCI_TXBUF_LENGTH in file "osmain.h".
///                       3. Serial Communication Interface (UART) receive buffer manager.
///			     Data received from the UART module of the microcontroller will be
///			     transferred from the UART registers to the RAM of the microcontroller
///			     called SCI receive buffer (gbytRXbuffer2[]).
///			     The flag bRXRDY will be set to indicate to the user modules that valid
///			     data is present.
///			     Maximum data length is determined by the constant _SCI_RXBUF_LENGTH in
///			     file "osmain.h".
///
///
/// Example of usage : The codes example below illustrates how to send 2 bytes of character,
///			'a' and 'b' via UART2.
///           if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
///           {
///             gbytTXbuffer2[0] = 'a';	// Load data.
///		   	    gbytTXbuffer2[1] = 'b';
///		   	    gbytTXbuflen2 = 2;		// Set TX frame length.
///		  	    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
///           }
///
/// Example of usage : The codes example below illustrates how to retrieve 1 byte of data from
///                    the UART receive buffer.
///            if (gSCIstatus2.bRXRDY == 1)	// Check if UART receive any data.
///		       {
///                 if (gSCIstatus2.bRXOVF == 0) // Make sure no overflow error.
///                 {
///                      bytData = gbytRXbuffer2[0];	// Get 1 byte and ignore all others.
///                 }
///                 else
///                 {
///                     gSCIstatus2.bRXOVF = 0; 	// Reset overflow error flag.
///                 }
///                         gSCIstatus2.bRXRDY = 0;	// Reset valid data flag.
///                         gbytRXbufptr2 = 0; 		// Reset pointer.
///			}
///
/// Note: Another way to check for received data is to monitor the received buffer pointer
/// gbytRXbufptr.  If no data this pointer is 0, a value greater than zero indicates the
/// number of bytes contain in the receive buffer.


void Proce_UART2_Driver(TASK_ATTRIBUTE *ptrTask)
{

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - UART2 Initialization.
				// Setup IO pins mode and configure the remappable peripheral pins:

				TRISEbits.TRISE5 = 0;			// Set RE5 (RP85) to output.
				RPOR6bits.RP85R = 0b000011;		// RP85 (RE5) tied to U2TX.
									// Note: Once this is done, the controller will 
									// override the I/O mode in the original TRISx 
									// register.

                TRISBbits.TRISB5 = 1;                   // Set RB5 (RPI37) to input.
                RPINR19bits.U2RXR = 0b0100101;          // RPI37 tied to U2RX.

				// Setup baud rate generator register, this requires BRGH=1:
				// NOTE: This formula is specified in the manner shown to prevent overflow of the 
				// Integer (16-bits) during the arithmetic operation.
			
				U2BRG = (__FOSC_MHz/8)*(1000/_UART2_BAUDRATE_kBPS)-1;
                                
				// Setup UART2 operation mode part 1:
				// 1. Enable UART2 module.
				// 2. Continue module operation in idle, no wake-up enabled.
				// 3. Loop-back mode disabled.
				// 4. No auto-baud rate detect.	
				// 5. Only U2TX and U2RX pins are used, no flow control.
				// 6. 8 bits data, even parity, 1 stop bit.
				// 7. Sync break transmission disabled.
				// 8. No interrupt when any character is received (the U1RSR can take in 4 characters).
				// 9. Address detect is disabled.
				U2MODEbits.PDSEL = 0b01;        // 8 bits, even parity.
				U2MODEbits.BRGH = 1;            // High-speed clock generation mode (better).
				U2MODEbits.UARTEN = 1;          // Enable UART2.
				U2MODEbits.UEN = 0b00;          // Only U2TX and U2RX pins are used. U2CTS, U2RTS and BCLK
                                                // pins are controlled by latches.
                U2MODEbits.PDSEL = 0b00;        // 8 bits data, no parity.
				U2MODEbits.STSEL = 0;           // 1 stop bit.
                U2MODEbits.UARTEN = 1;          // Enable UART2 module.
				U2STAbits.UTXEN = 1;            // Enable Transmit module in UART2.
                                //U2STAbits.URXEN = 1;          // NOTE: This bit is not available in all dsPIC33E devices.
				gbytTXbuflen2 = 0;               // Initialize all relevant variables and flags.
				gbytTXbufptr2 = 0;
				gSCIstatus2.bRXRDY = 0;
				gSCIstatus2.bTXRDY = 0;
				gSCIstatus2.bRXOVF = 0;
                                //U2STAbits.OERR = 0;
				gbytRXbufptr2 = 0;
                PIN_ILED2 = 0;                      // Off indicator LED2.
				OSSetTaskContext(ptrTask, 10, 100);  // Next state = 10, timer = 100.
			break;
			
			case 1: // State 1 - Transmit and receive buffer manager.
				// Check for data to send via UART.
				// Note that the transmit FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
				if (gSCIstatus2.bTXRDY == 1)                     // Check if valid data in SCI buffer.
				{
                    while (U2STAbits.UTXBF == 0)                // Check if UART transmit buffer is not full.
					// 2 conditions to exit the load transmit buffer
                                                                // routines, (1) When transmit butter is full or
                                                                // (2) when transmit buffer pointer equals
                                                                // transmit data length.
                    {
                        PIN_ILED2 = 1;                          // On indicator LED2.
                        if (gbytTXbufptr2 < gbytTXbuflen2)
                        {
                            U2TXREG = gbytTXbuffer2[gbytTXbufptr2];  // Load 1 byte data to UART transmit buffer.
                            gbytTXbufptr2++;                    // Pointer to next byte in TX buffer.
                        }
                        else
                        {
                            gbytTXbufptr2 = 0;                   // Reset TX buffer pointer.
                            gbytTXbuflen2 = 0;                   // Reset TX buffer length.
                            gSCIstatus2.bTXRDY = 0;              // Reset transmit flag.
                            PIN_ILED2 = 0;                       // Off indicator LED2.
                            break;
                        }
                    }
				}

				// Check for data to receive via UART.
				// Note that the receive FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
                // Here we ignore Framing and Parity error.  If overflow error is detected, the dsPIC33
                // UART receive circuitry will hangs, we need to reset it by clearing the OERR bits in
                // UxSTA register.
                                
                if (U2STAbits.OERR == 0)                        // Make sure no overflow error.
                {
                    while (U2STAbits.URXDA == 1)                // If at least 1 data byte is
                                                                // available at UART
                    {
                        PIN_ILED2 = 1;                          // On indicator LED2.
                        if (gbytRXbufptr2 < (__SCI_RXBUF_LENGTH2-1))	// check for data overflow.
                        {	// Read a character from USART.
                            gbytRXbuffer2[gbytRXbufptr2] = U2RXREG; // Get received data byte.
                            gbytRXbufptr2++;                    // Pointer to next byte in RX buffer.
                            gSCIstatus2.bRXRDY = 1;             // Set valid data flag.
                        }
                        else                                    // data overflow.
                        {
                            //gbytRXbuffer2[gbytRXbufptr2-1] = U2RXREG; // Get received data byte.
                            gbytRXbufptr2 = 0;			// Reset buffer pointer.
                            gSCIstatus2.bRXOVF = 1;		// Set receive data overflow flag.
                        }
                        //PIN_ILED2 = 0;
                    }
                }
                else                                            // Overflow/overrun error.
                {
                    U2STAbits.OERR = 0;                         // Clear overrun flag, this will also
                                                                // clear and reset the receive FIFO in
                                                                // the UART module.
                    gbytRXbufptr2 = 0;                          // Reset buffer pointer.
                    gSCIstatus2.bRXOVF = 1;                     // Set receive data overflow flag.
                } 
				OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.
			break;

            case 10: // State 10 - Keep on reading the receive buffer to clear the receive status flag.
                     // Note: 1 May 2015, I noticed that upon power up, the UART module may contains invalid
                     // data, and the URXDA flag will be set to 1.  Thus we should clear this on power up, before
                     // proceeding.
                if (U2STAbits.URXDA == 1)
                {
                    gbytRXbuffer2[gbytRXbufptr2] = U2RXREG;               // Read the receive buffer.
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


