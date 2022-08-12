//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016-2020, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_I2C_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 16 July 2020
// Toolsuites		: Microchip MPLAB X IDE v5.35 or above
//                	  MPLAB XC16 C-Compiler v1.50 or above

#include "osmain.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
//
// --- PUBLIC VARIABLES ---
//

// Data buffer and address pointers for wired serial communications.
#define     __MAX_I2C_DATA_BYTE               16     // Number of bytes for I2C receive and transmit buffer.
#define     __I2C_TIMEOUT_COUNT               25    // No. of system ticks before the I2C routine timeout during
                                                    // read data stage.
//#define     __I2C_BAUD_RATE_MHZ               0.2   // 200 kHz
#define     __I2C_BAUD_RATE_MHZ               0.4   // 400 kHz

I2C_STATUS  gI2CStat;                   // I2C status.
BYTE        gbytI2CSlaveAdd;            // Slave address (7 bit, from bit0-bit6).
BYTE        gbytI2CRegAdd;              // Slave register address.
BYTE        gbytI2CByteCount;           // No. of bytes to read or write to Slave.
BYTE        gbytI2CRXbuf[__MAX_I2C_DATA_BYTE];                // Data read from Slave register.
BYTE        gbytI2CTXbuf[__MAX_I2C_DATA_BYTE];               // Data to write to Slave register.

///
/// Function name	: Proce_I2C_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 16 June 2020
///
/// Code Version	: 0.99
///
/// Processor/System Resource
/// PINS            : 1. Pin RF5 = I2C2 SCL, output.
///                   2. Pin RF4 = I2C2 SDA, input/output.
///
/// MODULES         : 1. I2C2 (Internal).
///
/// RTOS            : Ver 1 or above, round-robin scheduling.
///
/// Global Variables    :

#ifdef __OS_VER			// Check RTOS version compatibility.
	#if __OS_VER < 1
		#error "Proce_I2C_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_I2C_Driver: An RTOS is required with this function"
#endif

///
/// Description	:
/// This is a driver routines for I2C2 module of the dsPIC33E processor.  Due to the pin
/// assignment of the dsPIC33E core, I2C1 module cannot be used as the pins are already
/// in use for other application.  
/// The operation mode is the dsPIC33E processor assume the role of I2C Master, with
/// multiple slave devices.
/// This driver handles the low-level transmit and receive operations. 
/// Ver 0.91, 15 Nov 2015 - Basic version.
/// Ver 0.95, 17 Aug 2016 - Improved efficiency of multi-bytes read.
/// Ver 0.96, 3 April 2018 - Further improvement in efficiency of multi-bytes read.
/// Ver 0.97, 13 July 2018 - Further improve the efficiency of multi-bytes read by 1 clock tick! 
/// Ver 0.99, 16 June 2020 - Further improve the efficiency of multi-bytes read and transmit by 1 clock tick!

/// I2C bus properties:
/// Baud rate = 200 kHz.
/// Mode: Single Master.
///
/// --- Example of usage: Transmit operation ---
/// Suppose we want to update the registers of a Slave device:
/// Register address 0x20: data = 0xFA
/// Register address 0x21: data = 0xCD
/// The Slave device has an address 0x1E.
///
/// The codes as follows:
/// if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
/// {
///     gbytI2CByteCount = 2;       // Indicate no. of bytes to transmit.
///     gbytI2CRegAdd = 0x20;       // Start address of register.
///     gbytI2CTXbuf[0] = 0xFA;
///     gbytI2CTXbuf[1] = 0xCD;
///     gbytI2CSlaveAdd =  0x1E;
///     gI2CStat.bSend = 1;
/// }
/// The user routine can monitor the flag gI2CStat.bI2CBusy or gI2CStat.bSend.  Once the
/// transmission is completed, both flags will be cleared by the driver.
///
/// --- Example of usage: Receive operation ---
/// Suppose we want to receive 1 byte from the Slave device:
/// Register address 0x30.
/// Slave device address: 0x1E.
///
/// if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
/// {
///     gbytI2CByteCount = 1;       // Indicate no. of bytes to read.
///     gbytI2CRegAdd = 0x30;       // Start address of register.
///     gbytI2CSlaveAdd =  0x1E;
///     gI2CStat.bRead = 1;
/// }
/// The user routine can monitor the flag gI2CStat.bI2CBusy or gI2CStat.bRead.  Once the
/// reception is completed, both flags will be cleared by the driver.  The received
/// data will be stored in gbytI2CRXbuf[0].
/// if (gI2CStat.bRead == 0)       // Check if Read operation is completed.
/// {                              // Read operation complete, check received data.
///     User codes here
/// }
/// else if (gI2CStat.bCommError == 1)  // Check for I2C bus error.
/// {
/// }

void Proce_I2C_Driver(TASK_ATTRIBUTE *ptrTask)
{
    static int nIndex = 0;
    static int nTimeOut = 0;
    static int nCount = 0;

    if (ptrTask->nTimer == 0)
    {
	switch (ptrTask->nState)
	{
            case 0: // State 0 - Initialization of I2C2 module and set as Master mode.
                gI2CStat.bCommError = 0;                // Clear error flag.
                gI2CStat.bI2CBusy = 1;                  // Initially indicates I2C module is busy.
                gbytI2CRXbuf[0] = 0;                    // Once it is ready we will clear the busy flag.
                gbytI2CRegAdd = 0;
                gI2CStat.bSend = 0;
                gI2CStat.bRead = 0;

                // Note: 23 July 2015.  According to the datasheet for dsPIC33EP256GP806, the pulse
                // gobbler delay ranges from 65-390 nsec, with 130 nsec being the typical value.
                I2C2BRG = ((1/__I2C_BAUD_RATE_MHZ)-0.13)*(__FCYC_MHz) - 1;
                I2C2CONbits.I2CEN = 1;                  // Enable I2C2 module.
                OSSetTaskContext(ptrTask, 52, 1000);     // Next state = 52, timer = 1000.  This state generate
                                                        // a STOP condition, putting the I2C bus in idle mode.
                nIndex = 0;                             // Reset the index.
            break;

            case 1: // State 1 - Dispatcher.
                if (gI2CStat.bRead == 1)                  // Reading data from Slave.
                {
                    I2C2CONbits.SEN = 1;                  // Assert Start condition on I2C bus.
                    nCount = 0;                           // Reset counter.                    
                    gI2CStat.bI2CBusy = 1;                // Indicate I2C module is occupied.
                    OSSetTaskContext(ptrTask, 31, 1);     // Next state = 31, timer = 1.
                }
                else if (gI2CStat.bSend == 1)             // Transmission of data to Slave.
                {
                    nCount = 0;                           // Reset pointer.
                    I2C2CONbits.SEN = 1;                  // Assert Start condition on I2C bus.                    
                    gI2CStat.bI2CBusy = 1;                // Indicate I2C module is occupied.
                    OSSetTaskContext(ptrTask, 46, 1);     // Next state = 46, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);     // Next state = 1, timer = 1.
                }
                break;

            // --- Multi-bytes read ---
            case 30: // State 30 - Assert Start condition.
                if (I2C2STATbits.P == 1)                    // Make sure the I2C bus is in idle condition.
                {
                    I2C2CONbits.SEN = 1;                    // Assert Start condition on I2C bus.
                    nCount = 0;                             // Reset counter.
                    OSSetTaskContext(ptrTask, 31, 1);       // Back to state = 31, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 30, 1);        // Back to state = 30, timer = 1.
                }            
            break;

           case 31: // State 31 - Tx slave device address, write mode.
                I2C2TRN = gbytI2CSlaveAdd<<1;            // Data = Slave address with R/W bit = 0 (Master is writing to the Slave).
                OSSetTaskContext(ptrTask, 32, 1);        // Back to state = 32, timer = 1.
           break;

           case 32: // State 32 - Check status of device address transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 32, 1);    // Back to state = 32, timer = 1.
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;            // Clear communication error flag.
                        I2C2TRN = gbytI2CRegAdd;            // Send register address to read.
                        OSSetTaskContext(ptrTask, 34, 1);   // Next state = 34, timer = 1.
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 41, 1);    // Next state = 41, timer = 1.
                    }    
                }
                break;

           case 34: // State 34 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 34, 1);   // Back to state = 34, timer = 1.
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;        // Clear communication error flag.
                        I2C2CONbits.RSEN = 1;                // Assert repeat start condition on I2C bus.
                        OSSetTaskContext(ptrTask, 36, 1);    // Next state = 36, timer = 1.
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 41, 1);    // Next state = 41, timer = 1.
                    }                   
                }
                break;

           case 36: // State 36 - TX slave device address, read mode.
                I2C2TRN = (gbytI2CSlaveAdd<<1) | 0x01;     // Data = Slave address with R/W bit = 1 (Master is reading from the Slave).
                OSSetTaskContext(ptrTask, 37, 1);        //  Next state = 37, timer = 1.
                break;

           case 37: // State 37 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 37, 1);    // Back to state = 37, timer = 1.
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;            // Clear communication error flag.
                        I2C2CONbits.RCEN = 1;               // Enable receive operation.
                        nTimeOut = 0;                       // Reset timeout timer.                        
                        OSSetTaskContext(ptrTask, 39, 1);   // Next state = 39, timer = 1.
                                                            // Note: we can actually jump to state 38, but this
                                                            // will cause one tick delay.
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 41, 1);    // Next state = 41, timer = 1.
                    }                 
                }
                break;
                
           case 38: // State 38 - Enable receive.
               I2C2CONbits.RCEN = 1;                    // Enable receive operation.
               nTimeOut = 0;                            // Reset timeout timer.
               OSSetTaskContext(ptrTask, 39, 1);        // Next state = 39, timer = 1.
               break;
             
           case 39: // State 39 - Check for receive buffer full status.
                if (I2C2STATbits.RBF == 1)              // Check if all bits are received.
                {
                    gbytI2CRXbuf[nCount] = I2C2RCV;      // Get received data.
                    gbytI2CByteCount--;
                    nCount++;
                    // Check for end of data to read, or the I2C receive buffer is full.
                    if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                    {
                        I2C2CONbits.ACKDT = 1;              // Set NAK when receive data, because expected data bytes is nil.
                        I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                        gI2CStat.bI2CBusy = 0;              // I2C module is idle.
                        gI2CStat.bRead = 0;                 // Clear read flag.    
                        OSSetTaskContext(ptrTask, 41, 1);   // Next state = 41, timer = 1.
                    }
                    else                                    // Still in reading mode.
                    {
                        I2C2CONbits.ACKDT = 0;              // Set ACK when receive data.
                        I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.  
                        OSSetTaskContext(ptrTask, 38, 1);   // Next state = 38, timer = 1.
                    }                    
                }
                else
                {
                    nTimeOut++;
                    if (nTimeOut > __I2C_TIMEOUT_COUNT)  // Check for timeout.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        gbytI2CByteCount--;
                        nCount++;
                        // Check of end of data to read, or the I2C receive buffer is full.
                        if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                        {
                            I2C2CONbits.ACKDT = 1;              // Set NAK when receive data.
                            I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                            gI2CStat.bI2CBusy = 0;              // I2C module is idle.
                            gI2CStat.bRead = 0;                 // Clear read flag.                             
                            OSSetTaskContext(ptrTask, 41, 1);   // Next state = 41, timer = 1.
                        }
                        else                                    // Still in reading mode.
                        {
                            I2C2CONbits.ACKDT = 0;              // Set ACK when receive data.
                            I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                            OSSetTaskContext(ptrTask, 38, 1);   // Next state = 38, timer = 1.
                        }
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 39, 1);   // Next state = 39, timer = 1.
                    }
                }
                break;

            case 41: // State 41 - End, initiate Stop condition on I2C bus and tidy up.
                     // Also to hasten the response of this driver, we run the dispatcher in this state.
                     // If there no data to be read from I2C bus, then the state-machine will revert
                     // to state 1 and wait for new commands.  By running the dispatcher here,
                     // the driver can immediately execute another read operation right after the 
                     // stop condition is generated.
                I2C2CONbits.PEN = 1;                        // Initiate Stop condition.
                if (gI2CStat.bRead == 1)                    // Reading data from Slave.
                {
                    gI2CStat.bI2CBusy = 1;                  // Indicate I2C module is occupied.
                    OSSetTaskContext(ptrTask, 30, 1);       // Next state = 30, timer = 1.
                }
                else if (gI2CStat.bSend == 1)               // Transmission of data to Slave.
                {
                    gI2CStat.bI2CBusy = 1;                  // Indicate I2C module is occupied.
                    OSSetTaskContext(ptrTask, 45, 1);       // Next state = 45, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);        // Next state = 1, timer = 1.
                }                
                break;


            // --- Multi-bytes write ---
            case 45: // State 45 - Assert Start condition.
                nCount = 0;                             // Reset pointer.
                I2C2CONbits.SEN = 1;                    // Assert Start condition on I2C bus.
                OSSetTaskContext(ptrTask, 46, 1);       // Next state = 46, timer = 1.
                break;

           case 46: // State 46 - Tx slave device address, write mode.
                I2C2TRN = gbytI2CSlaveAdd<<1;          // Data = Slave address with R/W bit = 0 (Master is writing to the Slave).
                OSSetTaskContext(ptrTask, 47, 1);       // Next state = 47, timer = 1.
           break;

           case 47: // State 47 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 47, 1);    // Next state = 47, timer = 1.
                }
                else                                    // Transmission end, check acknowledg from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, proceed.
                    {
                        gI2CStat.bCommError = 0;
                        OSSetTaskContext(ptrTask, 48, 1);    // Next state = 48, timer = 1.
                    }
                    else                            // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 52, 1);    // Next state = 52, timer = 1.
                    }                
                }
                break;
           case 48: // State 48 - TX start register to write to.
                I2C2TRN = gbytI2CRegAdd;                 // Send register address to update.
                OSSetTaskContext(ptrTask, 49, 1);        // Next state = 49, timer = 1.
                break;

           case 49: // State 49 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 49, 1);    // Next state = 49, timer = 1.
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, proceed.
                    {
                        gI2CStat.bCommError = 0;
                        OSSetTaskContext(ptrTask, 50, 1);    // Next state = 50, timer = 1.
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 52, 1);    // Next state = 52, timer = 1.
                    }                   
                }
                break;

            case 50: // State 50 - TX data to Slave.
                I2C2TRN = gbytI2CTXbuf[nCount];          // Upload data for Slave.
                OSSetTaskContext(ptrTask, 51, 1);        // Next state = 51, timer = 1.
                break;

            case 51: // State 51 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    OSSetTaskContext(ptrTask, 51, 1);    // Next state = 51, timer = 1.
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave.
                    {
                        gbytI2CByteCount--;
                        nCount++;
                        gI2CStat.bCommError = 0;
                        // Check for end of byte or reach end of I2C transmit buffer.
                        if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                        {
                            OSSetTaskContext(ptrTask, 52, 1);    // Next state = 52, timer = 1.
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 50, 1);    // Next state = 50, timer = 1.
                        }
                    }
                    else                                // NAK received from Slave.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        OSSetTaskContext(ptrTask, 52, 1);    // Next state = 52, timer = 1.
                    }
                }
                break;

            case 52: // State 52 - End, initiate Stop condition on I2C bus.
                I2C2CONbits.PEN = 1;                    // Initiate Stop condition.
                //OSSetTaskContext(ptrTask, 53, 1);        // Next state = 53, timer = 1.
                gI2CStat.bI2CBusy = 0;                  // I2C module is idle.
                gI2CStat.bSend = 0;
                OSSetTaskContext(ptrTask, 1, 1);        // Next state = 1, timer = 1.                
                break;
/*
            case 53: // State 53 - Tidy up.
                gI2CStat.bI2CBusy = 0;                  // I2C module is idle.
                gI2CStat.bSend = 0;
                OSSetTaskContext(ptrTask, 1, 1);        // Next state = 1, timer = 1.
                break;
*/
            default:
		OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}