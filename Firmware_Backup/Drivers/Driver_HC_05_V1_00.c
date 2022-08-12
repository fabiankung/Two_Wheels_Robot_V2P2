//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016-2022, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved
//
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_HC_05_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 27 Jan 2022
// Toolsuites		: Microchip MPLAB X IDE v5.50 or above
//                	  MPLAB XC16 C-Compiler v1.70 or above

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder.
#include "../osmain.h"
#include "../Driver_UART1_V100.h"
//#include "C:\Users\wlkung\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"

#define     _DRIVER_HC_05_NO_TX             0
#define     _DRIVER_HC_05_TX_BINARYDATA     1
#define     _DRIVER_HC_05_TX_TEXTDATA       2

typedef struct StructProceHC05Driver
{
    unsigned char bytRFCommand;     // Command byte received from remote host.        
    unsigned char bytRFArgument1;   // Argument 1 for the command byte.  
    unsigned char bytRFArgument2;   // Argument 2 for the command byte.  
    unsigned char bytRFAdd;         // ID for the device connected to this Bluetooth module. From 1 to 255.
                                    // 0 is for broadcasting purpose.
    int nRFLinkState;               // Status to indicate RF link with remote host.
                                    // 1 = RF link established with remote PC host or Tablet or similar computers.  These
                                    // devices usually have large display and ability to display graphs.  With these 
                                    // devices we can transmit robot's status and sensors output periodically and have
                                    // the data display as graphs on the remote host.  When gnRFLinkState = 1, the other
                                    // user processes can set gnEnRFTxPeriodicData = 1 to enable periodic data transmission
                                    // from robot to remote host.  Else it is not advisable to set this flag.
                                    // 2 = RF link established with remote host such as smartphone or similar devices
                                    // with small display screen.  
                                    // Otherwise = No RF link.    
    int nEnRFTxPeriodicData;        // Indicate whether to send periodic data packet to remote host.
                                    // _DRIVER_HC_05_NO_TX or else = Don't transmit data periodically.
                                    // _DRIVER_HC_05_TX_BINARYDATA = Transmit data periodically, binary format.
                                    // _DRIVER_HC_05_TX_TEXTDATA = Transmit data periodically, ASCII format.    
    int nDriverBusy;                // 1 or larger = True. True when there is still pending data to be send.
                                    // 0 or negative = idle.
} DSPIC33E_HC05_DRIVER;

DSPIC33E_HC05_DRIVER    gobjDriverHC05;

int gnEnHC05Init = 0;           // 0 = Don't initialize the attached HC-05 module on power up.
                                // 1 = Initialize the device name and baud rate of attached
                                //     HC-05 module on power up.
                                //     Note: Initialization is usually needed
                                //     when one uses the HC-05 module for the first time,
                                //     unless one uses the default name and baud rate.

///
/// Process name	: Proce_BT_HC_05_CommStack - Communication Stack with Remote Station over Bluetooth UART.
///
/// Author          : Fabian Kung
///
/// Last modified	: 27 Jan 2022
///
/// Code version	: 1.25
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN			:  PIN_ILED2
///                PIN_HC_05_RESET
///                PIN_HC_05_KEY
///
/// MODULE		:  UART1 (Internal)
///                HC-05 Bluetooth Module (External)
///
/// DRIVER		:  Driver_UART3_V100
///
/// RTOS		:  Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytTXbuffer, gbytRXbuffer and others.
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce__BT_HC_05_CommStack: Incompatible OS version"
	#endif
#else
	#error "Proce__BT_HC_05_CommStack: An RTOS is required with this function"
#endif

#define PIN_HC_05_RESET         _RF0                    // Pin RF0 = Reset pin for HC-05 bluetooth module.
#define PIN_HC_05_KEY           _RF1                    // Pin RF1 = Key pin for HC-05 bluetooth module.

///
/// Description		: Driver and wireless communication stack for HC-05 bluetooth module.  The HC-05 
/// module will acts as Bluetooth Server with the remote Bluetooth host as the Client.               
///
/// This module initializes the HC-05 bluetooth module, and communicates with it using the built-in UART
/// module of the processor (UART1). Upon power up, it will (if gnEnHC05Init = 1)
/// 1. Set the HC-05 module into AT Command mode.
/// 2. Change the default baud rate of HC-05 at 9600 bps to 115000 bps and update the Bluetooth device
///    name.
/// 3. Wait for SPP (serial-port protocol) link to be initiated by the Remote Bluetooth (BT) host.
/// 4. Once SPP link is established with the Remote BT host, global variable 'gnRFLinkState' will be
///    set to 1 or 2, depending on whether the Remote BT host is a PC/Tablet or smartphone.  The user
///    processes can thus make use of this info and decide on the duty cycle of the communication rate
///    and also the type of information to send back to the remote BT host.
/// 5. After this the modules goes into message clearing loop, monitoring data packet from Remote Host
/// and performing the necessary message handling routines.
/// 6. The user codes can monitor gnRFLinkState to ascertain when the wireless SPP link is ready for 
///    bilateral communication
/// If gnEnHC05Init = 0, step (1) and (2) will be skipped, the routine will proceed directly to step (3)
/// upon power up.
///
/// The data packets to and from the Remote Host are as follows:
///
/// To Remote Host:
/// Command: [Dev ID][0x55][Data 0][Data 1]...[Data 5]   Packet length = 8 bytes
/// ASCII Data: [Dev ID][0xAA][Data 0][Data 1]...[Data61]  Packet length = 64 bytes
/// Binary Data: [Dev ID][0xAB][Data 0][Data 1]...[Data61]  Packet length = 64 bytes
///
/// From Remote Host:
/// Command: [Dev ID][0x55][Command][Argument1][Argument2]  Packet length = 5 bytes
/// ASCII Data: [Dev ID][0xAA][Data1][Data2][Data3]  Packet length = 5 bytes
/// Binary Data: [Dev ID][0xAB][Data1][Data2][Data3]  Packet length = 5 bytes
///
/// Usage:
/// 1. Set gnEnHC05Init to 0 or 1 (default is 0) in the user codes within 100 msec on power up.
/// 2. The C source file and header file need to be included to the user project workspace.  In addition
///    the header file should also be included into the user codes.
/// 3. The user needs to provide the implementation of 4 callback functions in user C source codes:
///     int HC05_CommandHandler(void) - Routines to handle command.
///     void HC05_ASCIIHandler(void) - Routines to handle ASCII data.
///     void HC05_BinaryHandler(void) - Routines to handle binary data.
///     void HC05_SendPeriodData(void) - Routine to send back periodic data packet to remote host. 
///                                      The data packet consist of 64 bytes payload, and can be
///                                      ASCII or binary data.
///                                      Assume 1 OS ticks = 0.1667 msec, this routine will be called
///                                      roughly once every 40 msec.  Thus 25 data packet will be send
///                                      every 1 second.  When data is being send, the flag nDriverBusy in
///                                      object gobjDriverHC05 will be set to 1, else nDriverBusy will 
///                                      be set to 0.  Thus external process can monitor this flag to 
///                                      ascertain if transmission of data packet is complete or not.  
///
/// The prototype of these functions are declared in the header file for this driver.  The function can
/// be as simple as a null function, for instance:
/// void HC05_ASCIIHandler() {}
///
/// 4. This driver also has a handy utility where the user can send a string of text to the remote host, in
///    the form of the function SentTexttoRemoteHost().  See the header of the function for usage.
///
/// NOTE: 13 May 2015 - Beware of path and filename length limitation on MPLAB-X IDE!


#define		_SPP_ESTABLISH_LINK         0x10
#define		_SPP_LINK_ESTABLISHED   	0x11
#define     _TUNABLE_COEFFICIENT_INFO   0x12
#define     _COEFFICENT_INFO            0x13
#define     _DEVICE_RESET               0x00
#define     _RES_PARAM                  0x01
#define     _SET_PARAM                  0x02


#define	_BT_UART_BAUDRATE_kBPS	115.2	// Bluetooth UART datarate in kilobits-per-second.

void Proce_BT_HC_05_CommStack(TASK_ATTRIBUTE *ptrTask)
{
    int nTemp;
    static int nTimeoutCounter = 0;
    static int nCounter;
    static int nRFHostType = 0; // 0 = PC or Tablet
                                // Otherwise = Smartphone or similar devices.
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
	{
            case 0: // State 0 - Initialization of HC-05 Bluetooth module.
                PIN_HC_05_RESET = 1;                // Deassert reset pin for bluetooth module.  
                PIN_HC_05_KEY = 0; 
                PIN_ILED2 = 0;                      // Off indicator LED2 in case
                                                    // it is turned on by other
                                                    // processes.
                gobjDriverHC05.nRFLinkState = 0;
                gobjDriverHC05.nEnRFTxPeriodicData = 0;
                gobjDriverHC05.bytRFAdd = 1;        // User to change accordingly.
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 1,
                                                    // timer = 1000 msec. A sufficient long
                                                    // period for the HC-05 Bluetooth module
                                                    // to initialize and settle down.
            break;

            case 1: // State 1 - Check if we need to setup the HC-05 Bluetooth module or not.
                if (gnEnHC05Init == 1)
                {                                   // Need to setup the module by going into
                                                    // the AT Command Mode.
                    PIN_HC_05_KEY = 1;              // Assert Key pin for Bluetooth module.
                }
                OSSetTaskContext(ptrTask, 2, 10*__NUM_SYSTEMTICK_MSEC); // Next state = 2, timer = 10 msec.
                break;
                
            // --- Specifics sequence to reset and configure HC-05 bluetooth module ---
            // 16 Jan 2016:
            // Here we are using Method 2 to get the HC-05 module into AT command mode.
            // Here we first set HC-05 to AT command mode (by setting KEY=1), then reset the module.
            // At this point the default baud rate is 38400 bps, LED1 should blinks at 0.5Hz.
            // After this we change the bluetooth device name and its serial port baud rate to 115200 bps.
            // Once this is done, we deassert the AT command mode, and reset the HC-05
            // module once again. LED1 should now blinks at 2Hz until the HC-05 module is paired.
            // The methods to do this can be found on www.instructables.com
            // or more importantly the documentation for HC-05/03 on the Net.  
            // 27 Jan 2022:
            // Depending on the firmware on the HC-05 module, certain version upon establishing a SPP link
            // with Bluetooth client, will transmit the string "OK\r\n" (0x4F 0x4B 0x0D 0x0A) to the
            // micro-controller board. So we must be able to handle this string.
            case 2: // State 2 - Reset HC-05 bluetooth module.
                PIN_HC_05_RESET = 0;                     // Reset bluetooth module.
                OSSetTaskContext(ptrTask, 3, 100*__NUM_SYSTEMTICK_MSEC);     // Next to state = 3, timer = 100 msec.
                break;

            case 3: // State 3 - Deassert the reset pin.
                PIN_HC_05_RESET = 1;
                if (gnEnHC05Init == 1)
                {   
                    OSSetTaskContext(ptrTask, 4, 200*__NUM_SYSTEMTICK_MSEC);     // Next to state = 4, timer = 0.2 sec.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 8, 200*__NUM_SYSTEMTICK_MSEC);     // Next to state = 8, timer = 0.2 sec.
                }
                break;

            case 4: // State 4 - Change Bluetooth module name to 'RMV1'.
                gSCIstatus.bRFTXRDY = 1;            // Set transmit flag.
                gbytTXbuffer[0] = 'A';              // AT command "AT+NAME=RMON"
                gbytTXbuffer[1] = 'T';
                gbytTXbuffer[2] = '+';
                gbytTXbuffer[3] = 'N';
                gbytTXbuffer[4] = 'A';
                gbytTXbuffer[5] = 'M';
                gbytTXbuffer[6] = 'E';
                gbytTXbuffer[7] = '=';
                gbytTXbuffer[8] = 'V';
                gbytTXbuffer[9] = '2';
                gbytTXbuffer[10] = 'P';
                gbytTXbuffer[11] = '2';
                gbytTXbuffer[12] = 0x0D;    // '\r'
                gbytTXbuffer[12] = 0x0A;    // '\n'
                gbytTXbuflen = 13;
                gSCIstatus.bTXRDY = 1;	// Initiate TX.
                OSSetTaskContext(ptrTask, 5, 200*__NUM_SYSTEMTICK_MSEC);     // Next to state = 5, timer = 0.2sec.
                break;

            case 5: // State 5 - Wait until all data is send to Bluetooth module.
                if (gSCIstatus.bTXRDY == 0)
                {
                    OSSetTaskContext(ptrTask, 6, 1000*__NUM_SYSTEMTICK_MSEC);     // Next to state = 6, timer = 1 sec.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 100);     // Next to state = 5, timer = 100.
                }
                break;

            case 6: // State 105 - Change Bluetooth module baud rate to 115200 bps.
                gSCIstatus.bRFTXRDY = 1;            // Set transmit flag.
                gbytTXbuffer[0] = 'A';              // AT command "AT+UART=115200,0,0"
                gbytTXbuffer[1] = 'T';
                gbytTXbuffer[2] = '+';
                gbytTXbuffer[3] = 'U';
                gbytTXbuffer[4] = 'A';
                gbytTXbuffer[5] = 'R';
                gbytTXbuffer[6] = 'T';
                gbytTXbuffer[7] = '=';
                gbytTXbuffer[8] = '1';
                gbytTXbuffer[9] = '1';
                gbytTXbuffer[10] = '5';
                gbytTXbuffer[11] = '2';
                gbytTXbuffer[12] = '0';
                gbytTXbuffer[13] = '0';
                gbytTXbuffer[14] = ',';
                gbytTXbuffer[15] = '0';
                gbytTXbuffer[16] = ',';
                gbytTXbuffer[17] = '0';
                gbytTXbuffer[18] = 0x0D;    // '\r'
                gbytTXbuffer[18] = 0x0A;    // '\n'
                gbytTXbuflen = 19;
                gSCIstatus.bTXRDY = 1;	// Initiate TX.
                OSSetTaskContext(ptrTask, 7, 200*__NUM_SYSTEMTICK_MSEC);        // Next to state = 7, timer = 0.2sec.
                break;

            case 7: // State 7 - Wait until all data is send.
                if (gSCIstatus.bTXRDY == 0)
                {
                    OSSetTaskContext(ptrTask, 8, 1000*__NUM_SYSTEMTICK_MSEC);   // Next to state = 8, timer = 1sec.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 7, 100);                          // Next to state = 7, timer = 100.
                }
                break;

             case 8: // State 8 - De-assert AT command mode and set new UART1 baud rate.
                PIN_HC_05_KEY = 0;                                              // Deassert KEY pin.
                U1BRG = (__FOSC_MHz/8)*(1000/_BT_UART_BAUDRATE_kBPS)-1;         // Update new baud rate for internal
                                                                                // UART module.
                OSSetTaskContext(ptrTask, 9, 100*__NUM_SYSTEMTICK_MSEC);        // Next to state = 9, timer = 0.1 sec.
                break;

            case 9: // State 9 - Idle.
                OSSetTaskContext(ptrTask, 10, 1);                               // Next to state = 10, timer = 1.
                break;

            case 10: // State 10 - Assert reset.
                PIN_HC_05_RESET = 0;                                            // Reset Bluetooth module.
                OSSetTaskContext(ptrTask, 11, 10*__NUM_SYSTEMTICK_MSEC);        // Next to state = 11, timer = 10 msec.
                break;
                
            case 11: // State 11 - De-assert reset, entering into normal communication mode.
                PIN_HC_05_RESET = 1;                                            // De-assert reset pin.
                OSSetTaskContext(ptrTask, 12, 200*__NUM_SYSTEMTICK_MSEC);       // Next to state = 12, timer = 200 msec.
                                                                                // Here we should have sufficient delay for
                                                                                // the HC-05 module to reboot properly 
                                                                                // before we start operation.
                break;
                
            case 12: // State 12 - Clear UART buffers and flags before we begin.
                gSCIstatus.bRXRDY = 0;                                          // Reset valid data flag.
                gbytRXbufptr = 0;                                               // Reset pointer.
                gSCIstatus.bRXOVF = 0;                                          // Reset overflow error flag.
                PIN_ILED2 = 0;                                                  // Off indicator LED2                
                OSSetTaskContext(ptrTask, 13, 1*__NUM_SYSTEMTICK_MSEC);         // Next to state = 13, timer = 1 msec.
                break;
                
            // --- Specifics sequence to establish SPP (serial port protocol) with remote host ---
            // Modify this on 27 Jan 2022, add a timeout counter, where if a packet of 5 bytes is not 
            // received within 10 msec, the receive buffer is flushed. This is to cater for certain HC-05
            // firmware which send out "OK\r\n" string when power up or establish an SPP link with
            // external controller.
            case 13: // State 12 - Wait for initial message from host computer to establish SPP link.
                     // Byte 0 = Robot RF Address.
                     // Byte 1 = 0x55 (Command)
                     // Byte 2 = _SPP_ESTABLISH_LINK
                     // Byte 3 = Remote host type:
                     //     0: PC or Tablet (devices with large display, at least 11 inches or larger).
                     //     Otherwise: Smartphone or similar devices (devices with small display).
                     // Byte 4: Not used, can set to 0.                  
                if (gSCIstatus.bRXRDY == 1)                             // Check if UART receives any valid data.
                {                                                       // data packet from remote host.
                    if (gbytRXbufptr > 4)                               // Make sure more than 4 bytes (1 packet = 5 bytes)
                    {    
                        if (gSCIstatus.bRXOVF == 0)                     // Make sure no overflow error.
                        {
                            if ((gbytRXbuffer[0] == gobjDriverHC05.bytRFAdd)&&(gbytRXbuffer[1] == 0x55) && (gbytRXbuffer[2] == _SPP_ESTABLISH_LINK))
                            {
                                nRFHostType = gbytRXbuffer[3];          // Get remote RF host type.
                                                                        // The remote RF host is PC or tablet computer
                                                                        // or smartphone (or similar devices with small or no display).                                    
                                OSSetTaskContext(ptrTask, 14, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 14, timer = 10 msec.
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 13, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 13, timer = 10 msec.
                            }
                        } 
                        else
                        {
                            gSCIstatus.bRXOVF = 0;                      // Reset overflow error flag.
                            OSSetTaskContext(ptrTask, 13, 10*__NUM_SYSTEMTICK_MSEC);         // Next state = 13, timer = 10 msec.
                        } // if (gSCIstatus.bRXOVF == 0)    
                        gSCIstatus.bRXRDY = 0;                          // Reset valid data flag.
                        gbytRXbufptr = 0;                               // Reset pointer.
                        PIN_ILED2 = 0;                                  // Off indicator LED2
                        nTimeoutCounter = 0;                            // Reset timeout counter.                        
                    }  
                    else
                    {
                        nTimeoutCounter++;                              // Increment timeout counter.
                        if (nTimeoutCounter > 10*__NUM_SYSTEMTICK_MSEC) // 10 msec timeout. After the 1st bytes is received, the rest of the bytes 
                                                                        // should arrive within 10 msec.
                        {
                            nTimeoutCounter = 0;                        // Reset timeout counter.  
                            gSCIstatus.bRXRDY = 0;                      // Reset valid data flag.
                            gbytRXbufptr = 0;                           // Reset pointer.
                            PIN_ILED2 = 0;                              // Off indicator LED2      
                        }
                        OSSetTaskContext(ptrTask, 13, 10*__NUM_SYSTEMTICK_MSEC);         // Next state = 13, timer = 10 msec.
                        
                    } // if (gbytRXbufptr > 4)
                }
                else
                {
                    OSSetTaskContext(ptrTask, 13, 10*__NUM_SYSTEMTICK_MSEC); // Next state = 13, timer = 10 msec.
                } // if (gSCIstatus.bRXRDY == 1) 
                break;

            case 14: // State 14 - Reply host computer, note that standard packet consist
                    // of 8 bytes.
                    // Byte 0 = Robot RF Address.
                    // Byte 1 = 0x55 (Command)
                    // Byte 2 = _SPP_LINK_ESTABLISHED
                    // Byte 3 = Data length for parameters set 1 (optional).
                    // Byte 4 = Data length for parameters set 2 (optional).
                    //  For both Byte 3 and Byte 4:
                    //  0 = 16 bits unsigned integer.
                    //  1 = 15 bits unsigned integer.
                    //  2 = 14 bits unsigned integer.
                    //  3 = 13 bits unsigned integer.
                    //  ...
                    //  7 = 8 bits unsigned integer.
                    // Bytes 5-7: Not used, can set to 0.
                gbytTXbuffer[0] = gobjDriverHC05.bytRFAdd;
                gbytTXbuffer[1] = 0x55;                 // Indicate command.
                gbytTXbuffer[2] = _SPP_LINK_ESTABLISHED;
                gbytTXbuffer[3] = 8;                    // Bit length of data set 1, 8-bits.
                gbytTXbuffer[4] = 3;                    // Bit length of data set 2, 13-bits.
                gbytTXbuffer[5] = 0;
                gbytTXbuffer[6] = 0;
                gbytTXbuffer[7] = 0;
                gbytTXbuflen = 8;
                gSCIstatus.bTXRDY = 1;	// Initiate TX.

                //gnAudioTone[0] = 5;
                //gnAudioTone[1] = 2;
                //gnAudioTone[2] = 4;
                //gnAudioTone[3] = 0;
                OSSetTaskContext(ptrTask, 20, 200*__NUM_SYSTEMTICK_MSEC); // Next state = 20, timer = 200msec.
                break;

            case 20: // State 20 - Prepare to start main user wireless routines.
                nCounter = 0;
                PIN_ILED2 = 0;                      // Off indicator LED2.
                                                    // LED2 is turned on to indicate
                                                    // wireless data link activities.
                if (nRFHostType == 0)               // Check if remote host is PC or tablet type computing 
                {                                   // devices.
                    gobjDriverHC05.nRFLinkState = 1;              // Indicate RF wireless link is established with PC,
                                                    // tablet or devices with high processing power and
                }                                   // large display.
                else
                {
                    gobjDriverHC05.nRFLinkState = 2;              // Indicate RF wireless link is established with smartphone
                                                    // or similar devices with small or no display at all.
                }
                OSSetTaskContext(ptrTask, 21, 1000);  // Next state = 21, timer = 1000.
                break;

            case 21: // State 21 - Main message clearing loop, check for data packet from remote host.
                if ((gSCIstatus.bRXRDY == 1) && (gbytRXbufptr > 4))     // Check if UART receive any data.
                {                                                       // 5 bytes per data packet from
                                                                        // remote host.
                    if (gSCIstatus.bRXOVF == 0) // Make sure no overflow error.
                    {
                        if (gbytRXbuffer[0] == gobjDriverHC05.bytRFAdd) // Make sure the device ID is for this unit.
                        {
                            if (gbytRXbuffer[1] == 0xAA)           // Check for ASCII data.
                            {   // Goto ASCII data handler.
                                gobjDriverHC05.bytRFCommand = gbytRXbuffer[2];
                                OSSetTaskContext(ptrTask, 35, 1);   // Next state = 35, timer = 1.
                            }
                            else if (gbytRXbuffer[1] == 0xAB)      // Check for Binary data.
                            {   // Goto Binary data handler.
                                OSSetTaskContext(ptrTask, 30, 1); // Next state = 30, timer = 1.
                            }
                            else if (gbytRXbuffer[1] == 0x55)      // Check for command and arguments
                            {   // Goto command handler.
                                gobjDriverHC05.bytRFCommand = gbytRXbuffer[2];
                                gobjDriverHC05.bytRFArgument1 = gbytRXbuffer[3];
                                gobjDriverHC05.bytRFArgument2 = gbytRXbuffer[4];
                                OSSetTaskContext(ptrTask, 25, 1);  // Next state = 25, timer = 1.
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 21, 1); // Next state = 21, timer = 1.
                            }
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 21, 1); // Next state = 21, timer = 1.
                        }
                    }
                    else
                    {
                        PIN_ILED2 = 0;              // Off indicator LED2.
                        gSCIstatus.bRXOVF = 0; 	// Reset overflow error flag.
                        OSSetTaskContext(ptrTask, 21, 1); // Next state = 21, timer = 1.
                    }
                    gSCIstatus.bRXRDY = 0;	// Reset valid data flag.
                    gbytRXbufptr = 0; 		// Reset receive buffer pointer.
                }
                else
                {
                    //PIN_ILED2 = 0;              // Off indicator LED2.
                    OSSetTaskContext(ptrTask, 50, 1); // Next state = 50, timer = 1.
                }
                break;

            case 25: // State 25 - Command handler.
                nTemp = HC05_CommHandler();                 // Call command handler routines.  The returned value
                                                            // is the next stage this process should go to.
                PIN_ILED2 = 0;                              // Off indicator LED2.
                                                            // LED2 is turned on to indicate
                                                            // wireless data link activities.
                OSSetTaskContext(ptrTask, nTemp, 1*__NUM_SYSTEMTICK_MSEC);       // Next state = 21, timer = 1 msec.
                break;

            case 30: // State 30 - Binary data handler.
                HC05_BinaryHandler();                       // Call binary data handler routines.
                PIN_ILED2 = 0;                              // Off indicator LED2.
                OSSetTaskContext(ptrTask, 21, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 21, timer = 1 msec.
                break;

            case 35: // State 35 - ASCII data handler,interprete ASCII commands
                     // from remote RF host.
                HC05_ASCIIHandler();                       // Call ASCII data handler routines.
                PIN_ILED2 = 0;                             // Off indicator LED2.
                OSSetTaskContext(ptrTask, 21, 1*__NUM_SYSTEMTICK_MSEC);         // Next to state = 21, timer = 1 msec.
                break;

            case 50: // State 50 - Periodically send back data to remote host.
                     // Data packet format:
                     // Byte 0: ID or RF address of this unit.
                     // Byte 1: Format = Binary or ASCII.
                     // Byte 2: 8-bit unsigned integer data 1.
                     // Byte 3: 8-bit unsigned integer data 2.
                     // Byte 4: Upper 8-bit of 16-bit unsigned integer data 3.
                     // Byte 5: Lower 8-bit of 16-bit unsigned integer data 3.
                     // Byte 6: Upper 8-bit of 16-bit unsigned integer data 4.
                     // Byte 7: Lower 8-bit of 16-bit unsigned integer data 4.
                     // Byte 8-63: Miscellaneous data.

                if (gobjDriverHC05.nEnRFTxPeriodicData > 0)      // Check if transmission of periodic data via RF link is enabled.
                {
                    nCounter++;
                    if (gSCIstatus.bTXRDY == 0)     // Check if all pending data is transmitted.
                    {
                        gobjDriverHC05.nDriverBusy = 0; // Indicate driver is not busy.
                    }
                }
                if (nCounter > 59)                  // Check timer for sending periodic data back to remote host.
                                                    // NOTE: 15 Jan 2018
                                                    // From actual measurement of the TX pin it takes roughly 5.5 msec to send
                                                    // out 63 bytes of data including the headers at 115.2 kbps, with 1 OS tick
                                                    // period of 0.16667 msec.  Thus after packing the data with a delay of
                                                    // 20 msec would be more than sufficient.
                                                    // Assuming no other commands is received
                                                    // (59+1)x2 = 120 ticks, or 120/6 = 20.0 msec (Assume 1 OS ticks = 0.1667 msec).
                                                    // In total the transmission of periodic data is every 20.0 + 20.0 = 40.0 msec
                                                    // or 25.0 samples every second.  
                {
                    nCounter = 0;
                    
                    if (gSCIstatus.bTXRDY == 0)                 // Check if UART module is busy.
                    {
                        gbytTXbuffer[0] = gobjDriverHC05.bytRFAdd;
                        if (gobjDriverHC05.nEnRFTxPeriodicData == 1)          // Send binary data
                        {
                            gbytTXbuffer[1] = 0xAB;             // Indicate binary data packet.
                        }
                        else                                    // 2 or larger, send ASCII data.
                        {
                            gbytTXbuffer[1] = 0xAA;             // Indicate ASCII data packet.
                        }
                        HC05_SendPeriodData();                  // Call user routine to transfer periodic data
                                                                // to be send back to remote host.
                        gbytTXbuflen = 64;                      // Packet length.
                        gSCIstatus.bTXRDY = 1;	// Initiate TX.
                        gobjDriverHC05.nDriverBusy = 1;         // Indicate driver is busy sending data.
                        OSSetTaskContext(ptrTask, 21, 20*__NUM_SYSTEMTICK_MSEC);    // Next to state = 21, timer = 20 msec.
                                                                                    // See notes above.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 21, 1);    // Next to state = 21, timer = 1.
                    }

                }

                else
                {
                    OSSetTaskContext(ptrTask, 21, 1);    // Next to state = 21, timer = 1.
                }
                break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1);    // Next to state = 0, timer = 1.
                break;
        }
    }
}

// ptrText = Address of character string.
// nTextLength = Length of text string exclusive of the newline character (0xD).
// Usage example:
// SendTexttoHost("Hello",5);
//
// or
// char *ptrChar;
//
// ptrChar = "Hello";
// SendTexttoHost(ptrChar,5);

int SendTexttoRemoteHost(char *ptrText, int nTextLength)
{
    int nIndex;

    if (gobjDriverHC05.nRFLinkState > 0)  // Check if RF link is established.
    {
        if (gSCIstatus.bTXRDY == 0) // Make sure RF link is not busy.
        {
            if (nTextLength > 51)
            {
                nTextLength = 51;           // Limit text length to 61 characters per line.
            }
            gbytTXbuffer[0] = gobjDriverHC05.bytRFAdd;    // Set destination address.
            gbytTXbuffer[1] = 0xAA;         // Indicate ASCII data.
            for (nIndex = 0; nIndex < nTextLength; nIndex++ )
            {
                gbytTXbuffer[nIndex + 2] = *(ptrText + nIndex);
            }
            gbytTXbuffer[nIndex + 2] = 0xD; // Append the newline character.
            gbytTXbuflen = 64;
            gSCIstatus.bTXRDY = 1;          // Initiate TX.
            return 1;                       // Success.
        }
        else
        {
            return  0;
        }
    }
    else
    {
        return 0;
    }
}