// Author			: Fabian Kung
// Date				: 22 Aug 2018
// Filename			: Driver_HC_05_V100.h

#ifndef _DRIVER_HC_05_dsPIC33_H
#define _DRIVER_HC_05_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//

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
    int nDriverBusy;                // 1 or larger = True.  True when there is still pending data to be send.
                                    // 0 or negative = idle.
} DSPIC33E_HC05_DRIVER;



extern  DSPIC33E_HC05_DRIVER    gobjDriverHC05;

//extern unsigned char gbytRFCommand;	// Command byte received from Wireless Master Device.
//extern unsigned char gbytRFArgument1;   // Argument for the command byte.
//extern unsigned char gbytRFArgument2;   // Argument for the command byte.
//extern int gnRFLinkState;           // Status to indicate RF link.
                                    // 1 = RF link established with remote PC host or Tablet or similar computers.  These
                                    // devices usually have large display and ability to display graphs.  With these 
                                    // devices we can transmit robot's status and sensors output periodically and have
                                    // the data display as graphs on the remote host.  When gnRFLinkState = 1, the other
                                    // user processes can set gnEnRFTxPeriodicData = 1 to enable periodic data transmission
                                    // from robot to remote host.  Else it is not advisable to set this flag.
                                    // 2 = RF link established with remote host such as smartphone or similar devices
                                    // with small display screen.  
                                    // Otherwise = No RF link.
//extern unsigned char gbytRFAdd;     // ID for the device connected to this Bluetooth module.
//extern int gnEnRFTxPeriodicData;    // 0 or else = Don't transmit data periodically.
                                    // 1 = Transmit data periodically, binary format.
                                    // 2 = Transmit data periodically, ASCII format.
extern  int gnEnHC05Init;           // 0 = Don't initialize the attached HC-05 module on power up.
                                    // 1 = Initialize the device name and baud rate of attached
                                    //     HC-05 module on power up.
                                    //     Note: Initialization is usually needed
                                    //     when one uses the HC-05 module for the first time,
                                    //     unless one uses the default name and baud rate.
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_BT_HC_05_CommStack(TASK_ATTRIBUTE *);
int HC05_CommHandler(void);
void HC05_ASCIIHandler(void);
void HC05_BinaryHandler(void);
void HC05_SendPeriodData(void);
int  SendTexttoRemoteHost(char *, int);
#endif
