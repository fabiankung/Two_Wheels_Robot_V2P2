// Author			: Fabian Kung
// Date				: 27 May 2015
// Filename			: Driver_UART3_V100.h

#ifndef _DRIVER_UART3_dsPIC33_H
#define _DRIVER_UART3_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//


#define     __SCI_TXBUF_LENGTH3     15
#define     __SCI_RXBUF_LENGTH3     15

// Data buffer and address pointers for wired serial communications.
extern BYTE gbytTXbuffer3[__SCI_TXBUF_LENGTH3-1];
extern BYTE gbytTXbufptr3;
extern BYTE gbytTXbuflen3;
extern BYTE gbytRXbuffer3[__SCI_RXBUF_LENGTH3-1];
extern BYTE gbytRXbufptr3;

extern SCI_STATUS gSCIstatus3;
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_UART3_Driver(TASK_ATTRIBUTE *);

#endif
