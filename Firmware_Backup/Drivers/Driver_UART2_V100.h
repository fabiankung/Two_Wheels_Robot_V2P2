// Author			: Fabian Kung
// Date				: 7 May 2015
// Filename			: Driver_UART2_V100.h

#ifndef _DRIVER_UART2_dsPIC33_H
#define _DRIVER_UART2_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//


#define     __SCI_TXBUF_LENGTH2     15
#define     __SCI_RXBUF_LENGTH2     15

// Data buffer and address pointers for wired serial communications.
extern BYTE gbytTXbuffer2[__SCI_TXBUF_LENGTH2-1];
extern BYTE gbytTXbufptr2;
extern BYTE gbytTXbuflen2;
extern BYTE gbytRXbuffer2[__SCI_RXBUF_LENGTH2-1];
extern BYTE gbytRXbufptr2;

extern SCI_STATUS gSCIstatus2;
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_UART2_Driver(TASK_ATTRIBUTE *);

#endif
