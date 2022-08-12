// Author			: Fabian Kung
// Date				: 23 March 2015
// Filename			: Driver_UART1_V100.h

#ifndef _DRIVER_UART1_dsPIC33_H
#define _DRIVER_UART1_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//
				
// Data buffer and address pointers for wired serial communications.
extern BYTE gbytTXbuffer[__SCI_TXBUF_LENGTH-1];
extern BYTE gbytTXbufptr;
extern BYTE gbytTXbuflen;
extern BYTE gbytRXbuffer[__SCI_RXBUF_LENGTH-1];
extern BYTE gbytRXbufptr;


//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_UART1_Driver(TASK_ATTRIBUTE *);

#endif
