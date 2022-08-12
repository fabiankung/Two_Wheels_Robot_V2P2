////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	MICROCONTROLLER INDEPENDENT APPLICATION PROGRAM INTERFACE ROUTINES
//
//  (c) Copyright 2013-2020, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename         : os_APIs.c
// Author           : Fabian Kung
// Last updated     : 24 Aug 2020
// File Version     : 1.11
// Description      : This file contains the implementation of all the important routines
//                    used by the Kernel for task management. It include routines to create or
//                    initialize a task, delete a task from the Scheduler, setting a task  
//                    context etc.  The routines are general and can be used for most 
//                    microcontroller families.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "osmain.h"

// --- GLOBAL VARIABLES AND DATAYPES DECLARATION ---
volatile INT16 gnRunTask;                       // Flag to determine when to run tasks.  Need to declare as volatile 
                                                // so that the code will run properly with compiler optimization. This
                                                // indicates that the variable can be modify by other threads outside 
                                                // of the visible codes to the compiler.
INT16 gnTaskCount;                              // Task counter.
volatile UINT32 gulClockTick;                   // Processor clock tick. See comments for gnRunTask.
TASK_ATTRIBUTE gstrcTaskContext[__MAXTASK-1];   // Array to store task contexts.
TASK_POINTER gfptrTask[__MAXTASK-1];            // Array to store task pointers.
SCI_STATUS gSCIstatus;				// Status for UART and RF serial communication interface.
UINT16 gunTaskExecutedCount;        // This counter keep track of how many tasks are called in one
                                    // OS clock tick.  It it automatically incremented in the 
                                    // Scheduler when a task is called, and decremented when a 
                                    // task exits by calling the OS function OSSetTaskContext().
UINT16 gunOSError;                  // OS error status.  
                                    // bit 0 - State error.  Whenever an invalid state is called
                                    // in the task this flag should be set (this is done by 
                                    // setting this bit in the default case of the state 
                                    // dispatcher within the task).
                                    // bit 1 - Task context error.  This bit is set whenever 
                                    // the OS detects that OSSetTaskContext() is not called 
                                    // when a task exits. 

// --- RTOS FUNCTIONS ---

// Function name	: OSInit()
// Author           : Fabian Kung
// Last modified	: 8 March 2019
// Purpose          : Initialize the variables and parameters of the RTOS.
// Arguments		: None.
// Return           : None.
void OSInit()
{
	gulClockTick = 0; 		// Initialize 32-bits RTOS global timer.
	gSCIstatus.bTXRDY = 0;		// Initialize SCI status flags.
	gSCIstatus.bRXRDY = 0;
	gSCIstatus.bRXOVF = 0;
	gSCIstatus.bRFTXRDY = 0;
	gSCIstatus.bRFRXRDY = 0;
    gunOSError = 0;
    gunTaskExecutedCount = 0;
}

// Function name	: OSTaskCreate()
// Author           : Fabian Kung
// Last modified	: 2 Nov 2009
// Purpose          : Add a new task to the OS's scheduler.
// Arguments		: ptrTaskData = A pointer to the structure structTASK.
//                    ptrTask = a valid pointer to a user routine.
// Return           : 0 if success, 1 or >0 if not successful.
// Others           : Increment global variable gnTaskCount.
BYTE OSCreateTask(TASK_ATTRIBUTE *ptrTaskData, TASK_POINTER ptrTask)
{
	if (gnTaskCount> __MAXTASK) 
	{
		return 1;                           // Maximum tasks exceeded.
	}
	else
	{
		ptrTaskData->nState = 0;            // Initialize the task state and timer variables.
		ptrTaskData->nTimer = 1;
											
		gfptrTask[gnTaskCount] = ptrTask;	// Assign task address to function pointer array.
		gnTaskCount++;                      // Increment task counter.
                                            // Initialize the task ID
		ptrTaskData->nID = gnTaskCount; 	// Task ID = current Task Count + 1.

		return 0;
	}
}

// Function name	: OSSetTaskContext()
// Author           : Fabian Kung
// Last modified	: 9 March 2019
// Purpose          : Set the task's State and Timer variables.
// Arguments		: ptrTaskData = A pointer to the structure structTASK.
//                  nState = Next state of the task.
//                  nTimer = Timer, the no. of clock ticks before the task
//                  executes again.
// Return           : None.
void OSSetTaskContext(TASK_ATTRIBUTE *ptrTaskData, INT16 nState, INT16 nTimer)
{
	ptrTaskData->nState = nState;
	ptrTaskData->nTimer = nTimer;
    gunTaskExecutedCount--;
}

// Function name	: OSTaskDelete()
// Author           : Fabian Kung
// Last modified	: 2 Nov 2009
// Description		: Delete a task from the OS's scheduler.
// Arguments		: nTaskID = An integer indicating the task ID.
// Return           : 0 if success, 1 or >0 if not successful.
BYTE OSTaskDelete(INT16 nTaskID)
{
	INT16 ni;

	if (nTaskID < gnTaskCount) // nTaskID can be from 1 to (gnTaskCount-1)
	{
		if (nTaskID < (gnTaskCount - 1))	// Shift all the elements of the gstrcTaskContext
		{					// array down 1 position.
			ni = nTaskID - 1;
			while (ni < (gnTaskCount - 1))
			{
				gstrcTaskContext[ni].nState = gstrcTaskContext[ni + 1].nState;
				gstrcTaskContext[ni].nTimer = gstrcTaskContext[ni + 1].nTimer;
				gstrcTaskContext[ni].nID = gstrcTaskContext[ni + 1].nID;
				gfptrTask[ni] = gfptrTask[ni + 1];
				ni++;
			}
			gnTaskCount--; 			// There is 1 less task to execute now.
		}
		return 0;
	}
	else
	{
		return 1;
	}
}

// Function name	: OSUpdateTaskTimer()
// Author           : Fabian Kung
// Last modified	: 2 Nov 2009
// Description		: Update the timer attribute of each task.
// Arguments		: None.
// Return           : None.
void OSUpdateTaskTimer(void)
{
	INT16 ni = 0;

	ni = 0; 					// Reset index.
	while (ni < gnTaskCount)
	{	
		--(gstrcTaskContext[ni].nTimer); 	// Decrement the timer of each task.
		ni++; 					// Next task.
	}
}

