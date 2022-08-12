///////////////////////////////////////////////////////////////////////////////////////////////////
//
//	BASIC REAL-TIME OPERATING SYSTEM (RTOS) FOR 8/16-BITS MICROCONTROLLER
//
//
//  (c) Copyright 2013-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename         : osmain_V1_00.c
// Author(s)		: Fabian Kung
// Last modified	: 13 March 2019
// Release Version	: 1.06 (This refers to OS version or release)
// Description		: Main C file for the RTOS.
//                    Uses round-robin scheduling algorithm to schedule tasks.
// Toolsuites		: Microchip MPLAB X IDE v5.10 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above
//                    Microchip Pickit3 or ICD3
// Microcontroller	: dsPIC33EP256MU806 16-bits, 64 pins, 28kbyte RAM, 256kbyte Flash ROM, 70 MIPS.
// Oscillator/Clock	: 4 MHz crystal (XT mode) with Phase-Locked Loop multiplier to 140 MHz (70 MIPS)
//					  
// Main Features:
// 1) Up to 70 MIPS.
// 2) Multi-tasking with Round-robin equal priority scheduler.
// 3) Hardware watchdog timer.
// 4) Systick timer.
// 5) Ability to assign a timer for all processes, resolution of 1 Systick.
// 6) 3x Wired serial communication (UART) interface stacks.
// 7) 1x I2C serial communication interface stack.
// 8) Wireless RF communication stack using IEEE 802.15.4 or Bluetooth.
// 9) Include a new Resource Usage Summary table, giving an overview of controller and external circuits
//    resources usage, pin usage etc.
// 10) Better task error tracking.  This version includes two error tracking features, in which
//     the processor will hang whenever one of the following errors is encountered:
//     a) When OSSetTaskContext() is not called before the task pass control back to the 
//     Scheduler.
//     b) In the case of task using state-machine architecture, the default state is called.
//     This indicates a wrong state is assigned in the codes, hence the state-machine look-up
//     mechanism cannot find a correct state, and assign the current execution to the default
//     state.
//
// Minimum Directory and file structures:
// C library routines for various drivers/peripherals:
// 1. <path for library>\C_Library\dsPIC33E\
// Files needed: "osmain.h", "os_API.c" and "os_dsPIC33E_API.c"
// 2. <Project folder>\os "osmain.c" codes and user files.
//

///////////////////////////////////////////////////////////////////////////////
// --- INCLUDE ALL DRIVER AND USER HEADER FILES --- ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Include common header to all drivers and sources.  Here absolute path is used for library source.
// To edit if one change folder.

// Main header file.  All source codes should include this.
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\osmain.h"
// Header file for each driver used.

#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_ADC_DAC_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_UART1_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_PWMServo_V101.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_I2C_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Bluetooth_HC_05\Driver_HC_05_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_UART2_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_QuadEncoder_V100.h"
// Header file user routines.
#include ".\Robot_Tasks_dsPIC33E.h"
///////////////////////////////////////////////////////////////////////////////

// --- MAIN LOOP ---

INT16 main(void)
{
	INT16 ni = 0;

	OSEnterCritical();			// Disable and store all processor's interrupt setting.
	OSEnterCritical();			// Do this twice to prevent interrupt happen midway, and the 
                                // processor interrupt is re-enabled upon return from the 
                                // interrupt service routine.
	dsPIC33E_PORInit(140);		// Power-on Reset initialization for the controller, set processor
                                // clock speed to 140 MHz.
	OSInit();                   // Initialize the RTOS.
	gnTaskCount = 0; 			// Initialize task counter.
//
//								// Initialize core OS processes.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], OSProce1);  // Start blinking LED.
								// User and other non-core OS tasks are initialized 
								// here.
//	OSExitCritical();			// Enable all processor interrupts (optional, some processor 
								// interrupts are turned on within PIC24H_PORInit or dsPIC33_PORInit
								// routines, thus it is not necessary for now).

///////////////////////////////////////////////////////////////////////////////
// --- CREATE AN INSTANCE OF DRIVER AND USER TASKS HERE --- ///////////////////
///////////////////////////////////////////////////////////////////////////////

        //OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_Claw_Driver);            // 15, for use with V2T2.  If V2T1 this should be disabled.
        
        // NOTE: DO NOT EXCEEED THE MAXIMUM LIMIT SET BY  __MAXTASK in file "osmain.h".
	// Kernel Driver tasks:
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_UART1_Driver);           // 1
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_UART2_Driver);           // 2    
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_ADC_Driver);             // 3
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_12DAC_MCP4822_Driver);   // 4
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Audio_Driver);           // 5
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_BT_HC_05_CommStack);     // 6 , HC-05 Bluetooth module driver/communication stack.
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_PWMServo_Driver);        // 7
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_I2C_Driver);             // 8
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_QuadEncoder_Driver);     // 9
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_ExControllerMessageLoop);     // 10
	// Robot/User tasks:
        OSCreateTask(&gstrcTaskContext[gnTaskCount], User_ProceTest2);               // 11, Experimental, to deactivate when not used.
        OSCreateTask(&gstrcTaskContext[gnTaskCount], User_Proce1);                  // 12
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_Sensor_MPU6050_CF);      // 13
        //OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_Sensor_MPU6050_KF);      // 13
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_Balance);                // 14
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_MoveTurn);               // 15
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_MoveLinear);             // 16
        OSCreateTask(&gstrcTaskContext[gnTaskCount], Robot_Proce1);                 // 17
///////////////////////////////////////////////////////////////////////////////

    while (1)				// Infinite loop.
    {
        ClearWatchDog();		// Clear the Watch Dog Timer.
        if (gnRunTask > 0) 		// Only execute tasks/processes when gnRunTask is not 0.
        {                   
            //_RB1 = 1;                               // Trigger debug flag.
            for (ni = 0; ni < gnTaskCount; ni++)    // Inspect the task attribute of each process.
            {
                if (gstrcTaskContext[ni].nTimer > 0) // Only decrement timer if it is greater than zero.
                {
                    --(gstrcTaskContext[ni].nTimer); // Decrement timer for each process.
                }
                // Only execute a process/task if it's timer = 0.
                if (gstrcTaskContext[ni].nTimer == 0)
                {	
                    gunTaskExecutedCount++;
                    // Execute user task by dereferencing the function pointer.
                    (*((TASK_POINTER) gfptrTask[ni]))(&gstrcTaskContext[ni]);
                }
            }
            gnRunTask = 0; 		// Reset gnRunTask.        
            //_RB1 = 0;                               //
        }
    }
    return 0;
}

