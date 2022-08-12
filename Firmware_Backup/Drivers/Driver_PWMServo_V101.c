//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2013-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Driver_PWMServo_V101.c
// Author(s)		: Fabian Kung
// Last modified	: 9 March 2019
// Tool suites		: Microchip MPLAB X IDE v4.05 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above


// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//
typedef struct StructProcePWMDriver
{
    UINT16   NumChannel;            // No. of outputs, valid values are 4, 6 and 8.  Any
                                    // other values not of these will be assume as 8 channel
                                    // of outputs.
    UINT16   unEnPWM;               // Set greater than 0 to enable the PWM module.
} DSPIC33E_PWM_DRIVER;

DSPIC33E_PWM_DRIVER    gobjDriverPWM;

// RC servo motor driver control variables
UINT16 gunRCServoPW_us[8];                  // RC servo motor pulse width in usec.
UINT16 gunPWStep_us[8];                     // RC servo motor pulse width increment/decrement
                                            // step in usec.
UINT16 gnCurrentMotorIDA = 0;               // Current ID of RC servo motor being triggered.
UINT16 gunRCServoStat[8] = {0,0,0,0,0,0,0,0}; // Status of RC servo motor.
                                            // 0 - Motor reached position, i.e. the pulse width is similar to the
                                            // required value, or the motor is not moving.
                                            // Otherwise - Motor is still moving towards target angle.
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//


//
// --- PRIVATE VARIABLES ---
//


//
// --- Microcontroller Pin Usage ---
//
// RD0 = Servo motor 0                                         
// RD1 = Servo motor 1
// RD2 = Servo motor 2
// RD3 = Servo motor 3
// RD4 = Servo motor 4
// RD5 = Servo motor 5
// RF4 = Servo motor 6
// RF5 = Servo motor 7
// 

//
// --- Process Level Constants Definition --- 
//
#define         _MAX_PULSEWIDTH_US	2300                        // Max pulse width in microseconds.
#define         _MIN_PULSEWIDTH_US	620                         // Min pulse width in microseconds.

#define         _PWM_SERVO_MOT_CONST              __FCYC_MHz    // This is FOSC_MHz/2, which is 60 for 120 MHz clock

                                                                // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                                // or 1093, and so on.
#if             __FCYC_MHz == 60    
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     1093
#elif           __FCYC_MHz == 62
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     1057
#elif           __FCYC_MHz == 64
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     1024
#elif           __FCYC_MHz == 66
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     993
#elif           __FCYC_MHz == 68
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     964
#elif           __FCYC_MHz == 70
    #define         _PWM_SERVO_SINGLE_COUNTER_LIMIT     936
#else
    #error "Proce__PWMServo_Driver: Processor frequency not supported"
#endif


#define         _INITIAL_PW1_US         1300           // A neutral position for the motor/actuator triggered.
#define         _INITIAL_PW2_US         1300
#define         _INITIAL_PW3_US         1300
#define         _INITIAL_PW4_US         1300
#define         _INITIAL_PW5_US         1300
#define         _INITIAL_PW6_US         1300
#define         _INITIAL_PW7_US         1300
#define         _INITIAL_PW8_US         1300

#define         _DEFAULT_PW_STEP_US     3

///
/// Process name		: Proce__PWMServo_Driver
///
/// Author              : Fabian Kung
///
/// Last modified		: 9 March 2019
///
/// Code Version		: 1.02
///
/// Processor			: 
///                       dsPIC33E256MU80X family.
///                       dsPIC33E512MC80X family.
///
/// Processor/System Resources 
/// PINS                : See above.
///
/// MODULES             : 1. OC1 - output compare 1 (Internal).
///                       2. OC2 - output compare 2 (Internal).                               
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables    : gunRCServoPW - User set RC servo control pulse width.
///                       nRCServoState - RC servo driver state.

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce__PWMServo_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce__PWMServo_Driver: An RTOS is required with this function"
#endif

///
/// Description : Driver for RC type servo motor or similar actuator using PWM pulses.
///               1) This routine support up to 8 (EIGHT) simultaneous PWM outputs, refer to the
///                  micro-controller pin usage for the relevant pins.
///               2) With this version the driver also supports multiple processor core frequency, 
///                  from 60 MHz to 70 MHz in step of 2 MHz.  Also variable output channels is
///                  now supported.  User can select between 4, 6, and 8 output channels.  For 
///                  instance if 6 output channels is selected, then the other two output pins
///                  of the processor can be used by other routines.  This improves flexibility
///                  in allocating the processor hardware resources.
///               3) The routine uses OC1 module of the dsPIC33E to produce one pulse every 
///				     time it is called.  This process will trigger each output with a pulse
///                  in a round-robin fashion.  
///               4) Two global parameters control the property of PWM outputs:
///                     gunRCServoPW_us[n]
///                     gunPWStep_us[n]
///
///                 Where n is the index of the PWM output from 0 to 7.  For 6 channel version only
///                 0 to 5 are used.
///               5) The width of the control pulse to the servo motor is determined by the global
///               variable gunRCServoPW_us[], with the unit in microseconds.
///               The second parameter gunPWStep_us[] determines how fast the servo motor
///               turns to its destination, typically value from 3 to 40 microseconds.
///
///               For instance for MOTOR1 the current pulse width is 1000 usec.  Say we set
///               gunRCServoPW_us[1] = 1600 usec, and gunPWStep_us[1] = 30 usec.
///               In this case whenever a control pulse is generated for MOTOR1, the pulse width will
///               be incremented from 1000 usec to 16000 usec in 30 usec step.  When the difference
///               between the set value and current pulse width is < gunPWStep_us[1], a default
///               step of 2 usec (declared as unPWSmallStep in the routine) will be used.  Once
///               the current pulse width is within +- 1 usec of the set value, the pulse width
///               will be fixed.  In this way by using smaller gunPWStep_us[1] we can slow down
///               the angular rotation of the motor (in moving from one angle to another)
///               and using larger gunPWStep_us[1] to speed up the transition.
///               6) Finally we now introduce a status register for each motor, in gunRCServoStat[].  
///               For now only bit0 is used.  If this is set, it means the motor is transitioning 
///               to it's set position.  If it is 0, it indicates the motor has reached it's 
///               target shaft position.
///
/// Usage example:
///               To use this driver:
///               1. Set gnPWMServoNoChannel to either 4, 6 or 8.  Default is 8.
///               2. Start the driver by setting the flag unEnPWM to 1.  After this is done 
///                  the corresponding processor pins will be set to output and other routines should
///                  not use the assigned pins.
///               3. Set gnRCServoPW_us[] to between _MIN_PULSEWIDTH_US and _MIN_PULSEWIDTH_US.
///               4. Setting gnRCServoPW to 0 will stop the module (e.g. no output).

#define __SERVO_DELAY       14      // Duration for each pulse in terms of no. of timer ticks.  Here assuming
                                    // each tick at 166.67 usec, then 14 x 166.67 sec = 2333.33 usec > _MAX_PULSEWIDTH_US

void Proce_PWMServo_Driver(TASK_ATTRIBUTE *ptrTask)
{
    INT16 nTemp;
    INT16 nIndex;
    unsigned long ulMotor;
    static int    nNoChannel = 8;
    static UINT16 unCurrentPW[8];                       // Current pulse width (PW).
    static UINT16 unMotorPW;
    static UINT16 unPWStep = 10;                        // Normal increment step for pulse width setting.
    static UINT16 unPWSmallStep = 1;                    // Small increment step for pulse width setting.

    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Power up.  Wait until unEnPWM is set to 1 or greater.
                if (gobjDriverPWM.unEnPWM == 0)
                {
                    OSSetTaskContext(ptrTask, 0, 1);    // Next state = 0, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 10 msec.
                }
                break;
                
            case 1: // State 1 - Initialization of OC1 and OC2 modules, uC pins and motor variables.
                // Setup IO pins mode.
                if (gobjDriverPWM.NumChannel == 4)           // 4 output channels.
                {
                    nNoChannel = 4;
                    TRISDbits.TRISD0 = 0;               // Set RD0 as output.
                    TRISDbits.TRISD1 = 0;               // Set RD1 as output.
                    TRISDbits.TRISD2 = 0;               // Set RD2 as output.
                    TRISDbits.TRISD3 = 0;               // Set RD3 as output.                    
                }
                else if (gobjDriverPWM.NumChannel == 6)      // 6 output channels.
                {
                    nNoChannel = 6;
                    TRISDbits.TRISD0 = 0;               // Set RD0 as output.
                    TRISDbits.TRISD1 = 0;               // Set RD1 as output.
                    TRISDbits.TRISD2 = 0;               // Set RD2 as output.
                    TRISDbits.TRISD3 = 0;               // Set RD3 as output.
                    TRISDbits.TRISD4 = 0;               // Set RD4 as output.
                    TRISDbits.TRISD5 = 0;               // Set RD5 as output.                    
                }
                else                                    // 8 output channels.
                {
                    nNoChannel = 8;
                    TRISDbits.TRISD0 = 0;               // Set RD0 as output.
                    TRISDbits.TRISD1 = 0;               // Set RD1 as output.
                    TRISDbits.TRISD2 = 0;               // Set RD2 as output.
                    TRISDbits.TRISD3 = 0;               // Set RD3 as output.
                    TRISDbits.TRISD4 = 0;               // Set RD4 as output.
                    TRISDbits.TRISD5 = 0;               // Set RD5 as output.
                    TRISFbits.TRISF4 = 0;               // Set RF4 as output.
                    TRISFbits.TRISF5 = 0;               // Set RF5 as output.                   
                }

                OC1CON1 = 0;                            // According to the datasheet it is a
                OC1CON2 = 0;                            // good habit to set all OC control bits to 0 first.
                OC2CON1 = 0;
                OC2CON2 = 0;
                         
                gnCurrentMotorIDA = 0;

                for (nIndex = 0; nIndex < 8; nIndex++)
                {
                    gunRCServoPW_us[nIndex] = 0;        // Initialize output duty cycle to 0, eg. to turn off motor.
                    gunPWStep_us[nIndex] = _DEFAULT_PW_STEP_US;  // Initialize duty cycle increment/decrement step.
                }
                                                        // NOTE: 5th March 2013 F. Kung
                                                        // We need to set the initial pulse width to some
                                                        // neutral position, as upon
                                                        // activation this driver will send out the default
                                                        // pulse width to the first motor with no user input.
                                                        // If the pulse width is not correct, it might cause
                                                        // the motor or actuator to hit a barrier and stalled,
                                                        // causing damage to the motor/actuator.
                unCurrentPW[0] = _INITIAL_PW1_US;       // Initialize current duty cycle.
                unCurrentPW[1] = _INITIAL_PW2_US;       // Initialize current duty cycle.
                unCurrentPW[2] = _INITIAL_PW3_US;       // Initialize current duty cycle.
                unCurrentPW[3] = _INITIAL_PW4_US;       // Initialize current duty cycle.
                unCurrentPW[4] = _INITIAL_PW5_US;       // Initialize current duty cycle.
                unCurrentPW[5] = _INITIAL_PW6_US;       // Initialize current duty cycle.
                unCurrentPW[6] = _INITIAL_PW7_US;       // Initialize current duty cycle.
                unCurrentPW[7] = _INITIAL_PW8_US;       // Initialize current duty cycle.
                OSSetTaskContext(ptrTask, 2, 1000*__NUM_SYSTEMTICK_MSEC);     // Next state = 2, timer = 1000 msec.
            break;

            case 2: // State 2 - Gradually increase or decrease the servo motor angle by adjusting the duty cycle.
                    // Without this the RC servo motor might move too fast from one position/angle to another
                    // position/angle, resulting in overshoot.  With modern digital servo, which uses PID
                    // feedback control, this step can be eliminated.
                
                OC1CON1 = 0;                                        // According to the datasheet it is a
                OC1CON2 = 0;                                        // good habit to set all control bits to 0 first.
                OC2CON1 = 0;
                OC2CON2 = 0;
                                                                    // Disconnect OC1 and OC2 from all output pins.
                if (gobjDriverPWM.NumChannel == 4)                  // 4 output channels.
                {
                    RPOR0bits.RP64R = 0b000000;                     // Disconnect RP64 (RD0) from OC peripheral.
                    RPOR0bits.RP65R = 0b000000;                     // Disconnect RP65 (RD1) from OC peripheral.
                    RPOR1bits.RP66R = 0b000000;                     // Disconnect RP66 (RD2) from OC peripheral.
                    RPOR1bits.RP67R = 0b000000;                     // Disconnect RP67 (RD3) from OC peripheral.                    
                }
                else if (gobjDriverPWM.NumChannel == 6)
                {
                    RPOR0bits.RP64R = 0b000000;                     // Disconnect RP64 (RD0) from OC peripheral.
                    RPOR0bits.RP65R = 0b000000;                     // Disconnect RP65 (RD1) from OC peripheral.
                    RPOR1bits.RP66R = 0b000000;                     // Disconnect RP66 (RD2) from OC peripheral.
                    RPOR1bits.RP67R = 0b000000;                     // Disconnect RP67 (RD3) from OC peripheral.
                    RPOR2bits.RP68R = 0b000000;                     // Disconnect RP68 (RD4) from OC peripheral.
                    RPOR2bits.RP69R = 0b000000;                     // Disconnect RP67 (RD5) from OC peripheral.                           
                }
                else
                {
                    RPOR0bits.RP64R = 0b000000;                     // Disconnect RP64 (RD0) from OC peripheral.
                    RPOR0bits.RP65R = 0b000000;                     // Disconnect RP65 (RD1) from OC peripheral.
                    RPOR1bits.RP66R = 0b000000;                     // Disconnect RP66 (RD2) from OC peripheral.
                    RPOR1bits.RP67R = 0b000000;                     // Disconnect RP67 (RD3) from OC peripheral.
                    RPOR2bits.RP68R = 0b000000;                     // Disconnect RP68 (RD4) from OC peripheral.
                    RPOR2bits.RP69R = 0b000000;                     // Disconnect RP67 (RD5) from OC peripheral.         
                    RPOR9bits.RP100R = 0b000000;                    // Disconnect RP100 (RF4) from OC peripheral.
                    RPOR9bits.RP101R = 0b000000;                    // Disconnect RP101 (RF5) from OC peripheral.                    
                }

                unPWStep = gunPWStep_us[gnCurrentMotorIDA];         // Get step change for pulse width.
                
                if (gunRCServoPW_us[gnCurrentMotorIDA] > 0)         // Only generate an output pulse if gunRCServoPW_us is > 0.
                {                                                   // Increment or decrement the pulse width setting by
                                                                    // comparing current pulse width setting with user setting.
                    nTemp = gunRCServoPW_us[gnCurrentMotorIDA] - unCurrentPW[gnCurrentMotorIDA];
                                                                    // Get the difference between current and user set pulse
                                                                    // width.
                    if (nTemp > 0)                                  // User set pulse width is larger than current pulse width.
                    {                                               
                                                                    // Check the difference between current setting and user setting.
                        if (nTemp > unPWStep)                       // If it is larger than the increment step unPWStep, increase
                        {                                           // the current setting by unPWStep.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] + unPWStep;
                            gunRCServoStat[gnCurrentMotorIDA] = 1;  // Indicate RC servo motor is still turning.
                        }
                        else
                        {                                           // Else increase the current setting by a small step, this is
                                                                    // to minimize the error between user setting and current setting.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] +  unPWSmallStep;
                            gunRCServoStat[gnCurrentMotorIDA] = 1;  // Indicate RC servo motor is still turning.
                        }
                    }
                    else if (nTemp < 0)                             // User set pulse width is smaller than current pulse width.
                    {
                        nTemp = -nTemp;                             // Check the difference between current setting and user setting.
                        if (nTemp > unPWStep)                       // If it is smaller than the increment step unPWStep, decrease
                        {                                           // the current setting by unPWStep.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] - unPWStep;
                            gunRCServoStat[gnCurrentMotorIDA] = 1;  // Indicate RC servo motor is still turning.
                        }
                        else
                        {                                           // Else decrease the current setting by a small step, this is
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] -  unPWSmallStep;
                            gunRCServoStat[gnCurrentMotorIDA] = 1;  // Indicate RC servo motor is still turning.
                        }             
                    }
                    else    // nTemp = 0
                    {
                        gunRCServoStat[gnCurrentMotorIDA] = 0;      // Indicate RC servo motor has reached target shaft angle.
                    }
                    
                    // Check for over and under limits (of the pulse width).
                    if (unCurrentPW[gnCurrentMotorIDA] > _MAX_PULSEWIDTH_US)       // Make sure the maximum PWM pulsewidth do not
                    {                                           // exceed the MAX_PULSEWIDTH_US limit.
                        unCurrentPW[gnCurrentMotorIDA] = _MAX_PULSEWIDTH_US;
                    }
                    if (unCurrentPW[gnCurrentMotorIDA] < _MIN_PULSEWIDTH_US)       // Make sure the minimum PWM pulsewidth is not
                    {                                           // smaller than the MIN_PULSEWIDTH_US limit.
                        unCurrentPW[gnCurrentMotorIDA] = _MIN_PULSEWIDTH_US;
                    }

                    switch (gnCurrentMotorIDA)
                    {
                        case 0:
                            OSSetTaskContext(ptrTask, 3, 1);            // Next state = 3, timer = 1.
                            break;
                        case 1:
                            OSSetTaskContext(ptrTask, 4, 1);            // Next state = 4, timer = 1.
                            break;
                        case 2:
                            OSSetTaskContext(ptrTask, 5, 1);            // Next state = 5, timer = 1.
                            break;
                        case 3:
                            OSSetTaskContext(ptrTask, 6, 1);            // Next state = 6, timer = 1.
                            break;
                        case 4:
                            OSSetTaskContext(ptrTask, 7, 1);            // Next state = 7, timer = 1.
                            break;
                        case 5:
                            OSSetTaskContext(ptrTask, 8, 1);            // Next state = 8, timer = 1.
                            break;
                        case 6:
                            OSSetTaskContext(ptrTask, 9, 1);            // Next state = 9, timer = 1.
                            break;

                        default:
                            OSSetTaskContext(ptrTask, 10, 1);            // Next state = 10, timer = 1.
                            break;
                    }
                    unMotorPW = unCurrentPW[gnCurrentMotorIDA];
                    
                }
                else
                {
                    gunRCServoStat[gnCurrentMotorIDA] = 0;              // Indicate RC servo motor is not moving.
                    OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);        // Next state = 2, timer = around 2.3 msec.
                }

                gnCurrentMotorIDA++;
                if (gnCurrentMotorIDA == nNoChannel)                    // Max no. of channels reached?
                {
                    gnCurrentMotorIDA = 0;                              // Reset motor/actuator ID.
                }
            break;

            case 3: // State 3 - Generate 1 pulse for motor/device 1 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;  
                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)                       
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR0bits.RP64R = 0b010000;             // RP64 or RD0 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bits mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR0bits.RP64R = 0b010001;             // RP64 or RD0 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.
                              
                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;                   // 32 bits mode.
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }                                      
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 4: // State 4 - Generate 1 pulse for motor/device 2 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.


                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin. 
                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)   
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    
                    RPOR0bits.RP65R = 0b010000;             // RP65 or RD1 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bits mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    
                    RPOR0bits.RP65R = 0b010001;             // RP65 or RD1 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;                   // 32 bits mode.
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 5: // State 5 - Generate 1 pulse for motor/device 3 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)  
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR1bits.RP66R = 0b010000;             // RP66 or RD2 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR1bits.RP66R = 0b010001;             // RP66 or RD2 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 6: // State 6 - Generate 1 pulse for motor/device 4 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)  
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR1bits.RP67R = 0b010000;             // RP67 or RD3 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    //RPOR9bits.RP101R = 0b010001;            // RP101 or RF5 connected to OC2's output.
                    RPOR1bits.RP67R = 0b010001;             // RP67 or RD3 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 7: // State 7 - Generate 1 pulse for motor/device 5 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                
                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT) 
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR2bits.RP68R = 0b010000;             // RP68 or RD4 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR2bits.RP68R = 0b010001;             // RP68 or RD4 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 8: // State 8 - Generate 1 pulse for motor/device 6 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.                

                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR2bits.RP69R = 0b010000;             // RP69 or RD5 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR2bits.RP69R = 0b010001;             // RP69 or RD5 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 9: // State 9 - Generate 1 pulse for motor/device 7 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                
                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR9bits.RP100R = 0b010000;            // RP100 or RF4 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR9bits.RP100R = 0b010001;            // RP100 or RF4 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            case 10: // State 10 - Generate 1 pulse for motor/device 8 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to (65535/FCYC)+1 clock cycles, only OC1 is used.  For delay greater than (65535/FCYC)+1
                    // clock cycles, OC1 and OC2 are connected in cascade to form a 32 bits timer 
                    // (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _PWM_SERVO_MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                
                if (unMotorPW < _PWM_SERVO_SINGLE_COUNTER_LIMIT)
                                                            // Check if exceed the limit of single OC module (16 bits).
                                                            // For FCYC = 60MHz, this corresponds to 65535/60 = 1092.468
                                                            // or 1093.                    
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR9bits.RP101R = 0b010000;            // RP101 or RF5 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR9bits.RP101R = 0b010001;            // RP101 or RF5 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 2, __SERVO_DELAY);   // Next state = 2, timer = around 2.3 msec.
            break;

            default:
                gunOSError = gunOSError | 0x0001;           // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1);            // Back to state = 0, timer = 1.
            break;
	}
    }
}



