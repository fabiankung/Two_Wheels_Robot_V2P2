
// Filename         : Robot_Tasks_dsPIC33E.c
// Author(s)		: Fabian Kung
// Last modified	: 25 July 2022


// Include MPLAB XC16 standard libraries.
#include <math.h>
#include <stdio.h>

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\osmain.h"

// Include header files for driver used.  Here absolute path is used, to edit if one changes folder.
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Motor_RCServo\RCServo_API_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_ADC_DAC_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_UART1_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_I2C_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Bluetooth_HC_05\Driver_HC_05_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_UART2_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_QuadEncoder_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_PWMServo_V101.h"

//
// --- Microcontroller Pin Usage ---
// Pin Name                      uC Pin                External Module Pin Function             uC Pin Function
// --------                      ------                ----------------------------             --------------
#define PIN_DCMD_M1CA           _RE1                   // Pin RE1 = DC motor driver, Motor 1,output A.
#define PIN_DCMD_M1CB           _RE2                   // Pin RE2 = DC motor driver, Motor 1,output B.
#define PIN_DCMD_M2CA           _RE3                   // Pin RE3 = DC motor driver, Motor 2,output A.
#define PIN_DCMD_M2CB           _RE6                   // Pin RE6 = DC motor driver, Motor 2,output B.

//
// --- PUBLIC VARIABLES ---
//
// Definition for move direction:
#define		_FORWARD				1
#define		_REVERSE				2
#define     _BACKWARD               2
#define		_STOP					0

// Standard status:
#define     _READY                  1
#define     _NOT_READY              0
#define     _ERROR                  -1

//#define     __DEBUG__             // Uncomment this statement during debug or when tuning the default state feedback 
                                    // coefficients. This will disable adaptive feedback feature of the balancing routines.
                                    // As different sets of state-feedback coefficients are used for different situation,
                                    // the behavior of the robot will be confusing if we attempt to tune the 
                                    // state-feedback coefficients.  

// --- Load Robot Hardware Description Table (HDT) ---

//#include "RobotParams_75_1_Pololu.h" 
//#include "RobotParams_100_1_Cytron_001.h"   // For production version V2P2 with 42mm diameter rubber wheels and 100:1 HP 
#include "RobotParams_100_1_Cytron_002.h" // micrometer gear motors from Cytron Technologies.

#include "FFT.h"

char    gbytFirmwareVersion[5] = "V1.25";

// --- End of Load Robot HDT ---

//
// --- PRIVATE VARIABLES ---
//

// Constants stored in the controller Program Memory.
int       progData[8]   __attribute__((space(prog), address(0x015000))) = {_FP_DEFAULT_FP , _FKX_DEFAULT_FP, _FKV_DEFAULT_FP, _FCW_DEFAULT_FP, _R_DEFAULT_FP, 
                                        _L_MOT_OFFSET_DEFAULT_FP, _R_MOT_OFFSET_DEFAULT_FP, _MOTOR_DRIVER_SETTING_DEFAULT}; 
// Store program constants at address 43010 or 0x015000.  
// Important: Address of progData[0] should be aligned to a page boundary in the dsPIC33E program 
// memory and within the range of PC (program counter) to avoid linker error.
// Note that in dsPIC33E the program memory
// space starts at line 2 for address = 0x000000.  Each program memory location is 24 bits or 
// 2x 16-bits memory words, the last 8 bits of the 2nd word is not implemented.  
// This location should be beyond the reach of the firmware codes.

// Unsigned integers to store global system coefficients in Q8.8 fixed point format.   
unsigned int    gunProgDataKP_Q8_8;                 // System coefficients.
unsigned int    gunProgDataKX_Q8_8;
unsigned int    gunProgDataKV_Q8_8;
unsigned int    gunProgDataKWg_Q8_8;
unsigned int    gunProgDataR_Q8_8;
unsigned int    gunProgDataLMotorOffset_Q8_8;
unsigned int    gunProgDataRMotorOffset_Q8_8;
unsigned int    gunProgDataMotorDriverSetting;

unsigned int    gunProgDataKP_Ori_Q8_8;             // Backup of system coefficients.
unsigned int    gunProgDataKX_Ori_Q8_8;
unsigned int    gunProgDataKV_Ori_Q8_8;
unsigned int    gunProgDataKWg_Ori_Q8_8;
unsigned int    gunProgDataR_Ori_Q8_8;
unsigned int    gunProgDataLMotorOffset_Ori_Q8_8;
unsigned int    gunProgDataRMotorOffset_Ori_Q8_8;
unsigned int    gunProgDataMotorDriverSetting_Ori;

// --- ROBOT PARAMETERS ---

// --- Tilt Angle and Orientation Parameters as measured by IMU ---
float   gfWg_MPU6050;                   // Instantaneous tilt angular velocity from MPU6050 gyroscope in radian/sec.
float   gfTheta_MPU6050;                // Robot's tilt angle in radian, from MPU6050, in radian.
int     gnTheta_Deg_MPU6050;            // Robot's tilt angle in degree, from MPU6050, in degree.

int     gnStatus_IMU = _NOT_READY;      // IMU status.  It is recommended to check this status
                                        // upon power up to make sure the IMU is READY, otherwise
                                        // the outputs from the IMU are not valid.

#define     _ROR_UPRIGHT        0       // Upright (within +- 3 deg of normal, i.e. 0 degree).
#define     _ROR_LEAN_FRONT     1       // Lean to the front (+2 to +15 degrees).
#define     _ROR_LEAN_BACK      -1      // Lean to the back (-2 to -15 degrees).
#define     _ROR_TOPPLE_FRONT   2       // Toppled to the front (> 15 degrees).
#define     _ROR_TOPPLE_BACK    -2      // Toppled to the back (< -15 degrees).
int     gnTiltOrient_IMU = _ROR_TOPPLE_BACK;          // Robot tilt orientation

// --- Primary Control Loop Coefficients: Balancing Feedback Control Parameters ---
int   	gnC;                            // Control variable for motor driver, general.
int     gnCL;                           // Control variable for motor driver, left wheel (after adding offset).
int     gnCR;                           // Control variable for motor driver, right wheel (after adding offset).
                                        // Coefficients for state-space feedback control.
float 	gfR = 0.00;                     // Reference tilt angle in radian.
float   gfR_Ori = 0.00;                 // Backup for gfR.
float	gfP = 0.00;                     // Coefficient for tilt angle.
float   gfP_Ori = 0.00;                 // Backup for gfP.
float	gfKnw = 0.00;                   // Coefficient for linear distance traveled.
float   gfKX_Ori = 0.00;                // Backup for gfKX.  Note that gfKX and gfKnw are related by just a 
                                        // constant multiplier.
float	gfKomega = 0.00;                // Coefficient for linear velocity.
float   gfKV_Ori = 0.00;                // Backup for gfKV.
float 	gfCwg = 0.00;                   // Coefficient for time derivative of tilt angle.
float   gfCwg_Ori = 0.00;               // Backup for gfCWg.
int 	gnMotOffsetL_mV = 0;            // The motor driver circuit voltage offset, in mV, for left motor.
int     gnMotOffsetL_mV_Ori = 0;
int     gnMotOffsetR_mV = 0;            // The motor driver circuit voltage offset, in mV, for right motor.
int     gnMotOffsetR_mV_Ori = 0;        // I make the offset voltage different as in real system I observe
                                        // that there is a small mismatch between the motor driver circuits
                                        // and the motor/gearbox, and also the load.
                                        // This parameter is to account for dead-band in the motor driver
                                        // circuit and the motor/gearbox itself.  It is applied to both 
                                        // left and right motors.
// Global state errors
float   gfER = 0.0;                     // Error in reference tilt angle. 
float   gfEnw = 0.0;                    // Error in average wheel distance (in ticks).
float   gfEOmegaw = 0.0;                // Error in average wheel angular velocity.

int     gnStabilityIndexH = 0;   
int     gnStabilityIndexL = 0;
int     gnFBCTuneLevel = 0;

// --- Secondary Control Loop Coefficients: Turning, Linear movement.
int     gnKp_Turn = 0;                  // Coefficients for turning (heading mode) PD control loop.
int     gnKd_Turn = 0;
int     gnKp_Move = 0;                  // Coefficients for linear movement PD control loop.
int     gnKd_Move = 0;
int     gnKp_TurnCont = 0;              // Coefficients for turning (turn speed mode) PI control loop.
int     gnKi_TurnCont = 0;
float   gfKp_Move = 0.0;

// --- Secondary Control Loop parameters: Turning and Linear Movement, Speed Regulation ---
float	gfRoffset = 0.00;               // Offset for reference angle in radian.
int   	gnCLoffset = 0;                 // Offset for control voltage (in mV) to left wheel motor driver.
int     gnCRoffset = 0;                 // Offset for control voltage (in mV) to right wheel motor driver.
INT32   gnlDistanceSet = 0;             // Robot linear traveled distance setting in no. of ticks from quadrature encoder.
                                        // > 0 move forward.
                                        // < 0 move backward.
INT16   gnHeadingSet = 0;               // Robot heading (direction) setting.
                                        // 0 = Look straight ahead
                                        // > 0 turn left.
                                        // < 0 turn right.
float   gfOmegaWSet = 0.0;              // Robot wheel rotation velocity setting in rotation/sec.
                                        // > 0.0 move forward.
                                        // < 0.0 move backward.

INT16   gnOmegaW = 0;                   // Current average wheel angular velocity, in integer.  This is to make certain routines
                                        // more efficient as integer manipulation requires less steps.  At present this
                                        // global variable is updated regularly in process Robot_MoveLinear().

INT16   gnTurnSpeed = 0;                // 0 Robot is not turning.
                                        // > 0 turning left.
                                        // < 0 turning right.
INT16   gnTurnSpeedSet = 0;             // 0 = not turning.
                                        // > 0 turning left continuously, the magnitude indicates the rate of turning.
                                        // < 0 turning right, as above.
INT16   gnTurnMode = 0;                 // 0 or otherwise, no turning, the robot will try to maintain current heading.
                                        // 1 to enable continuous turn. Variable gnHeadingSet will be ignored, only
                                        // gnTurnSpeedSet is used to control the turning speed.
                                        // 2 turn to fixed heading value.
#define     _TURN_MODE_NONE             0
#define     _TURN_MODE_FIXED_HEADING    2
#define     _TURN_MODE_CONT_TURN        1

// --- Robot High Level States and Parameters ---

#define     _ON_ANALOG_POWER    1   // We follow the C-language convention of true (not zero) and false (zero)
#define     _OFF_ANALOG_POWER   0
#define     _ENABLE             1   
#define     _DISABLE            0

#define     _ROBOT_MODE_IDLE    0   // Robot is passive mode, just powered up or motor driver circuit is 
                                    // disabled.
#define     _ROBOT_MODE_FAC_DEFAULT   1   // Set the robot to this mode to indicate that it's system variables need to 
                                    // be restore to factory default.
#define     _ROBOT_MODE_NORMAL  2   // Normal operation mode.
#define     _ROBOT_MODE_ERROR_MOTOR_DRIVER  3   // Error - Motor driver is overloaded.
#define     _ROBOT_MODE_ERROR_BATTERY       4   // Error - Battery voltage too low.
#define     _ROBOT_MODE_ERROR_TEMPERATURE   5   // Error - Temperature too high.
#define     _ROBOT_MODE_TEST1   10  // Test mode 1.
#define     _ROBOT_MODE_TEST2   11  // Test mode 2.
#define     _ROBOT_MODE_TEST3   12  // Test mode 3.

#define     _ROBOT_MOVE_MANUAL  0   // Manual move mode.  Robot waits for command from remote control, will responds to 
                                    // sensor stimulus.
#define     _ROBOT_MOVE_AUTO    1   // Autonomous move mode, robot will make use of instructions from head unit and sensors to navigate.

#define     _HEAD_ELEVATION_SERVO   0    // Map the robot head elevation servo motor to Motor0 control.
#define     _HEAD_AZIMUTH_SERVO     1    // Map the robot head azimuth servo motor to Motor1 control.


int     gnRobotMode = _ROBOT_MODE_IDLE;             // Robot overall mode.
int     gnRobotMoveMode = _ROBOT_MOVE_MANUAL;       // Robot move mode.
int     gnRobotBalance = _DISABLE;                  // 0 = Disable/off self-balancing module.
                                                    // 1 or other = Enable self-balancing module.  
int     gnMVMStatus = 0;
unsigned int gunMVMAveLuminance = 0;

typedef struct StructRobotFlag
{
	unsigned bSENL: 	1;	// Set to indicate left object sensor detects present of object.
	unsigned bSENM: 	1;	// Set to indicate middle object sensor detects present of object.
	unsigned bSENR:     1;	// Set to indicate right object sensor detects present of object.
	unsigned bCOLLISION:	1;	// Set to indicate robot detects collision with obstacle or object.
    unsigned bMOVE:     1;      // Set to indicate robot move (whether voluntary or involuntary!)
    unsigned bSENBOT:     1;	// Set to indicate bottom object sensor detects present of object.
    unsigned bSENBACK:     1;	// Set to indicate back object sensor detects present of object.
} ROBOTFLAG;

ROBOTFLAG   gobjRobotFlag;

typedef struct StructRobotState
{
    int nTemperature_C;
    int nBatteryLevel_mV;
    int nFrontObjDistance_mm;           // Variable to store the approximate distance in mm
                                        // of any object detected by distance sensor in front
                                        // of the robot.
    int nMode;
    int nMoveMode;
    int nBalance;
} ROBOTSTATE;

ROBOTSTATE   gobjRobotState;
        
//unsigned int gunFrontObjDistance_mm = 0;            // Variable to store the approximate distance in mm
                                                    // of any object detected by distance sensor in front
                                                    // of the robot.

int     gnHeadAzimuthAngleSet_Deg = 0;              // Head azimuth angle setting, in degree.
int     gnHeadElevationAngleSet_Deg = 0;            // Head elevation angle setting, in degree.
int     gnPreferHeadingDeg = 90;
int     gnCurrentHeadingDeg;
int     gunExperimentalMode = 0;

char    *gptrSystemText;                            // Pointer to string, for system text message to remote PC/host.
int     gnSystemTextLength;                         // No. of characters in system text message.

// Variables for storing DFT results.
float   gfXR[__FFT_N];
float   gfXI[__FFT_N];
float   gfXmagsq[__FFT_N];
unsigned int    gunXmag[__FFT_N];

float   gfOmegaW_LP = 0.0;                           // Low-pass filtered average wheels rotation velocity in rotation/sec.


//
// --- PRIVATE FUNCTION PROTOTYPES ---
//
void SetLWheelProperty(int, int);                   // Routines to set the motor driver.
void SetRWheelProperty(int, int);                   //
int  nSetMovementTurn(int);                         // Routine to set the robot to turn a certain degree.
                                                    // +ve angle is left, -ve angle is right.
void SetMovementStop(void);                         // Routine to set the robot to stand still
int nPrintln(char *, int );

float FixedPointtoFloatingPoint(unsigned int);      // Routines to access and reprogram constants stored
int  nGetTemperature(void);                         // Routines to get the temperature from the on-board temperature sensor.
void SetSoundPattern(unsigned int *, int);               // Routine to setup the audio buffer so that a string of beeps can be generated.

//
// --- PRIVATE VARIABLES ---
//

// HC-05 Bluetooth wireless module command handler.
// The command packet from the Remote Host is interpreted here.  Essentially there
// are three types of command: device reset, restore parameters and set parameter.
// This function returns the next state for the state machine in the Driver_HC_05.

// --- Wireless link commands ---
#define     _SPP_ESTABLISH_LINK         0x10
#define     _SPP_LINK_ESTABLISHED       0x11
#define     _TUNABLE_COEFFICIENT_INFO   0x12
#define     _COEFFICENT_INFO            0x13
#define     _DEVICE_RESET               0x00
#define     _RES_PARAM                  0x01
#define     _SET_PARAM                  0x02

// Tunable state-feedback control coefficients descriptor.
unsigned char   gbytCeoffKp[3] = {'K', 'P', ' '};                  // 3 characters name for each tuning coefficient.
unsigned char   gbytCoeffKx[3] = {'K', 'x', ' '}; 
unsigned char   gbytCoeffKv[3] = {'K', 'v', ' '};
unsigned char   gbytCoeffKw[3] = {'K', 'w', ' '};
unsigned char   gbytCoeffRs[3] = {'R', 's', 't'};
unsigned char   gbytCoeffMoL[3] = {'M', 'o', 'L'};
unsigned char   gbytCoeffMoR[3] = {'M', 'o', 'R'};
unsigned char   gbytCoeffKp2[3] = {'K', 'p', '2'};
unsigned char   gbytCoeffKd2[3] = {'K', 'd', '2'};
unsigned char   gbytCoeffKi2[3] = {'A', 'z', 'i'};
unsigned char   gbytCoeffEle[3] = {'E', 'l', 'e'};

// Sound vocabulary
// This series of array store various sound patterns to be used by the robot to signal different scenarios.
const unsigned int  gunSoundConnectedPC[] = {1,2,3,4,5};    // Connected to remote computer.
const unsigned int  gunSoundConnectedSP[] = {1,5,2,4,3};    // COnnected to remote smart phone or tablet.
const unsigned int  gunSoundDisconnected[] = {7,5,6,4,5,3}; // All remote devices disconnected.
const unsigned int  gunSoundUpright[] = {3,8};
const unsigned int  gunSoundToppled[] = {9,4};
const unsigned int  gunSoundBatteryLow[] = {2,7,2,5};
const unsigned int  gunSoundIMUAbsent[] = {2,9,5,6};
const unsigned int  gunSoundRobotStalled[] = {4,8,7};
 
// Tunable state-feedback control coefficient structure.
typedef struct	StructCoeffTuneDef
{
	unsigned int    unDefault;          // Default value in Q8.8 fixed point format.
    unsigned int    unInterval;         // Interval in Q8.8 fixed point format.
    float           fDefault;           // Default value in floating point format.
    unsigned char   *pbytDes;           // Pointer to a string.
}	COEFFTUNEDEF;

COEFFTUNEDEF    objCoeffDef[10];

// Function to initialize a stream of tones in the Audio driver.
void SetSoundPattern(unsigned int *unSoundArray, int nCount)
{
    int nIndex = 0;
    
    if (nCount > 0)
    {
        while(nCount > 0)
        {
            gnAudioTone[nIndex] = unSoundArray[nIndex];
            nCount--;
            nIndex++;
        }
        gnAudioTone[nIndex] = 0;    // End the string of beep with a 0.
    }
}

// Callback function for HC-05 Bluetooth module
// Argument:        None
// Return:          The next state in the HC-05 driver state machine
 int HC05_CommHandler()
{
    int nTemp, nTemp2;
     
    if (gobjDriverHC05.bytRFCommand == _DEVICE_RESET)       // Check if the host request
                                                            // for client to reset.
    {
        gobjDriverHC05.nRFLinkState = 0;                    // Reset RF link established.
        gobjDriverHC05.nEnRFTxPeriodicData = 0;             // Disable periodic transmission of data to remote host.
        return 10;                                          // Return the RF SPP link process.
    }
    else if (gobjDriverHC05.bytRFCommand == _RES_PARAM)     // Restore parameter sequence.
    {
        return 21;                                          // Return to message clearing loop.
    }
    else if (gobjDriverHC05.bytRFCommand == _SET_PARAM)     // Set parameters routines.
    {
        gnAudioTone[0] = 3;                                 // Beep.
        gnAudioTone[1] = 0;

        nTemp = gobjDriverHC05.bytRFArgument1;                // Get parameter index.
        nTemp2 = gobjDriverHC05.bytRFArgument2 - 90;          // Get change in parameter from the initial or start value.
                                                // This value ranges from 0-180, with initial value at 90.
        if (nTemp == 0)                         // First parameter.
        {
            gfP = gfP_Ori + (nTemp2 * _FP_INTERVAL);           // Update the floating version of the variable.
            gunProgDataKP_Q8_8 = gunProgDataKP_Ori_Q8_8 + (nTemp2*_FP_INTERVAL_FP); // Update the fixed point version of the variable.
        }
        else if (nTemp == 1)                    // Second parameter.
        {
            gfKnw = gfKX_Ori + (nTemp2 * _FKX_INTERVAL);        // Update the floating version of the variable.    
            gfKnw = gfKnw * _KXTOKNW_COEFF;                     // Convert kX to knw.    
            gunProgDataKX_Q8_8 = gunProgDataKX_Ori_Q8_8 + (nTemp2*_FKX_INTERVAL_FP); // Update the fixed point version of the variable.
        }
        else if (nTemp == 2)                    // Third parameter.
        {
            gfKomega = gfKV_Ori + (nTemp2 * _FKV_INTERVAL);     // Update the floating version of the variable.
            gfKomega = gfKomega * _KVTOKOMEGA_COEFF;            // Convert kV to kOmega.
            gunProgDataKV_Q8_8 = gunProgDataKV_Ori_Q8_8 + (nTemp2*_FKV_INTERVAL_FP); // Update the fixed point version of the variable.    
        }
        else if (nTemp == 3)                    // Fourth parameter.
        {
            gfCwg = gfCwg_Ori + (nTemp2 * _FCW_INTERVAL);       // Update the floating version of the variable.
            gunProgDataKWg_Q8_8 = gunProgDataKWg_Ori_Q8_8 + (nTemp2*_FCW_INTERVAL_FP); // Update the fixed point version of the variable. 
        }
        else if (nTemp == 4)                    // Fifth parameter.
        {
            gfR = gfR_Ori + (nTemp2 * _R_INTERVAL);             // Update the floating version of the variable.
            gunProgDataR_Q8_8 = gunProgDataR_Ori_Q8_8 + (nTemp2*_R_INTERVAL_FP); // Update the fixed point version of the variable. 
        }
        else if (nTemp == 5)
        {
            gnMotOffsetL_mV = gnMotOffsetL_mV_Ori + (nTemp2 * _MOS_INTERVAL);
            //gunProgDataLMotorOffset_Q8_8 = gunProgDataLMotorOffset_Ori_Q8_8 + (nTemp2 * _MOS_INTERVAL_FP);
            //gnKp_Move = _FP2_MOVE_DEFAULT + nTemp2;
            //gfKd_Move = nTemp2*0.002;
        }
        else if (nTemp == 6)
        {        
            gnMotOffsetR_mV = gnMotOffsetR_mV_Ori + (nTemp2 * _MOS_INTERVAL);
            //gunProgDataRMotorOffset_Q8_8 = gunProgDataRMotorOffset_Ori_Q8_8 + (nTemp2 * _MOS_INTERVAL_FP);
            

        }
        else if (nTemp == 7)
        {
            //gfKi_Move = _FI2_MOVE_DEFAULT + (nTemp2*0.000005);
            //gnKd_Move = nTemp2;      
            gnKd_Turn = _FD2_TURN_DEFAULT + nTemp2;
            //gnKi_TurnCont = _FI2_TURNCONT_DEFAULT + nTemp2;
        }
        else if (nTemp == 8)
        {
            gnPreferHeadingDeg = nTemp2 + 90;
             
            //gnHeadAzimuthAngleSet_Deg = _HEAD_AZIMUTH_ANGLE_DEFAULT + nTemp2;
            //gnKp_Move = _FP2_MOVE_DEFAULT + nTemp2;
            //gfKp_Move = _FP2_MOVE2_DEFAULT + nTemp2*0.0001;           
            //gnKp_Turn = _FP2_TURN_DEFAULT + nTemp2;
            //gnKp_TurnCont = _FP2_TURNCONT_DEFAULT + nTemp2*10;            
        }
        else if (nTemp == 9)
        {   
            gnHeadElevationAngleSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT + nTemp2;
        }
        return 21;                          // Return to message clearing loop.
    }
    else    // Unhandled commands.
    {
        return 21;                          // Return to message clearing loop.
    }
}

// Callback function for HC-05 Bluetooth module.
// HC-05 Bluetooth wireless module ASCII data handler.
// This callback function handles user command from Bluetooth client.
 void HC05_ASCIIHandler()
{  
    BYTE   bytTemp;
    
    bytTemp = gobjDriverHC05.bytRFCommand;
    
    if (bytTemp != '@')
    {
        switch (bytTemp)
        {
            case 'x': // Stop.
                gnAudioTone[0] = 2;
                gnAudioTone[1] = 0;
                SetMovementStop();
                nSetMovementTurn(0);  
                break;

            case 'f': // Move forward fast.
                gnAudioTone[0] = 3;
                gnAudioTone[1] = 0;
                
                gfOmegaWSet = _VELOCITY_MOVE_FAST;    
                //gfOmegaWSet = _VELOCITY_MOVE_NORMAL;            
                break;

            case 'F': // Move forward at normal speed.
                gnAudioTone[0] = 4;
                gnAudioTone[1] = 0;                        
                gfOmegaWSet = _VELOCITY_MOVE_NORMAL;
                //gfOmegaWSet = _VELOCITY_MOVE_SLOW;               
                break;
                
            case 's': // Move forward at slow speed.
                gnAudioTone[0] = 1;
                gnAudioTone[1] = 0;                       
                gfOmegaWSet = _VELOCITY_MOVE_SLOW;              
                break;     
                
            case 't': // Move backward.
                gnAudioTone[0] = 5;
                gnAudioTone[1] = 0; 
                //gfOmegaWSet = -_VELOCITY_MOVE_FAST;            
                gfOmegaWSet = -_VELOCITY_MOVE_NORMAL;                       
                break;
               
            case 'l': // Turn left a bit.
                gnAudioTone[0] = 6;
                gnAudioTone[1] = 0;
                nSetMovementTurn(15); 
                gnTurnMode = _TURN_MODE_FIXED_HEADING;              
                break;
                
            case 'r': // Turn right a bit.
                gnAudioTone[0] = 7;
                gnAudioTone[1] = 0;
                nSetMovementTurn(-15); 
                gnTurnMode = _TURN_MODE_FIXED_HEADING;         
                break;
                
            case 'c': // Continuous turn left.
                      // Sending the character 'c' will toggle the continuous turning
                      // on or off.
                gnAudioTone[0] = 6;
                gnAudioTone[1] = 8;
                gnAudioTone[2] = 6;
                gnAudioTone[3] = 0;
                
                if (gnTurnSpeedSet == 0)
                {    
                    gnTurnSpeedSet = _CONT_TURN_SPEED_DEFAULT;   // Enable continuous turning, and constant turning speed.    
                    gnTurnMode =  _TURN_MODE_CONT_TURN;  
                }
                else
                {
                    nSetMovementTurn(0);        // Stop all turning modes.                    
                    gnCLoffset = 0;
                    gnCRoffset = 0;                      
                }                        
                break;                

            case 'C': // Continuous turn right.
                      // Sending the character 'C' will toggle the continuous turning
                      // on or off.
                gnAudioTone[0] = 7;
                gnAudioTone[1] = 5;
                gnAudioTone[2] = 7;
                gnAudioTone[3] = 0;
                
                if (gnTurnSpeedSet == 0)
                {    
                    gnTurnSpeedSet = -_CONT_TURN_SPEED_DEFAULT;  // Enable continuous turning, and constant turning speed.    
                    gnTurnMode =  _TURN_MODE_CONT_TURN;                         
                }
                else
                {
                    nSetMovementTurn(0);        // Stop all turning modes.                    
                    gnCLoffset = 0;
                    gnCRoffset = 0;                             
                }                          
                break; 
                
            case 'm': // Change movement mode.                
                gnAudioTone[0] = 4;
                gnAudioTone[1] = 1;
                gnAudioTone[2] = 8;
                gnAudioTone[3] = 0;
                if (gnRobotMoveMode == _ROBOT_MOVE_MANUAL) 
                {
                    gnRobotMoveMode = _ROBOT_MOVE_AUTO;     // Engage Autonomous movement mode.
                }
                else
                {
                    gnRobotMoveMode = _ROBOT_MOVE_MANUAL;   // Engage Manual movement mode.
                }    
                break;   
                
            case 'N': // Engage normal mode.
                gnAudioTone[0] = 9;
                gnAudioTone[1] = 0;
                gnRobotMode = _ROBOT_MODE_NORMAL;   
                break; 
                
            case 'T': // Engage test mode 1.
                gnAudioTone[0] = 10;
                gnAudioTone[1] = 0;
                gnRobotMode = _ROBOT_MODE_TEST1;   
                break;                  
                
            case 'D': // Engage test mode 2.
                gnAudioTone[0] = 11;
                gnAudioTone[1] = 0;
                gnRobotMode = _ROBOT_MODE_TEST2;  
                break;   
                
            case 'n': // Engage test mode 3.
                gnAudioTone[0] = 12;
                gnAudioTone[1] = 0;
                gnRobotMode = _ROBOT_MODE_TEST3;  
                break;                   
                
            case 'E': // To put experimental function here.
                gunExperimentalMode = 1;
                gnAudioTone[0] = 14;
                gnAudioTone[1] = 0;       
                break;

            case 'S': // Store coefficients.
                if (gnRobotBalance == _DISABLE) 
                { // Only program the program memory if the robot is not standing upright.
                    gnAudioTone[0] = 13;
                    gnAudioTone[1] = 7;
                    gnAudioTone[2] = 5;
                    gnAudioTone[3] = 0;       
                    OSEnterCritical();                      // Disable all processor interrupts.
                    T1CONbits.TON = 0;                      // Turn off Timer1.  Timer1 powers the
                                                            // RTOS scheduler.
                    // Read 1 page of flash memory location into RAM buffer. See "osmain.h"
                    // for the size of 1 page of RAM buffer.
                    ProgMemReadPage(__builtin_tblpage(progData),__builtin_tbloffset(progData));
                    gnRamBuffer[0] = gunProgDataKP_Q8_8;    // Update 1st system variable.
                    gnRamBuffer[1] = 0;                     // Only lower 16-bits are used.
                    gnRamBuffer[2] = gunProgDataKX_Q8_8;    // Update 2nd system variable.
                    gnRamBuffer[3] = 0;                     // Only lower 16-bits are used.
                    gnRamBuffer[4] = gunProgDataKV_Q8_8;    // Update 3rd system variable.
                    gnRamBuffer[5] = 0;                     // Only lower 16-bits are used.
                    gnRamBuffer[6] = gunProgDataKWg_Q8_8;   // Update 4th system variable.
                    gnRamBuffer[7] = 0;                     // Only lower 16-bits are used.
                    gnRamBuffer[8] = gunProgDataR_Q8_8;     // Update 5th system variable.
                    gnRamBuffer[9] = 0;                     // Only lower 16-bits are used. 
                    gnRamBuffer[10] = gunProgDataLMotorOffset_Q8_8;     // Update 6th system variable.
                    gnRamBuffer[11] = 0;                     // Only lower 16-bits are used. 
                    gnRamBuffer[12] = gunProgDataRMotorOffset_Q8_8;     // Update 7th system variable.
                    gnRamBuffer[13] = 0;                     // Only lower 16-bits are used.                  
                    ProgMemErasePage(__builtin_tblpage(progData),__builtin_tbloffset(progData));                       
                                                            // Erase 1 page of flash memory which
                                                            // encompass the system variables.
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),0);   // Since 1 page consists of 8 rows, 
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),1);   // we need to execute flash write routine
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),2);   // 8 times to update all the flash memory
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),3);   // location contain in 1 page.
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),4);
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),5);
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),6);
                    ProgMemRowWrite(__builtin_tblpage(progData),__builtin_tbloffset(progData),7);
                
                    OSExitCritical();                       // Enable processor interrupts. 
                    T1CONbits.TON = 1;                      // Turn on Timer1 again.
                }
                else
                {
                    gnAudioTone[0] = 7;                     // Indicate update constants procedure
                    gnAudioTone[1] = 13;                    // is not successful.
                    gnAudioTone[2] = 0;                     
                } 
                break;
                
            case 'R': // Restore coefficients to factory default.
                if ((gnRobotBalance == _DISABLE) && (gnRobotMode == _ROBOT_MODE_IDLE))
                { // Only program the program memory if the robot is not standing upright.
                    gnAudioTone[0] = 13;
                    gnAudioTone[1] = 7;
                    gnAudioTone[2] = 0;       
                    gnRobotMode = _ROBOT_MODE_FAC_DEFAULT;
                }
                else
                {
                    gnAudioTone[0] = 7;                     // Indicate update constants procedure
                    gnAudioTone[1] = 13;                    // is not successful.
                    gnAudioTone[2] = 10;
                    gnAudioTone[3] = 5;
                    gnAudioTone[4] = 0;                     
                }                 
                break;
                
            default:
                break;
        } // switch (bytTemp)
    } 
    else
    {
        nPrintln("Request",7);
    }
	gobjDriverHC05.bytRFCommand = 0;                      // Reset command byte.
}

// Callback function for HC-05 Bluetooth module
// HC-05 Bluetooth wireless module binary data handler.
void HC05_BinaryHandler()
{

}

// Callback function for HC-05 Bluetooth module
// The periodic data packet is 64 bytes long.
// Byte 0 = ID of machine.
// Byte 1 = Identification of data packet, binary data in this case.
// Byte 2 = 8-bits positive integer nData8bit1.
// Byte 3 = 8-bits positive integer nData8bit2.
// Byte 4 = 8-bits positive integer nData8bit3.
// Byte 5 = 8-bits positive integer nData8bit4.
// Byte 6 = 8-bits positive integer nData8bit5.
// Byte 7 = 8-bits positive integer nData8bit6.
// Byte 8 = 8-bits positive integer nData8bit7.
// Byte 9 = 8-bits positive integer nData8bit8.
// Byte 10 = 8-bits positive integer nData8bit9.
// Byte 11 = 8-bits positive integer nData8bit10.
// Byte 12 = 8-bits positive integer nData8bit11.
// Byte 13 = 8-bits positive integer nData8bit12.
// Byte 14-15 = 16 bits positive integer data1.
// Byte 16-17 = 16 bits positive integer data2.
// Byte 18-19 = 16 bits positive integer data3.
// Byte 20-21 = 16 bits positive integer data4.
// Byte 22-63 = Miscellaneous data.  
//
// 31 Dec 2018:  Our assignment for the data payload as follows:
// nData8bit1 = tilt angle + 127 (127 being the offset as tilt angle ranges from -127 to + 127)
// nData8bit2 = 50x(Right wheel velocity) + 127 (actual value of velocity is small, so need to scale
// up)
// nData8bit3 = 50x(Left wheel velocity) + 127
// nData8bit4 = 50x(Average wheel velocity) + 127
// nData8bit5 = ((Right wheel count) / 4) + 127
// nData8bit6 = ((Left wheel count) / 4) + 127
// nData8bit7 = (Average wheel count / 4) + 127
// nData8bit8 = (Heading / 2) + 127
// nData8bit9 = (Set tilt angle) + 127
// nData8bit10 = 50x(Set average wheel velocity) + 127
// nData8bit11 = ((Set average wheel count) / 2) + 127
// nData8bit12 = (Set heading / 2) + 127

void HC05_SendPeriodData()
{
   unsigned int unTemp, unTemp2, unTemp3, unTemp4;
   int nData8bit1, nData8bit2, nData8bit3, nData8bit4;
   int nData8bit5, nData8bit6, nData8bit7, nData8bit8;
   int nData8bit9, nData8bit10, nData8bit11, nData8bit12;
   int nIndex;
   
    if (gobjDriverHC05.nEnRFTxPeriodicData == _DRIVER_HC_05_TX_BINARYDATA)      // Check if to transmit binary or ASCII data. 
    {   
        // 8-bits unsigned data packet.
        //nData8bit1 = gnFBCTuneLevel + 127;
        //nData8bit1 = gunMVMAveLuminance + 127;
        nData8bit1 = gnTheta_Deg_MPU6050 + 127;
        nData8bit2 = gfOmegaRW * 20; 
        nData8bit2 = nData8bit2 + 127;
        nData8bit3 = gfOmegaLW * 20; 
        nData8bit3 = nData8bit3 + 127;
        nData8bit4 = gfOmegaW * 20; 
        //nData8bit4 = gfOmegaW_LP * 20; 
        nData8bit4 = nData8bit4 + 127;        
        nData8bit5 = (gnDistanceMoveRW>>3) + 127;   // Distance in ticks divided by 8.
        nData8bit6 = (gnDistanceMoveLW>>3) + 127;       
        nData8bit7 = (gnDistanceMoveW>>3) + 127;    
        //nData8bit8 = (gnHeading>>1) + 127;
        nData8bit8 = (gnCurrentHeadingDeg>>1) + 127;
        nData8bit9 = (gfR + gfRoffset)*57.2958;     // Convert the tilt angle offset to degree.
        nData8bit9 = nData8bit9 + 127;
        nData8bit10 = gfOmegaWSet * 20;
        nData8bit10 = nData8bit10 + 127; 
        nData8bit11 = (gnlDistanceSet>>3) + 127;   // Distance set in ticks divided by 8.      
        //nData8bit12 = (gnHeadingSet>>1) + 127;
        nData8bit12 = (gnPreferHeadingDeg>>1) + 127;
        
        // 16-bits unsigned data packet.
        //unTemp4 = (gfP*100) + 4095;               // Use this to monitor the change in FB control coefficients.
        unTemp4 = gnC + 4095;       // Add positive offset to make the data unsigned integer.  
        unTemp = gobjRobotState.nFrontObjDistance_mm + 4095;
        //unTemp = gobjDriverADC.unADC1_mV + 4095;    // Transmit ADC channel values.
        unTemp2 = gobjDriverADC.unADC2_mV + 4095;
        unTemp3 = gobjDriverADC.unADC3_mV + 4095;
        
        /* 
        // For debugging
        unTemp = (gnStabilityIndexH<<2) + 4095;    // Transmit ADC channel values.
        unTemp2 = (gnStabilityIndexL<<1) + 4095;
        unTemp3 = gunXmag[0] + 4095;
        unTemp4 = gnC + 4095;       // Add positive offset to make the data unsigned integer.  
        */
        
        // Pack data into transmit buffers.
        gbytTXbuffer[2] = nData8bit1;
        gbytTXbuffer[3] = nData8bit2;
        gbytTXbuffer[4] = nData8bit3;
        gbytTXbuffer[5] = nData8bit4;
        gbytTXbuffer[6] = nData8bit5;
        gbytTXbuffer[7] = nData8bit6;
        gbytTXbuffer[8] = nData8bit7;
        gbytTXbuffer[9] = nData8bit8;
        gbytTXbuffer[10] = nData8bit9;  
        gbytTXbuffer[11] = nData8bit10;
        gbytTXbuffer[12] = nData8bit11;
        gbytTXbuffer[13] = nData8bit12;        
        gbytTXbuffer[14] = unTemp >> 8;           // Send upper 8-bits.
        gbytTXbuffer[15] = unTemp;                // Send lower 8-bits.
        gbytTXbuffer[16] = unTemp2 >> 8;          // Send upper 8-bits.
        gbytTXbuffer[17] = unTemp2;               // Send lower 8-bits.
        gbytTXbuffer[18] = unTemp3 >> 8;          // Send upper 8-bits.
        gbytTXbuffer[19] = unTemp3;               // Send lower 8-bits.  
        gbytTXbuffer[20] = unTemp4 >> 8;          // Send upper 8-bits.
        gbytTXbuffer[21] = unTemp4;               // Send lower 8-bits.
        // --- Start of Misc data data ---
        gbytTXbuffer[22] = gobjRobotFlag.bSENL + (gobjRobotFlag.bSENM<<1) + (gobjRobotFlag.bSENR<<2) +  (gobjRobotFlag.bCOLLISION<<4) + (gobjRobotFlag.bMOVE<<3);
        unTemp = gnHeading + 4095;                  // Add offset to heading to make it unsigned integer.
        gbytTXbuffer[23] = gobjDriverQuadEncoder.nDeltaDistance + 127;
        gobjDriverQuadEncoder.nDeltaDistance = 0;   // Clear change of distance traveled in no. of ticks since last
                                                    // reading.
        gbytTXbuffer[24] = unTemp >> 8;             // Send upper 8-bits of
                                                    // offset heading.
        gbytTXbuffer[25] = unTemp;                  // Send lower 8-bits of 
                                                    // offset heading.
        gbytTXbuffer[26] = (gnPreferHeadingDeg>>1) + 127;   
        gbytTXbuffer[27] = gunXmag[0]>>1;           // FFT module output, magnitude only.
        gbytTXbuffer[28] = gunXmag[1]>>1;
        gbytTXbuffer[29] = gunXmag[2]>>1;
        gbytTXbuffer[30] = gunXmag[3]>>1;
        gbytTXbuffer[31] = gunXmag[4]>>1;
        gbytTXbuffer[32] = gunXmag[5]>>1;
        gbytTXbuffer[33] = gunXmag[6]>>1;
        gbytTXbuffer[34] = gunXmag[7]>>1;      
   }
   else if (gobjDriverHC05.nEnRFTxPeriodicData == _DRIVER_HC_05_TX_TEXTDATA)  // Transmit ASCII data packet. 
   {
        if (gnSystemTextLength > 50)
        {
            gnSystemTextLength = 50;        // Limit text length to 50 characters per line.
        }
        for (nIndex = 0; nIndex < gnSystemTextLength; nIndex++ )
        {
            gbytTXbuffer[nIndex + 2] = *(gptrSystemText + nIndex);
        }
        gbytTXbuffer[nIndex + 2] = 0xD;     // Append the newline character.        
        gobjDriverHC05.nEnRFTxPeriodicData = _DRIVER_HC_05_TX_BINARYDATA;   // Set the system back to default binary packet transmission.
   }
}

///
/// Process name	: Robot_Sensor_MPU6050
///
/// Author          : Fabian Kung
///
/// Last modified	: 1 May 2022
///
/// Code Version	: 0.90
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  I2C pins
///
/// MODULE          :  MPU6050 external module.
///
/// DRIVER          :  Proce_I2C_Driver
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variable :  gnTheta_Deg_MPU6050 - Integer version of angle, in degree.
///                    gfTheta_MPU6050 - Floating point version of angle, in radian.
///                    gfWg_MPU6050 - Floating point value of angular velocity in rad/sec
///

/// Description     :  This is a driver for Invensense 6-axis IMU chip, the MPU6050.
///                    If there is no delay in each states, the sampling rate is 4.0 msec, 
///                    e.g. the accelerometer and gyroscope outputs are sample at 4.0 msec interval.  
///                    Complementary Filter or Kalman Filter techniques are used to fuse the outputs 
///                    of accelerometer and gyroscope to estimate the actual inclination angle. There
///                    are two versions of the process, one using CF and one using KF, thus the user
///                    can select the appropriate one be activating the process using the
///                    kernel function OSCreateTask() in the Main() routine.
///
///                    Sequence of operation:
///                    1. Upon power up the I2C driver is initialized.
///                    2. The driver then poll the MPU6050 WhoAmI register to make sure the sensor is
///                       attached to the robot controller.  If not the driver revert back to step 1.
///                    3. Once MPU6050 is verified connected to the robot controller, the driver
///                       proceeds to initialize the sensor.
///                    4. Finally the driver goes into normal operating mode, reading the output of the
///                       accelerometer and gyroscope at fixed intervals and compute the angle from
///                       these readings.
///                     
/// Example of usage: 
///                   gfTheta_MPU6050 gives the angle measured along Y axis in radian.
///                   gnTheta_Deg_MPU6050 gives the same angle in degree.
///                   gnTiltOrient_IMU gives the orientation (in enumerated form).
///                   These parameters can be read as frequent as required, but their values will only
///                   be updated every 4.0 msec.


// --- Check for RTOS version required with this code ---
#ifdef __OS_VER
	#if __OS_VER < 1
		#error "Robot_Sensor_MPU6050: Incompatible OS version"
	#endif
#else
	#error "Robot_Sensor_MPU6050: An RTOS is required with this function"
#endif

#define     __MPU6050_I2C_ADDRESS       0x68    // I2C slave address of MPU6050 (Assume pin AD0 is shorted to GND).

#define	_MPU6050_SMPLRT_DIV     0x19	//125Hz
#define	_MPU6050_CONFIG			0x1A	//0x06(5Hz)
#define	_MPU6050_GYRO_CONFIG	0x1B	//2000deg/s
#define	_MPU6050_ACCEL_CONFIG	0x1C	//5Hz
#define	_MPU6050_ACCEL_XOUT_H	0x3B
#define	_MPU6050_ACCEL_XOUT_L	0x3C
#define	_MPU6050_ACCEL_YOUT_H	0x3D
#define	_MPU6050_ACCEL_YOUT_L	0x3E
#define	_MPU6050_ACCEL_ZOUT_H	0x3F
#define	_MPU6050_ACCEL_ZOUT_L	0x40
#define	_MPU6050_TEMP_OUT_H		0x41
#define	_MPU6050_TEMP_OUT_L		0x42
#define	_MPU6050_GYRO_XOUT_H	0x43
#define	_MPU6050_GYRO_XOUT_L	0x44	
#define	_MPU6050_GYRO_YOUT_H	0x45
#define	_MPU6050_GYRO_YOUT_L	0x46
#define	_MPU6050_GYRO_ZOUT_H	0x47
#define	_MPU6050_GYRO_ZOUT_L	0x48
#define	_MPU6050_PWR_MGMT_1		0x6B	//
#define	_MPU6050_WHO_AM_I		0x75	//
#define	_MPU6050_SlaveAddress	0xD0	

#define     _GP2Y0A41SK0F_COEFF_        85145                        // 125000 * 0.68116, where 0.68116 is the attenuation
                                                                     // factor of the sensor analog voltage by a resistor
                                                                     // network, i.e. 4.7k/(4.7k + 2.2k) = 0.68116
//#define     _GP2Y0A41SK0F_COEFF_        125000

// Sensor fusion algorithm using Complementary Filter approach
void Robot_Sensor_MPU6050_CF(TASK_ATTRIBUTE *ptrTask)
{
    //int     nTemp;
    unsigned int unTemp;
    static  int     nAccelRaw, nGyroRaw, nAccelzRaw;
    static  float   fTemp1, fTemp2, fTemp3;
    static  float   fAngleAccel;
    static  float   fTheta, fTheta1;
    long lngTemp;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                gbytI2CSlaveAdd =  __MPU6050_I2C_ADDRESS;               // Initialize slave device I2C address. 
                fTheta = 0.0;
                fTheta1 = 0.0;
                nAccelRaw = 0;
                nGyroRaw = 0;
                gfWg_MPU6050 = 0.0;
                gfTheta_MPU6050 = 0.0;
                gnTheta_Deg_MPU6050 = 0;
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC); // Next state = 3, timer = 100 msec.
                break;
            
            case 1: // State 1 - Read WhoAmI register of MPU6050 to make sure it is present.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _MPU6050_WHO_AM_I;                  // Start address of register.
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 2, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                }                     
                break;
                
            case 2: // State 2 - Check the content of the read to make sure MPU6050 is present.  Else restart.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    if (gbytI2CRXbuf[0] == 0x68)                        // The default content of WhoAmI register is 0x68.
                    {
                        OSSetTaskContext(ptrTask, 3, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 3, timer = 1 msec.
                    }
                    else
                    {
                        SetSoundPattern(gunSoundIMUAbsent,4);          // Sound an alarm if IMU not detected.   
                        OSSetTaskContext(ptrTask, 0, 500*__NUM_SYSTEMTICK_MSEC);    // Back to state = 0, timer = 500 msec.
                    }
                }
                else if (gI2CStat.bCommError == 1)                      // Check for I2C bus error.
                {
                    OSSetTaskContext(ptrTask, 0, 1);                    // Next state = 0, timer = 1.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.
                }
                break;
                
            case 3: // State 3 - Initialize MPU6050.
                    // Set clock source to internal PLL (from X-gyroscope).
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_PWR_MGMT_1;                // Start address of register.
                    gbytI2CTXbuf[0] = 0x01;                             // Data.  Wakes up device and set internal clock to X gyro PLL.
                    //gbytI2CTXbuf[0] = 0x00;                             // Data.  Wakes up device and set internal clock to 8 MHz RC oscillator.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 4, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 4, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 3, 1);                    // Next state = 3, timer = 1.
                }                
                break;

            case 4: // State 4 - Initialize MPU6050.
                    // Set sample rate for both accelerometer and gyroscope.
                    // Sample rate = (Actual output rate)/(1 + _MPU6050_SMPLRT_DIV)
                    // According to Jeff Rowland, the sample rate for accelerometer is 1 kHz.
                    // For gyroscope it is 8 kHz if the digital LPF (dLPF) is disabled, and 1 kHz
                    // if dLPF is enabled.  I suppose this is an 8 tabs IIR filter.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_SMPLRT_DIV;                // Start address of register.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - sample rate = 333.33 sample/sec.
                    //gbytI2CTXbuf[0] = 0x03;                             // Data - sample rate = 250 sample/sec.
                    //gbytI2CTXbuf[0] = 0x04;                             // Data - sample rate = 200 sample/sec.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - sample rate = 1000 sample/sec.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - sample rate = 142.857 sample/sec.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 5, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 5, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 4, 1);                    // Next state = 4, timer = 1.
                }                
                break;
                
            case 5: // State 5 - Initialize MPU6050 - Set external synchronization and digital LPF bandwidth.
                
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_CONFIG;                    // Start address of register.
                    //gbytI2CTXbuf[0] = 0x01;                           // Data - disable external synchronization, BW around 194 Hz.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - disable external synchronization, BW around 94 Hz.
                    gbytI2CTXbuf[0] = 0x03;                             // Data - disable external synchronization, BW around 44 Hz.
                    //gbytI2CTXbuf[0] = 0x04;                             // Data - disable external synchronization, BW around 20 Hz.
                    //gbytI2CTXbuf[0] = 0x05;                             // Data - disable external synchronization, BW around 10 Hz.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - disable external synchronization, BW around 5 Hz.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 6, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 6, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1);                    // Next state = 5, timer = 1.
                }                
                break;                

            case 6: // State 6 - Initialize MPU6050 - Set full-scale range of gyroscope.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_GYRO_CONFIG;               // Start address of register.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - +-250 deg/sec.
                    //gbytI2CTXbuf[0] = 0x08;                             // Data - +-500 deg/sec.
                    //gbytI2CTXbuf[0] = 0x10;                             // Data - +-1000 deg/sec.
                    //gbytI2CTXbuf[0] = 0x18;                             // Data - +-2000 deg/sec.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 7, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 7, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 6, 1);                    // Next state = 6, timer = 1.
                }                
                break;   

            case 7: // State 7 - Initialize MPU6050 - Set full-scale range of accelerometer.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_ACCEL_CONFIG;              // Start address of register.
                    //gbytI2CTXbuf[0] = 0x00;                             // +-2g.
                    gbytI2CTXbuf[0] = 0x08;                             // +-4g.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 8, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 8, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 7, 1);                    // Next state = 7, timer = 1.
                }                
                break;
                       
            case 8: // State 8 - Initiate read accelerometer, chip temperature, gyroscope data.
                    // Address (hex) Register         Index
                    // 3B            ACCEL_XOUT_H       -
                    // 3C            ACCEL_XOUT_L       -
                    // 3D            ACCEL_YOUT_H       0
                    // 3E            ACCEL_YOUT_L       1
                    // 3F            ACCEL_ZOUT_H       2
                    // 40            ACCEL_ZOUT_L       3
                    // 41            TEMP_OUT_H         4
                    // 42            TEMP_OUT_L         5
                    // 43            GYRO_XOUT_H        6    
                    // 44            GYRO_XOUT_L        7
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 8;                               // Indicate no. of bytes to read.
                                                                        // To read from ACCEL_YOUT_H to GYRO_XOUT_L
                    gbytI2CRegAdd = _MPU6050_ACCEL_YOUT_H;              // Start address of register.
                    gI2CStat.bRead = 1;                                 // Start read. 
                    OSSetTaskContext(ptrTask, 9, 17);                   // Next state = 9, timer = 17.                    
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 8, 1);                        // Next state = 8, timer = 1.
                }                  
                break;
                
            case 9: // State 9 - Convert raw gyroscope and accelerometer data into rad/sec and g.
                gfWg_MPU6050 = 0.0001332*nGyroRaw;                      // Sensitivity = 1/131 or 0.0076336 for +-250 dps full scale.
                                                                        // Since our unit is radian not degree. We must multiply
                                                                        // by (pi/180) or 0.017453.  Thus the total coefficient is
                                                                        // 0.017453*0.0076336 = 0.0001332.  
                fTemp1 = 0.00012207*nAccelRaw;                          // Convert to g, where 1/8192 = 0.00012207
                OSSetTaskContext(ptrTask, 10, 1);                       // Next state = 10, timer = 1.
                break;
                
            case 10: // State 10 - Convert accelerometer g to radian.
                fAngleAccel = asinf(fTemp1);                            // Convert gravitational acceleration g to radian.
                                                                        // Inverse sine function takes a lot of computational resources,
                                                                        // so we only perform this step in 1 state.
                OSSetTaskContext(ptrTask, 11, 1);                       // Next state = 11, timer = 1.                
                break;
                
            case 11: // State 11 - Compute tilt angle using Complementary Filter approach.  Sampling interval
                     // is 4 msec (0.004).
                fTheta1 = fTheta;                               // Store previous tilt angle sample.
                fTemp1 = gfWg_MPU6050*0.004;                    // Theta(n) = a(Theta(n-1) + W*Delta_t) + (1-a)(ThetaAcc(n))
                fTemp2 = fTheta1 + fTemp1;                      // The time-constant for the low-pass and high-pass filters                
                OSSetTaskContext(ptrTask, 12, 1);               // Next state = 12, timer = 1.
                break;

            case 12: // State 12 - Combine outputs from accelerometer and gyroscope using Complementary Filters
                    // approach.    Coefficient a = 0.994.
                fTemp2 = 0.994*fTemp2;			
                fTemp3 = 0.006*fAngleAccel;		 
                fTheta = fTemp2 + fTemp3;                       // Work out final tilt angle.               
                
                OSSetTaskContext(ptrTask, 13, 1);               // Next state = 13, timer = 1.
            break;
            
            case 13: // State 13 - Update global variables.
                gnTheta_Deg_MPU6050 = fTheta*57.2958;               // Tilt angle in degree.
                gfTheta_MPU6050 = fTheta;                           // Tilt angle in radian.
                if (gnTheta_Deg_MPU6050 > 40)
                //if (gnTheta_Deg_MPU6050 > 30)                     // Check tilt angle for threshold of leaning forward.
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_FRONT;           // Robot topples to the front.
                }
                else if (gnTheta_Deg_MPU6050 > 4)
                {
                    gnTiltOrient_IMU = _ROR_LEAN_FRONT;             // Robot leans to the front.
                }
                else if (gnTheta_Deg_MPU6050 >= -4)
                {
                    gnTiltOrient_IMU = _ROR_UPRIGHT;                // Robot is upright.
                }
                else if (gnTheta_Deg_MPU6050 > -40)
                //else if (gnTheta_Deg_MPU6050 > -34)                 // Check tilt angle for threshold of leaning back.
                {
                    gnTiltOrient_IMU = _ROR_LEAN_BACK;              // Robot leans to the back.
                }
                else
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_BACK;           // Robot topples to the back.               
                }                                     
                gnStatus_IMU = _READY;                          // Indicate IMU module has valid output.
                OSSetTaskContext(ptrTask, 14, 1);               // Next state = 14, timer = 1.
                break;
                
            case 14: // State 14 - Sample output of reflective IR Sensor and convert to mm.
                    // Note: This routine is tailored to Sharp GP2Y0A41SK0F reflective IR sensor with range from
                    // 4cm to 30 cm.
                //gobjDriverADC.unADC1_mV = 408;
                if (gobjDriverADC.unADC1_mV > 0)                                // To prevent divide-by-zero.
                {                
                    lngTemp = _GP2Y0A41SK0F_COEFF_/gobjDriverADC.unADC1_mV;     // See notes and datasheet on the derivation
                                                                                // equation for this sensor. Basically the 
                                                                                // estimated distance of object from the sensor
                                                                                // is inversely proportional to the analog voltage
                                                                                // voltage. The coefficient of 125000 is via
                                                                                // trial-and-error fitting experimental data
                                                                                // to the trial equation.
                    gobjRobotState.nFrontObjDistance_mm = lngTemp; 
                    if (gobjRobotState.nFrontObjDistance_mm > 255)              // Limit to 255 mm.
                    {
                        gobjRobotState.nFrontObjDistance_mm = 255;
                    }
                }
                else
                {
                    gobjRobotState.nFrontObjDistance_mm = 255;
                }                   
                OSSetTaskContext(ptrTask, 15, 1);                       // Next state = 15, timer = 1.                
                break;                   

            case 15: // State 15 - Read gyroscope data.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    unTemp = gbytI2CRXbuf[0]<<8;                        // Get upper 8 bits.                                //
                    nAccelRaw = unTemp + gbytI2CRXbuf[1];               // Form 16 bits unsigned integer by adding lower 8 bits. 
                                                                        // Note that the data is in signed 16-bits 2's complement format. 
                    unTemp = gbytI2CRXbuf[6]<<8;                        // Get upper 8 bits.                                //
                    nGyroRaw = unTemp + gbytI2CRXbuf[7];                // Form 16 bits unsigned integer by adding lower 8 bits.                     
                                                                        // Note that the data is in signed 16-bits 2's complement format.      
                    if (nAccelRaw > 8192)                               // Limit the raw accelerator output, for +-4g full scale.
                    {
                        nAccelRaw = 8192;
                    }
                    if (nAccelRaw < -8192)
                    {
                        nAccelRaw = -8192;
                    }          
                    unTemp = gbytI2CRXbuf[2]<<8;
                    nAccelzRaw = unTemp + gbytI2CRXbuf[3];
                    OSSetTaskContext(ptrTask, 8, 1);                   // Next state = 8, timer = 1. 
                    //OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.   
                }
                else
                {
                    OSSetTaskContext(ptrTask, 15, 1);                  // Next state = 15, timer = 1.
                }
                break;                
            
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); 	// Back to state = 0, timer = 1.
            break;                
        }
    }
}

// Sensor fusion algorithm using Kalman-Filter approach
void Robot_Sensor_MPU6050_KF(TASK_ATTRIBUTE *ptrTask)
{
    //int     nTemp;
    unsigned int unTemp;
    static  int     nAccelRaw, nGyroRaw;
    static  float   fTemp1;
    static  float   fAngleAccel;
    static  float   fTheta, fTheta1;
    long lngTemp;
   // Static variables for Kalman-Filter.
    static  float   fSigma11, fSigma12, fSigma21, fSigma22;
    static  float   fbX1, fbX2;
    static  float   fX1, fX2;
    static  float   fP11 = 0.0, fP12 = 0.0, fP21 = 0.0, fP22 = 0.0;
    static  float   fdt = 0.004;            // 4 msec sampling interval.
    static  float   fZ = 0.0;
    static  float   fSigmaGyro = 0.0008;    // From datasheet of MPU6050, total rms gyroscope noise
                                            // is 0.05 deg/sec typical (DLPFCFG=2 (100Hz))
                                            // or 0.00873 rad/sec
    static  float   fSigmaAcce = 0.02;       // From datasheet of MPU6050, typical power spectral density
                                            // is 400 ug/sqrt(Hz). For a bandwidth of around 50 Hz
                                            // variance = 0.02 g
                                            // Converting this to angle in radian = 0.02 rad
    static  float   fK0, fK1, fK2;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                gbytI2CSlaveAdd =  __MPU6050_I2C_ADDRESS;               // Initialize slave device I2C address. 
                fTheta = 0.0;
                fTheta1 = 0.0;
                nAccelRaw = 0;
                nGyroRaw = 0;
                gfWg_MPU6050 = 0.0;
                gfTheta_MPU6050 = 0.0;
                gnTheta_Deg_MPU6050 = 0;
                
                // KF variables initialization.
                //fSigma11 = 0.016*fdt;
                fSigma11 = 1.0;
                fSigma12 = 0.0;
                fSigma21 = 0.0;
                fSigma22 = fSigma11;
                fX1 = -0.5;  // in radian, assume robot initially line face up.
                fX2 = 0.01; // gyroscope drift, in radian-per-second.                
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC); // Next state = 3, timer = 100 msec.
                break;
            
            case 1: // State 1 - Read WhoAmI register of MPU6050 to make sure it is present.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _MPU6050_WHO_AM_I;                  // Start address of register.
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 2, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                }                     
                break;
                
            case 2: // State 2 - Check the content of the read to make sure MPU6050 is present.  Else restart.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    if (gbytI2CRXbuf[0] == 0x68)                        // The default content of WhoAmI register is 0x68.
                    {
                        OSSetTaskContext(ptrTask, 3, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 3, timer = 1 msec.
                    }
                    else
                    {
                        SetSoundPattern(gunSoundIMUAbsent,4);          // Sound an alarm if IMU not detected.   
                        OSSetTaskContext(ptrTask, 0, 500*__NUM_SYSTEMTICK_MSEC);    // Back to state = 0, timer = 500 msec.
                    }
                }
                else if (gI2CStat.bCommError == 1)                      // Check for I2C bus error.
                {
                    OSSetTaskContext(ptrTask, 0, 1);                    // Next state = 0, timer = 1.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.
                }
                break;
                
            case 3: // State 3 - Initialize MPU6050.
                    // Set clock source to internal PLL (from X-gyroscope).
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_PWR_MGMT_1;                // Start address of register.
                    gbytI2CTXbuf[0] = 0x01;                             // Data.  Wakes up device and set internal clock to X gyro PLL.
                    //gbytI2CTXbuf[0] = 0x00;                             // Data.  Wakes up device and set internal clock to 8 MHz RC oscillator.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 4, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 4, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 3, 1);                    // Next state = 3, timer = 1.
                }                
                break;

            case 4: // State 4 - Initialize MPU6050.
                    // Set sample rate for both accelerometer and gyroscope.
                    // Sample rate = (Actual output rate)/(1 + _MPU6050_SMPLRT_DIV)
                    // According to Jeff Rowland, the sample rate for accelerometer is 1 kHz.
                    // For gyroscope it is 8 kHz if the digital LPF (dLPF) is disabled, and 1 kHz
                    // if dLPF is enabled.  I suppose this is an 8 tabs IIR filter.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_SMPLRT_DIV;                // Start address of register.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - sample rate = 333.33 sample/sec.
                    //gbytI2CTXbuf[0] = 0x03;                             // Data - sample rate = 250 sample/sec.
                    //gbytI2CTXbuf[0] = 0x04;                             // Data - sample rate = 200 sample/sec.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - sample rate = 1000 sample/sec.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - sample rate = 142.857 sample/sec.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 5, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 5, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 4, 1);                    // Next state = 4, timer = 1.
                }                
                break;
                
            case 5: // State 5 - Initialize MPU6050 - Set external synchronization and digital LPF bandwidth.
                
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_CONFIG;                    // Start address of register.
                    //gbytI2CTXbuf[0] = 0x01;                           // Data - disable external synchronization, BW around 194 Hz.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - disable external synchronization, BW around 94 Hz.
                    gbytI2CTXbuf[0] = 0x03;                             // Data - disable external synchronization, BW around 44 Hz.
                    //gbytI2CTXbuf[0] = 0x04;                             // Data - disable external synchronization, BW around 20 Hz.
                    //gbytI2CTXbuf[0] = 0x05;                             // Data - disable external synchronization, BW around 10 Hz.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - disable external synchronization, BW around 5 Hz.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 6, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 6, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1);                    // Next state = 5, timer = 1.
                }                
                break;                

            case 6: // State 6 - Initialize MPU6050 - Set full-scale range of gyroscope.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_GYRO_CONFIG;               // Start address of register.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - +-250 deg/sec.
                    //gbytI2CTXbuf[0] = 0x08;                             // Data - +-500 deg/sec.
                    //gbytI2CTXbuf[0] = 0x10;                             // Data - +-1000 deg/sec.
                    //gbytI2CTXbuf[0] = 0x18;                             // Data - +-2000 deg/sec.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 7, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 7, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 6, 1);                    // Next state = 6, timer = 1.
                }                
                break;   

            case 7: // State 7 - Initialize MPU6050 - Set full-scale range of accelerometer.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_ACCEL_CONFIG;              // Start address of register.
                    //gbytI2CTXbuf[0] = 0x00;                             // +-2g.
                    gbytI2CTXbuf[0] = 0x08;                             // +-4g.
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 8, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 8, timer = 1 msec.
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 7, 1);                    // Next state = 7, timer = 1.
                }                
                break;
                       
            case 8: // State 8 - Initiate read accelerometer, chip temperature, gyroscope data.
                    // Address (hex) Register         Index
                    // 3B            ACCEL_XOUT_H       -
                    // 3C            ACCEL_XOUT_L       -
                    // 3D            ACCEL_YOUT_H       0
                    // 3E            ACCEL_YOUT_L       1
                    // 3F            ACCEL_ZOUT_H       2
                    // 40            ACCEL_ZOUT_L       3
                    // 41            TEMP_OUT_H         4
                    // 42            TEMP_OUT_L         5
                    // 43            GYRO_XOUT_H        6    
                    // 44            GYRO_XOUT_L        7
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 8;                               // Indicate no. of bytes to read.
                                                                        // To read from ACCEL_YOUT_H to GYRO_XOUT_L
                    gbytI2CRegAdd = _MPU6050_ACCEL_YOUT_H;              // Start address of register.
                    gI2CStat.bRead = 1;                                 // Start read. 
                    //OSSetTaskContext(ptrTask, 9, 17);                   // Next state = 9, timer = 17.  
                    OSSetTaskContext(ptrTask, 9, 11);                   // Next state = 9, timer = 11.  
                }                
                else
                {
                    OSSetTaskContext(ptrTask, 8, 1);                        // Next state = 8, timer = 1.
                }                  
                break;
                
            case 9: // State 9 - Convert raw gyroscope and accelerometer data into rad/sec and g.
                gfWg_MPU6050 = 0.0001332*nGyroRaw;                      // Sensitivity = 1/131 or 0.0076336 for +-250 dps full scale.
                                                                        // Since our unit is radian not degree. We must multiply
                                                                        // by (pi/180) or 0.017453.  Thus the total coefficient is
                                                                        // 0.017453*0.0076336 = 0.0001332.  
                fTemp1 = 0.00012207*nAccelRaw;                          // Convert to g, where 1/8192 = 0.00012207
                OSSetTaskContext(ptrTask, 10, 1);                       // Next state = 10, timer = 1.
                break;
                
            case 10: // State 10 - Convert accelerometer g to radian.
                fAngleAccel = asinf(fTemp1);                            // Convert gravitational acceleration g to radian.
                                                                        // Inverse sine function takes a lot of computational resources,
                                                                        // so we only perform this step in 1 state.
                fTheta1 = fTheta;                                       // Store previous tilt angle sample.
                OSSetTaskContext(ptrTask, 11, 1);                       // Next state = 11, timer = 1.                
                break;
                
            case 11: // State 11 - Compute tilt angle using KF Filter approach.  
                // State prediction.
                fbX1 = fX1 + fdt*(gfWg_MPU6050 - fX2);
                fbX2 = fX2;             
                OSSetTaskContext(ptrTask, 12, 1);               // Next state = 12, timer = 1.
                break;

            case 12: // State 12 - Compute tilt angle using KF Filter approach.  
                // Find P11
                fTemp1 = fdt*fSigma22;
                fP11 = fdt*(fTemp1 - fSigma21 - fSigma12 + fSigmaGyro);
                fP11 = fP11 + fSigma11;          
                OSSetTaskContext(ptrTask, 13, 1);               // Next state = 13, timer = 1.
            break;

            case 13: // State 13 - Compute tilt angle using KF Filter approach.  
                // Find P12 and P21.
                fTemp1 = fdt*fSigma22;
                fP12 = fSigma12 - fTemp1;
                fP21 = fSigma21 - fTemp1;
                OSSetTaskContext(ptrTask, 14, 1);               // Next state = 14, timer = 1.
            break;       
            
            case 14: // State 14 - Compute tilt angle using KF Filter approach.  
                // Find P22 and Z.
                //fP22 = fSigma22 + (fSigmaGyro);                   // P22
                // Note: 1 May 2022, the 2nd term in fP22 is the sigma for the gyroscope systematic bias
                // wb. Since the bias is very small for MPU6050, we can set it to 0. This seems better
                // from experiment as it leads to smaller initial error during sudden change in tilt
                // angle. The earlier version which assume sigma for wb to be equal to gyroscope
                // sigma leads to larger initial error when robot body is tilted forward or backward
                // rapidly, resulting in overshoot and undershoot of the tilt angle estimate, which 
                // causes vibration.
                fP22 = fSigma22;                   // P22
                fZ =  fAngleAccel - fbX1;                       // Measure discrepancy with accelerometer value.
                OSSetTaskContext(ptrTask, 15, 1);               // Next state = 15, timer = 1.
            break;       

            case 15: // State 15 - Compute tilt angle using KF Filter approach.  
                // Find Kalman Factors.
                fK0 = 1.0/(fP11 + fSigmaAcce);
                fK1 = fK0*fP11;
                fK2 = fK0*fP21;
                OSSetTaskContext(ptrTask, 16, 1);               // Next state = 16, timer = 1.
            break;
            
            case 16: // State 16 - Compute tilt angle using KF Filter approach.  
                // Find new state.
                fX1 = fbX1 + (fK1*fZ);
                fX2 = fbX2 + (fK2*fZ);
                fTheta = fX1;                                   // Assign fTheta to global variable.
                OSSetTaskContext(ptrTask, 17, 1);               // Next state = 17, timer = 1.
            break;            
            
            case 17: // State 17 - Compute tilt angle using KF Filter approach.  
                // Find new Sigma11 and Sigma12.
                fTemp1 = 1 - fK1;
                fSigma11 = fTemp1*fP11;
                fSigma12 = fTemp1*fP12;
                OSSetTaskContext(ptrTask, 18, 1);               // Next state = 18, timer = 1.
            break;    
            
            case 18: // State 18 - Compute tilt angle using KF Filter approach.  
                // Find new Sigma21 and Sigma22.
                fSigma21 = fP21 - (fK2*fP11);
                fSigma22 = fP22 - (fK2*fP12);
                gnStatus_IMU = _READY;                          // Indicate IMU module has valid output.
                OSSetTaskContext(ptrTask, 19, 1);               // Next state = 19, timer = 1.
            break; 
            
            case 19: // State 19 - Update global variables.
                gnTheta_Deg_MPU6050 = fTheta*57.2958;               // Tilt angle in degree.
                gfTheta_MPU6050 = fTheta;                           // Tilt angle in radian.
                if (gnTheta_Deg_MPU6050 > 40)
                //if (gnTheta_Deg_MPU6050 > 30)                     // Check tilt angle for threshold of leaning forward.
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_FRONT;           // Robot topples to the front.
                }
                else if (gnTheta_Deg_MPU6050 > 4)
                {
                    gnTiltOrient_IMU = _ROR_LEAN_FRONT;             // Robot leans to the front.
                }
                else if (gnTheta_Deg_MPU6050 >= -4)
                {
                    gnTiltOrient_IMU = _ROR_UPRIGHT;                // Robot is upright.
                }
                else if (gnTheta_Deg_MPU6050 > -40)
                //else if (gnTheta_Deg_MPU6050 > -34)                 // Check tilt angle for threshold of leaning back.
                {
                    gnTiltOrient_IMU = _ROR_LEAN_BACK;              // Robot leans to the back.
                }
                else
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_BACK;           // Robot topples to the back.               
                }                    
                gnStatus_IMU = _READY;                          // Indicate IMU module has valid output.
                OSSetTaskContext(ptrTask, 20, 1);               // Next state = 20, timer = 1.
                break;
                
            case 20: // State 20 - Sample output of reflective IR Sensor and convert to mm.
                    // Note: This routine is tailored to Sharp GP2Y0A41SK0F reflective IR sensor with range from
                    // 4cm to 30 cm.
                //gobjDriverADC.unADC1_mV = 408;
                if (gobjDriverADC.unADC1_mV > 0)                                // To prevent divide-by-zero.
                {                
                    lngTemp = _GP2Y0A41SK0F_COEFF_/gobjDriverADC.unADC1_mV;     // See notes and datasheet on the derivation
                                                                                // equation for this sensor. Basically the 
                                                                                // estimated distance of object from the sensor
                                                                                // is inversely proportional to the analog voltage
                                                                                // voltage. The coefficient of 125000 is via
                                                                                // trial-and-error fitting experimental data
                                                                                // to the trial equation.
                    gobjRobotState.nFrontObjDistance_mm = lngTemp; 
                    if (gobjRobotState.nFrontObjDistance_mm > 255)              // Limit to 255 mm.
                    {
                        gobjRobotState.nFrontObjDistance_mm = 255;
                    }
                }
                else
                {
                    gobjRobotState.nFrontObjDistance_mm = 255;
                }                   
                OSSetTaskContext(ptrTask, 21, 1);                       // Next state = 21, timer = 1.                
                break;                   

            case 21: // State 21 - Read gyroscope data.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    unTemp = gbytI2CRXbuf[0]<<8;                        // Get upper 8 bits.                                //
                    nAccelRaw = unTemp + gbytI2CRXbuf[1];               // Form 16 bits unsigned integer by adding lower 8 bits. 
                                                                        // Note that the data is in signed 16-bits 2's complement format. 
                    unTemp = gbytI2CRXbuf[6]<<8;                        // Get upper 8 bits.                                //
                    nGyroRaw = unTemp + gbytI2CRXbuf[7];                // Form 16 bits unsigned integer by adding lower 8 bits.                     
                                                                        // Note that the data is in signed 16-bits 2's complement format.      
                    if (nAccelRaw > 8192)                               // Limit the raw accelerator output, for +-4g full scale.
                    {
                        nAccelRaw = 8192;
                    }
                    if (nAccelRaw < -8192)
                    {
                        nAccelRaw = -8192;
                    }                                        
                    OSSetTaskContext(ptrTask, 8, 1);                   // Next state = 8, timer = 1. 
                    //OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.   
                }
                else
                {
                    OSSetTaskContext(ptrTask, 21, 1);                  // Next state = 21, timer = 1.
                }
                break;                
            
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); 	// Back to state = 0, timer = 1.
            break;                
        }
    }
}

///
/// Function name	: SetRWheelProperty
///
/// Author          : Fabian Kung
///
/// Last modified	: 17 March 2016
///
/// Code version	: 1.02
///
/// Processor		: Generic
///
/// Description		: Set the rotation direction of right wheel.
///
/// Arguments		: nDirection - 1 = Forward, 2 = Reverse and 0 = stop.
///                  
/// Return          : None.
///
/// Global variable	: None
///
/// Description:
/// 
/// Careless typo mistake in the default case corrected on 7 Jan 2020!

void SetRWheelProperty(int nDirection, int nWSpeed)
{
    // Note: when setting the output to drive large
    // capacitive load under logic high, sometimes
    // we need to assert this twice or more in order
    // for the pin to switch properly.

    switch (nDirection)
    {
	case _FORWARD: // Forward.
            PIN_DCMD_M1CA = 0;
            PIN_DCMD_M1CA = 0;
            PIN_DCMD_M1CB = 1;
            PIN_DCMD_M1CB = 1;				
	break;

	case _REVERSE: // Reverse.
            PIN_DCMD_M1CA = 1;
            PIN_DCMD_M1CA = 1;
            PIN_DCMD_M1CB = 0;
            PIN_DCMD_M1CB = 0;
	break;

	default: // Stop motor.
            //PIN_DCMD_M1CA = 0;
            //PIN_DCMD_M1CA = 0;
            //PIN_DCMD_M1CB = 0;
            //PIN_DCMD_M1CB = 0;
            gobjDriverDAC.unDAC1_mV = 0;          // Shut down motors by setting DAC output voltage to 0.
            gobjDriverDAC.unDAC2_mV = 0;            
            PIN_DCMD_M1CA = 1;
            PIN_DCMD_M1CA = 1;
            PIN_DCMD_M1CB = 1;
            PIN_DCMD_M1CB = 1;             
	break;
    }
}

///
/// Function name	: SetLWheelProperty
///
/// Author          : Fabian Kung
///
/// Last modified	: 17 March 2016
///
/// Code version	: 1.02
///
/// Processor		: Generic
///
/// Description		: Set the rotation direction of left wheel.
///
/// Arguments		: nDirection - 1 = Forward, 2 = Reverse and 0 = stop.
///
/// Return          : None.
///
/// Global variable	: None
///
/// Description:

void SetLWheelProperty(int nDirection, int nWSpeed)
{
    // Note: when setting the output to drive large
    // capacitive load under logic high, sometimes
    // we need to assert this twice or more in order
    // for the pin to switch properly.
    switch (nDirection)
    {
        case _REVERSE: // Reverse.
            PIN_DCMD_M2CA = 0;
            PIN_DCMD_M2CA = 0;
            PIN_DCMD_M2CB = 1;
            PIN_DCMD_M2CB = 1;
        break;

        case _FORWARD: // Forward.
            PIN_DCMD_M2CA = 1;
            PIN_DCMD_M2CA = 1;
            PIN_DCMD_M2CB = 0;
            PIN_DCMD_M2CB = 0;
        break;

        default: // Stop motor.
            //PIN_DCMD_M2CA = 0;
            //PIN_DCMD_M2CA = 0;
            //PIN_DCMD_M2CB = 0;
            //PIN_DCMD_M2CB = 0;
            gobjDriverDAC.unDAC1_mV = 0;          // Shut down motors by setting DAC output voltage to 0.
            gobjDriverDAC.unDAC2_mV = 0;            
            PIN_DCMD_M2CA = 1;
            PIN_DCMD_M2CA = 1;
            PIN_DCMD_M2CB = 1;
            PIN_DCMD_M2CB = 1;            
        break;
    }
}


///
/// Process name	: Robot_Balance
///
/// Author          : Fabian Kung
///
/// Last modified	: 6 Nov 2021
///
/// Code version	: 1.15
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  None
///
/// MODULE          :  DC Motor Driver (External)
///                    DAC (External)
///                    IMU (External)
///
/// DRIVER          :  Proce_12DAC_MCP4822_Driver
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variables	:  gnRobotBalance
///                        gnMotOffset_mV
///                        gnCLoffset
///                        gnCRoffset
///                        gunDACIn1_mV
///                        gunDACIn2_mV
///                        gnDistanceMoveW
///                        gfOmegaW
///                        gfTheta_IMU
///                        gfWg_IMU
///                        gfRoffset
///                        gfR
///                        And the feedback control coefficients
///
/// Description	: User process - Self-balancing algorithm 
///               This process implements a discrete-time feedback control loops to keep the robot upright 
///               by driving the left and right motor driver/wheels.  
///
///               1. The interval to execute the control tasks is 2.0 msec.
///
///               2. The balance controller used is the Pole-Placement method.  The controller expression 
///                  is given by:
///                 U =  KP(Theta:set - Theta:IMU) + 
///                      KCw ((dTheta/dt):set - (dTheta/dt):IMU) + 
///                      Kx(nw:set - nw:Encoder) + 
///                      Kv((d(Omegaw)/dt):set - (d(Omegaw)/dt):Encoder)
///               where parameters with :IMU or :Encoder are measured from sensors on the robot
///               and parameters with :set are user settings.
///
///                 U = control voltage to the motor driver servo amplifier.
///                 nw:Encoder = Average rotation of the wheels in number of ticks (e.g. the wheel encoder
///                      output counts the wheel rotation by the number of ticks).  
///                 Omegaw:Encoder = Wheel rotation angle / 2xpi.  This is related to nw by
///                          Thetaw = nw x (angle per tick)                        
///                 Theta:IMU = Tilt angle in radian as measured from IMU.
///
///               As the geared DC motors have some backlash, a discrete time IIR low-pass
///               filter is applied on the wheel rotational velocity output, as per the 
///               recommendation of Grasser et al 2001 paper.  According to Grasser this will
///               reduce the vibrational movement of the robot while standing still and moving.
///               The sampling interval for this IIR filter is similar to the sampling interval 
///               of the main control loop.  At present the IIR low-pass filter is 1st order,
///               and in the form of exponential averaging.
///
///               The user has a choice of using the low-pass filtered rotational velocity or 
///               the raw value from the wheel encoder sensor.


#define     _BALANCE_SAMPLING_INTERVAL_MSEC  2              // Sampling interval in milisecond for Balance Controller.

void Robot_Balance(TASK_ATTRIBUTE *ptrTask)
{
 static float   fC = 0.00;      // The output in floating point.
 static float   fC1, fC2;  
 static int     nC;             // The output in integer (mV).
 static int     nCR, nCL;		// The control voltage output for right and left motors,
                                // in mV.
 static float   fOmegaOld;
 
 //static float   gfOmegaW_LP = 0.0;     // Low-pass filtered average wheels rotation velocity in rotation/sec.
 
 
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.             
                                                    // Initial value for motor driver offset voltage.
                                                    // This value is obtained from experimentation.
                gnCLoffset = 0;                     // Initialized left and right wheels offset voltage
                gnCRoffset = 0;                     // for turning.
                gnRobotBalance = _DISABLE;
                
                gfER = 0.0;                         // Reset all error terms.
                gfEOmegaw = 0.0;                
                fOmegaOld = 0.0;                    // Accumulator for gfOmega.
                
                gobjDriverDAC.unDAC1_mV = 0;          // Shut down motors by setting DAC output voltage to 0.
                gobjDriverDAC.unDAC2_mV = 0;

                gobjDriverADC.unEnADC = 1;          // Enable ADC module (if not enabled).
                gobjDriverDAC.unEnDAC = 1;          // Enable DAC module (if not enabled.
                SetLWheelProperty(_STOP, -1.0);     // Turn off motors first.
                SetRWheelProperty(_STOP, -1.0);
                OSSetTaskContext(ptrTask, 1, 200*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 200 msec.
                //OSSetTaskContext(ptrTask, 90, 200*__NUM_SYSTEMTICK_MSEC); // Next state = 90, timer = 200 msec.
                break;
                
            case 1: // State 1 - Initialize feedback control coefficients from constants stored in program memory.
                gfP = objCoeffDef[0].fDefault;          // Initialize kP coefficient.
                gfP_Ori = gfP;                          // Also make a backup copy, as gfP can be tuned during program
                                                        // execution, so we would like to maintain an initial version.
                OSSetTaskContext(ptrTask, 2, 1);        // Next state = 2, timer = 1.
                break;
                
            case 2: // State 2 - Initialize feedback control coefficients from constants stored in program memory.
                gfKnw = objCoeffDef[1].fDefault;        // Initialize kX coefficient.
                gfKX_Ori = gfKnw;                       // Also make a backup copy, as gfP can be tuned during program
                                                        // execution, so we would like to maintain an initial version.
                gfKnw = gfKnw * _KXTOKNW_COEFF;         // Convert kX to kNw.
                OSSetTaskContext(ptrTask, 3, 1);        // Next state = 3, timer = 1.
                break;

            case 3: // State 3 - Load feedback control coefficients.
                gfKomega = objCoeffDef[2].fDefault;     // Initialize kV coefficient;
                gfKV_Ori = gfKomega;                    // Also make a backup copy, as gfKV can be tuned during program
                                                        // execution, so we would like to maintain an initial version.
                gfKomega = gfKomega * _KVTOKOMEGA_COEFF;
                OSSetTaskContext(ptrTask, 4, 1);        // Next state = 4, timer = 1.
                break;
                
            case 4: // State 4 - Load feedback control coefficients.
                gfCwg = objCoeffDef[3].fDefault;        // Initialize kwg coefficient;
                gfCwg_Ori = gfCwg;                      // Also make a backup copy, as gfCwg can be tuned during program
                                                        // execution, so we would like to maintain an initial version.
                OSSetTaskContext(ptrTask, 5, 1);        // Next state = 5, timer = 1.
                break;

            case 5: // State 5 - Load feedback control coefficient.
                gfR = objCoeffDef[4].fDefault;          // Initialize R coefficient;
                gfR_Ori = gfR;                          // Also make a backup copy, as gfR can be tuned during program
                                                        // execution, so we would like to maintain an initial version.
                OSSetTaskContext(ptrTask, 6, 1);        // Next state = 6, timer = 1.
                break;
               
            case 6: // State 6 - Load left motor offset voltage.
                gnMotOffsetL_mV = objCoeffDef[5].fDefault;  // Initialize left motor offset voltage.
                gnMotOffsetL_mV_Ori = gnMotOffsetL_mV;      // Also make a backup copy, as this parameter can be tuned 
                                                            // during program execution, so we would like to maintain 
                                                            // an initial version.
                OSSetTaskContext(ptrTask, 7, 1);        // Next state = 7, timer = 1.
                break;
                
            case 7: // State 7 - Load right motor offset voltage.
                gnMotOffsetR_mV = objCoeffDef[6].fDefault;  // Initialize right motor offset voltage.
                gnMotOffsetR_mV_Ori = gnMotOffsetR_mV;      // Also make a backup copy, as this parameter can be tuned 
                                                            // during program execution, so we would like to maintain 
                                                            // an initial version.
                OSSetTaskContext(ptrTask, 8, 1);        // Next state = 8, timer = 1.
                break;
                
            case 8: // State 8 - 
                OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
                break;                
                
            case 9: // State 9 - Check if machine machine ready before proceeding.
                if (gnRobotBalance == _DISABLE)         // Check if the balancing module is enable or not.
                {                                       // Do not run balancing module.
                    SetLWheelProperty(_STOP, -1.0);     // Check if balancing routine is still disabled.
                    SetRWheelProperty(_STOP, -1.0);
                    OSSetTaskContext(ptrTask, 21, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 21, timer = 1 msec.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 10, 1);    // Next state = 10, timer = 1.                    
                }
                break;

            case 10: // State 10 - Compute all state error terms.
                gfER = (gfR + gfRoffset) - gfTheta_MPU6050;  // Compute tilt angle error term, an offset is added to 
                                                        // allow for additional tuning. 
                gfEnw = gnlDistanceSet - gnDistanceMoveW; // Compute distance error term.                 
                
                //fE2 = gfOmegaWSet - gfOmegaW;         // Add wheel rotation velocity offset, using raw result.
                gfEOmegaw = gfOmegaWSet - gfOmegaW_LP;   // Add wheel rotation velocity offset, using low-pass
                                                        // filtered result.    
                OSSetTaskContext(ptrTask, 11, 1);       // Next state = 11, timer = 1.
                break;

            case 11: // State 11 - First term.         
                fC = gfP*gfER;                          // Tilt angle control term.  
                OSSetTaskContext(ptrTask, 12, 1);       // Next state = 12, timer = 1.                    
                break;

            case 12: // State 12 - 2nd loop.  Note that the set tilt angle velocity is assume to be
                    // 0.0 always.
                fC = fC - (gfCwg*gfWg_MPU6050);         // Tilt angular velocity control term,
                                                        // as measured from gyroscope.                                 
                OSSetTaskContext(ptrTask, 13, 1);       // Next state = 13, timer = 1.                
                break;
                
                
            case 13: // State 13 - 3rd term.
                fC1 = gfKnw * gfEnw;                    // Distance error term.    
                OSSetTaskContext(ptrTask, 14, 1);       // Next state = 14, timer = 1.
                break;
            
            case 14: // State 14 - 4th term.
                fC2 = gfKomega * gfEOmegaw;             // Wheel rotation velocity error term.                      
                OSSetTaskContext(ptrTask, 15, 1);       // Next state = 15, timer = 1.                  
                break;
                
            case 15: // State 15 - Compute control variable.
                fC = fC + fC1 + fC2;                    // Compute total error term. 
                OSSetTaskContext(ptrTask, 16, 1);        // Next state = 16, timer = 1.                   
                break;
                
            case 16: // State 16 - Update exponential averaging function for wheel angular velocity.                                                   
                gfOmegaW_LP = (0.9*fOmegaOld) + (0.1*gfOmegaW); // Implement exponential averaging.
                                                                    // This is a 1st order IIR filter expression,
                                                                    // v_ave(t) = b*v_ave(t-1) + (1-b)*v(t)
                                                                    // With b = 0.9, equivalent to taking average of 10 past samples.
                                                                    // With b = 0.95, 20 past samples.
                                                                    // With b = 0.98, 50 past samples.
                OSSetTaskContext(ptrTask, 17, 1);       // Next state = 17, timer = 1.                
                break;
                
            case 17: // State 17 - Backup current filtered wheel angular velocity.
                fOmegaOld = gfOmegaW_LP;
                OSSetTaskContext(ptrTask, 18, 1);       // Next state = 18, timer = 1.                
                break;
                
            case 18: // State 18 -                      // Convert control variable into integer in milivolts.
                nC = fC*1000;                           
                nCL = nC + gnCLoffset;                  // Add left and right offsets to control movement
                nCR = nC + gnCRoffset;                  // behavior.  
                OSSetTaskContext(ptrTask, 19, 1);       // Next state = 19, timer = 1.                                   
                break;

            case 19: // State 19 - Compute control voltage for left and right wheels. Also
                     // check for consistent over-loaded condition to prevent damaging the
                     // motors.
               
                if (nCL > _MOTOR_DRIVER_MAX_mV)         // Saturate the output control voltage.
                {
                    nCL = _MOTOR_DRIVER_MAX_mV;         // Limit the input to motor driver, else this
                                                        // may saturates the servo amplifier.
                }
                else if (nCL < _MOTOR_DRIVER_MIN_mV)
                {
                    nCL = _MOTOR_DRIVER_MIN_mV;
                }

                if (nCR > _MOTOR_DRIVER_MAX_mV)         // Saturate the output control voltage.
                {
                    nCR = _MOTOR_DRIVER_MAX_mV;         // Limit the input to motor driver, else this
                                                        // may saturates the servo amplifier.
                }
                else if (nCR < _MOTOR_DRIVER_MIN_mV)
                {
                    nCR = _MOTOR_DRIVER_MIN_mV;              
                }             
                OSSetTaskContext(ptrTask, 20, 1);       // Next state = 20, timer = 1.
            break;

            case 20: // State 20 - Drive motors and wheels.
                    // Add a DC offset to account for dead-band in the motor driver circuit.
                gnC = nC;                               // Assign control variables to global integers.  
                gnCL = nCL;                             // Basically this is for plotting the control 
                gnCR = nCR;                             // variables.
                // Drive right wheel.
                if (nCR == 0)           // Note that we can independently stop the right wheel.
                {
                    SetRWheelProperty(_STOP, -1.0);
                }
                else if (nCR > 0)
                {
                    nCR = nCR + gnMotOffsetR_mV;        // Add another small offset to overcome motor & gear
                                                        // internal friction in the motor and gearbox.
                    SetRWheelProperty(_FORWARD, -1.0);              
                }                                                
                else // nCR < 0
                {
                    nCR = (-nCR) + gnMotOffsetR_mV;     // Add another small offset to overcome motor & gear
                                                        // internal friction in the motor and gearbox.
                    SetRWheelProperty(_REVERSE, -1.0);
                }
                
                // Drive left wheel.
                if (nCL == 0)             // Note that we can independently stop the left wheel.
                {
                    SetLWheelProperty(_STOP, -1.0);
                }
                else if (nCL > 0)
                {
                    nCL = nCL + gnMotOffsetL_mV;        // Add another small offset to overcome motor & gear
                                             		    // internal friction in the motor and gearbox.
                    SetLWheelProperty(_FORWARD, -1.0);              
                }                                                   
                else // nCL < 0
                {
                    nCL = (-nCL) + gnMotOffsetL_mV;     // Add another small offset to overcome motor & gear
                                                        // internal friction in the motor and gearbox.
                    SetLWheelProperty(_REVERSE, -1.0);
                }
                gobjDriverDAC.unDAC2_mV = nCR;          // Output to DAC, then to motor driver circuit.
                gobjDriverDAC.unDAC1_mV = nCL;          // Output to DAC, then to motor driver circuit. 
                                                        // Note: Make sure the assignment is correct, e.g. to the
                                                        // correct side.
                OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
            break;

            case 21: // State 21 - Shut down the wheel motor and monitor the gnRobotBalance variable.
                // Check robot orientation, if it is still toppled over.
                if (gnRobotBalance == _DISABLE)
                {        
                    if (gnRobotMode == _ROBOT_MODE_FAC_DEFAULT)  // If the robot is in the midst of restoring it's
                                                                 // system level coefficients to factory setting, we
                                                                 // to restart this process from state 0.
                    {
                        OSSetTaskContext(ptrTask, 22, 1*__NUM_SYSTEMTICK_MSEC); 	// Next state = 22, timer = 1 msec.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 21, 1*__NUM_SYSTEMTICK_MSEC); 	// Next state = 21, timer = 1 msec.
                    }
                }
                else
                {                                       // Set the robot to regain balance again.
                    gfER = 0.0;
                    gfEnw = 0.0;
                    gfEOmegaw = 0.0;      
                    fC = 0.0;
                    fC1 = 0.0;
                    fC2 = 0.0;
                    //fOmega1 = 0.0;                      // Initialize low-pass filter registers for average wheel   
                    //fOmega2 = 0.0;                      // angular velocity.
                    //fOmega3 = 0.0;  
                    //fOmega4 = 0.0;
                    //fOmega5 = 0.0;
                    //fOmega6 = 0.0;
                    fOmegaOld = 0.0;
                    gfOmegaW_LP = 0.0;
                
                    gobjDriverDAC.unDAC1_mV = 0;        // Initialize motor driver DAC.
                    gobjDriverDAC.unDAC2_mV = 0;                                                                       
                    gobjDriverQuadEncoder.nQuadEncoderStatus = _QUADENCODER_RESET;   // Reset wheel encoder status and related wheel variables.
                    OSSetTaskContext(ptrTask, 10, 1*__NUM_SYSTEMTICK_MSEC); 	// Next state = 10, timer = 1 msec.
                }
            break;

            case 22:  // State 22 - Restoring settings to factory default.  Wait until robot mode become idle.
                      // When robot mode becomes idle, it means the restoration process is completed.
                    if (gnRobotMode == _ROBOT_MODE_FAC_DEFAULT)  // If the robot is in the midst of restoring it's
                                                                 // system level coefficients to factory setting, we
                                                                 // to restart this process.
                    {   // Restoration process is still on-going.
                        OSSetTaskContext(ptrTask, 22, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 22, timer = 1 msec.
                    }
                    else
                    {   // Restoration process is completed.
                        OSSetTaskContext(ptrTask, 0, 1*__NUM_SYSTEMTICK_MSEC); 	// Next state = 0, timer = 1 msec.
                    }                                
                break;

            case 90:  // State 90 - Test sequence 1.
                gobjDriverDAC.unDAC1_mV = 600;
                gobjDriverDAC.unDAC2_mV = 600;
                SetLWheelProperty(_FORWARD, -1);
                SetRWheelProperty(_FORWARD, -1);
                OSSetTaskContext(ptrTask,91, 1000*__NUM_SYSTEMTICK_MSEC); 		// Next state = 91, timer = 1000 msec.
            break;

            case 91: // State 91 - Test sequence 2.
                SetLWheelProperty(_REVERSE, -1);
                SetRWheelProperty(_REVERSE, -1);
                OSSetTaskContext(ptrTask,90, 1000*__NUM_SYSTEMTICK_MSEC);         // Next state = 92, timer = 1000 msec.
            break;
            
           case 92:  // State 92 - Test sequence 3.
                gobjDriverDAC.unDAC1_mV = 600;
                gobjDriverDAC.unDAC2_mV = 600;
                SetLWheelProperty(_FORWARD, -1);
                SetRWheelProperty(_FORWARD, -1);
                OSSetTaskContext(ptrTask,93, 1000*__NUM_SYSTEMTICK_MSEC); 		// Next state = 93, timer = 1000 msec.
            break;

            case 93: // State 93 - Test sequence 4.
                SetLWheelProperty(_REVERSE, -1);
                SetRWheelProperty(_REVERSE, -1);
                OSSetTaskContext(ptrTask,90, 1000*__NUM_SYSTEMTICK_MSEC); 	// Next state = 90, timer = 1000 msec.
            break;
            
            case 100: // State 100 - Idle, stop motors and monitor battery status.
                SetLWheelProperty(_STOP, -1.0);         //stop both wheels.
                SetRWheelProperty(_STOP, -1.0);
                OSSetTaskContext(ptrTask, 100, 100); // Yes, remain at state = 100, timer = 100.        
            break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); 	// Back to state = 0, timer = 1.
            break;
        }
    }
}


///
/// Function name	: SetMovementTurn
///
/// Author          : Fabian Kung
///
/// Last modified	: 23 Jan 2019
///
/// Code version	: 0.94
///
/// Processor		: Generic
///
/// Description		: Turn the robot.
///
/// Arguments		: Turn degree in integer format:
///                   1 < Argument <= 359 to turn left, 
///                   -359 <= Argument < -1 to turn right.
///                   >= 360 the turning angle will be set to 359.
///                   <= -360 the turning angle will be set to -359.
///                   0, stop turning.
///                  
/// Return          : Offset to gnHeadingSet.
///
/// Global variable	: gnHeadingSet
///                   gnTurnSpeedSet
///                   gnHeading
///
/// Description     : Call this function to turn the robot left or right by a certain
///                   degree or continuously.  
/// 
int nSetMovementTurn(int nAngle)
{
    int nTemp;
    
    if (nAngle > 0)                                 // Turn left, finite or continuous.
    {
        if (nAngle > 360)                           // 0 < nAngle < 360, turn left.
        {  
            nAngle = 359;                           // Limit maximum angle to 359 degrees.
        }
        nAngle = nAngle * 100;                      // Scale the angle.
        nTemp = nAngle/_TURN_DEG_PER_100;           // Integer division faster than floating point division.
        gnHeadingSet = gnHeadingSet + nTemp;  

    }
    else if (nAngle < 0)                            // Turn right, finite or continuous.
    {
        if (nAngle < -360)                          // -360 < nAngle < 0, turn right.
        {
            nAngle = -359;                          // Limit minimum angle to -359 degrees.
        }
        nAngle = nAngle * 100;                      // Scale the angle.
        nTemp = nAngle/_TURN_DEG_PER_100;           // Integer division faster than floating point division.
        gnHeadingSet = gnHeadingSet + nTemp;      
    }  
    else                                            // nAngle = 0.
    {
        gnTurnSpeedSet = 0;                         // Disable constant turning speed mode.
        gnTurnMode =  _TURN_MODE_NONE;               // Disable turning mode (if enable).    
        gnHeadingSet = gnHeading;                   // Update heading setting to current direction, disable
                                                    // heading mode.
        nTemp = 0;
    }
    return nTemp;
}

///
/// Process name	: Robot_MoveTurn
///
/// Author          : Fabian Kung
///
/// Last modified	: 1 Nov 2019
///
/// Code version	: 1.021
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  None
///
/// MODULE          :  Motor Driver.
///
/// DRIVER          :  Proce_Balance().
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variables	:  gnHeadingSet
///                        gnCLoffset
///                        gnCRoffset
///                        gnHeading
///                        gnTurnSpeed
///                        gnTurnSpeedSet
///                        gnTurnMode
///                        gnKp_TurnCont
///                        gnKi_TurnCont
///                        gnKp_Turn
///                        gnKd_Turn
///
/// Description	: This process controls the direction of the robot, whether it is
///               standing stationary or moving, turning.  Basically it monitors four
///               global parameters, 'gnHeading', 'gnHeadingSet', 'gnTurnSpeed' and
///               'gnTurnSpeedSet'.  gnHeading is the current direction of the robot,
///               being the difference between distance traveled by left and right 
///               wheels respectively.  Whereas gnHeadingSet it the required direction.
///               Similarly gnTurnSpeed and gnTurnSpeedSet are the current turning velocity
///               and required turning velocity respectively. 
///               
///                 gnHeading = Direction of robot in differential wheel ticks, positive to
///                             turn left, negative to turn right, 0 to maintain straight position.
///                 gnTurnSpeed = Turning speed in number of heading unit per 10 milliseconds.
///                               Typical value ranges from -5 to +5.  For instance
///                               if gnTurnSpeed is 3, and turn angle is 1.2 degree per heading
///                               unit.  Then the robot will be set to turn continuously
///                               at 300 heading ticks per 1 second, or 360 degree per second,
///                               e.g. 1 turn per second.
///
///               This process implements two higher order feedback control loops on top
///               of the basic balancing algorithm.  The two higher order feedback control
///               loops are (a) turn to fix heading mode, which uses a PD loop on
///               the global parameters 'gnHeading' and 'gnHeadingSet'. (b) constant 
///               turning speed mode, which uses a PI loop on the global parameters 
///               'gnTurnSpeed' and 'gnTurnSpeedSet'.  These higher order feedback loops
///               modulate the control voltages to the left and right motor drivers.
///
///               Sampling rate - At present the sampling rate for this feedback control
///               is 10 msec, or 10x slower than the balancing feedback control routines.
/// 
/// Example of usage:
///               1) When gnTurnSpeedSet = 0, Turn-to-fix-heading mode is active:
///               If gnHeadingSet = 0, then the robot will always try to face forward
///               direction, whether standing still or moving forward or backward.  If
///               during moving forward the direction changes slightly (due to slippage)
///               or difference in frictional force on the wheels, a small differential
///               offset voltage will be added to the motor driver servo amplifier, in 
///               the form of gnCLoffset and gnCRoffset.
///               If gnHeadingSet <0, then the robot will attempt to turn right to the 
///               direction set by gnHeadingSet, else it will turn
///               left.  
///               2) When gnTurnSpeedSet > or < 0, Constant-turning-speed mode is active: 
///               'gnHeading' will be ignored.
///               gnTurnSpeedSet > 0, continuous turn left, with the magnitude of gnTurnSpeedSet
///               determining the turn speed.
///               gnTurnSpeedSet < 0, continuous turn right, again magnitude of gnTurnSpeedSet
///               determines the turn speed.
///               Sub-modes of Constant-turning-speed operation:
///               In Constant-turning-speed mode, if the global parameter gnHeadingSet is 0,
///               the robot will be driven to turn at constant velocity until this mode is 
///               disabled (by setting gnTurnSpeedSet to 0).  If gnHeading is not 0, the robot
///               will be driven to turn at constant velocity until the heading specified by
///               gnHeading is reached or exceeded.  Then gnTurnSpeedSet will be set to 0 and
///               they robot will default back to Turn-to-fix-heading mode.  Thus the user can
///               activate the Constant-turning-speed mode with specified heading, and then
///               continously monitors gnTurnSpeedSet.  If gnTurnSpeedSet becomes 0, then
///               this indicates that the robot has reached the required heading.

void Robot_MoveTurn(TASK_ATTRIBUTE *ptrTask)
{
 static int nE1;		
 static int nE2;
 static int nHeadingOld = 0;
 static int nTurnSpeedErrorIntegral = 0;
 static int nTurn;
 static int nTurnSpeedError = 0;
 static int nCount = 0;
 static int nTurnThresholdP;
 static int nTurnThresholdN;
 
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                gnKp_Turn = _FP2_TURN_DEFAULT;                              // Default values for PD coefficients.
                gnKd_Turn = _FD2_TURN_DEFAULT;                              // 
                gnKp_TurnCont = _FP2_TURNCONT_DEFAULT;
                gnKi_TurnCont = _FI2_TURNCONT_DEFAULT;
                nTurnThresholdP = _MOTOR_DRIVER_TURN_MAX_mV/10;             // Here we just use the rule-of-thumb for the
                nTurnThresholdN = _MOTOR_DRIVER_TURN_MIN_mV/10;             // threshold to turn off turn mode at 10% of the
                                                                            // maximum turning voltage magnitude.
                gnHeadingSet = 0;
                nE1 = 0;
                nE2 = 0;
                gnTurnSpeed = 0;
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 1000 msec.
            break;

            case 1: // State 1 - Check if machine machine ready before proceeding.
                if (gnRobotBalance == _DISABLE)                         // Check if the balancing module is enable or not.
                {                                                       // Do not run balancing module.                
                    OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 0, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.                    
                }
            break;

            case 2: // State 2 - Turn feedback loop, calculate heading error and turning speed.  The loop interval is around 10 msec.
                nE1 = gnHeadingSet - gnHeading;                         // Compute error in current heading.
                nE2 = nE1 - nE2;                                        // Compute difference of current and previous error.
                gnTurnSpeed = gnHeading - nHeadingOld;                  // Compute present turning speed.  Here we just take the
                                                                        // turning speed as the difference between current heading
                                                                        // and previous heading, over the sampling interval (which
                                                                        // is 10 msec).
                nTurnSpeedError = gnTurnSpeedSet - gnTurnSpeed;         // Calculate the error is turning speed.
                nHeadingOld = gnHeading;                                // Update past heading value.
                if (gnTurnSpeedSet == 0)
                {                                                       // Fixed heading mode.
                    nTurnSpeedErrorIntegral = 0;                        // Clear the error integral if we are in heading mode.
                    OSSetTaskContext(ptrTask, 3, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 3, timer = 1 msec.
                }
                else
                {                                                       // Constant turning speed mode.
                    OSSetTaskContext(ptrTask, 5, 1);                    // Next state = 5, timer = 1.
                }
                break;
                
            case 3: // State 3 - Turn feedback loop, 2nd sequence, implement PD feedback control for heading mode.
                nTurn = gnKp_Turn*nE1 + gnKd_Turn*nE2;                  // Compute the turn offset differential voltage.                               
                if (nTurn > 0)                                          // Limit the offset differential
                {                                                       // voltage magnitude.
                    if (nTurn > _MOTOR_DRIVER_TURN_MAX_mV)              
                    {
                        nTurn = _MOTOR_DRIVER_TURN_MAX_mV;
                    }
                }
                else
                {
                    if (nTurn < _MOTOR_DRIVER_TURN_MIN_mV)
                    {
                        nTurn = _MOTOR_DRIVER_TURN_MIN_mV;
                    }                    
                }
                gnCLoffset = -nTurn;                                    // Add offset differential voltage.
                gnCRoffset = nTurn;                                     // to left and right wheel motor drivers.
                nE2 = nE1;                                              // Store current error value.
                OSSetTaskContext(ptrTask, 4, 1);                        // Next state = 4, timer = 1.
                break;
                
            case 4: // State 4 - Check if turn to fixed heading mode is enabled, 
                    // if enabled, and the robot has turned to the required heading,
                    // return the state-feedback coefficients to default values and disable
                    // all turning mode. 
                
                if (gnTurnMode == _TURN_MODE_FIXED_HEADING)
                {
                    if ((nTurn < nTurnThresholdP) && (nTurn > nTurnThresholdN)) // If turn offset voltage is less than 
                                                                                // the threshold, typically 10% of the 
                                                                                // max turning voltage magnitude.
                    {
                        gnTurnMode = _TURN_MODE_NONE;
                    }
                }
                
                OSSetTaskContext(ptrTask, 1, 9*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 9 msec.
                break;
                
            case 5: // State 5 - Turn feedback loop, 2nd sequence, constant turning speed mode.
                nTurnSpeedErrorIntegral = nTurnSpeedErrorIntegral + nTurnSpeedError;
                nTurn = gnKp_TurnCont*nTurnSpeedError;                      // Compute the turn offset differential voltage
                nTurn = nTurn + (gnKi_TurnCont*nTurnSpeedErrorIntegral);    // based on PI control.
                if (nTurn > 0)                                          // Limit the offset differential
                {                                                       // voltage magnitude.
                    if (nTurn > _MOTOR_DRIVER_TURN_MAX_mV)
                    {
                        nTurn = _MOTOR_DRIVER_TURN_MAX_mV;
                    }
                }   
                else
                {
                    if (nTurn < _MOTOR_DRIVER_TURN_MIN_mV)
                    {
                        nTurn = _MOTOR_DRIVER_TURN_MIN_mV;
                    }
                }
                gnCLoffset = -nTurn;                                    // Add offset differential voltage.
                gnCRoffset = nTurn;                                     // to left and right wheel motor drivers.
                nCount = 0;
                OSSetTaskContext(ptrTask, 6, 1);                        // Next state = 6, timer = 1.
                break;
                
            case 6: // State 6 - Constant turning speed mode, check timer, also monitor if turning angle or heading 
                    // reached target value.
                nCount++;                                               // Increment counter.
                if (nCount == 10)                                       // Check if time's up.
                {
                    OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                }
                else
                {
                    if (gnTurnMode == 1)                    // if continuous turning is enabled.
                    {
                        OSSetTaskContext(ptrTask, 6, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 6, timer = 1 msec.
                    }
                    else
                    {
                        if (gnTurnSpeedSet > 0)                             // Turning left.
                        {
                            if ((nE1 < 0) || (nE1 == 0))                    // Check if turning angle or heading reach the 
                            {                                               // required value.
                                gnTurnSpeedSet = 0; 
                                OSSetTaskContext(ptrTask, 1, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 1 msec.
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 6, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 6, timer = 1 msec.
                            } // if ((nE1 < 0) || (nE1 == 0))
                        }
                        else                                                // Turning right.
                        {
                            if ((nE1 > 0) || (nE1 == 0))                    // Check if turning angle or heading reach the 
                            {                                               // required value.
                                gnTurnSpeedSet = 0; 
                                OSSetTaskContext(ptrTask, 1, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 1 msec. 
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 6, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 6, timer = 1 msec.
                            } // if ((nE1 > 0) || (nE1 == 0))
                        } // if (gnTurnSpeedSet > 0)
                    } // if (gnTurnMode == 1)
                }
                break;
                
            case 10: // State 10 - Balancing operation is disabled, continue to monitor the global variable 
                     // gnRobotBalance until balancing routine is enabled again.
                if (gnRobotBalance == _DISABLE)
                {
                    OSSetTaskContext(ptrTask, 10, 10);                  // Next state = 10, timer = 10.
                }
                else
                {					
                    nE1 = 0;
                    nE2 = 0;
                    gnTurnSpeed = 0;
                    gnHeadingSet = 0;
                    gnTurnMode = 0;
                    gnTurnSpeedSet = 0;
                    nTurnSpeedErrorIntegral = 0;
                    nTurnSpeedError = 0;
                    OSSetTaskContext(ptrTask, 2, 10);                   // Next state = 2, timer = 10.
                }
            break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1);                        // Back to state = 0, timer = 1.
            break;
        }
    }
}

  
///
/// Process name	: Robot_MoveLinear
///
/// Author          : Fabian Kung
///
/// Last modified	: 6 Aug 2018
///
/// Code version	: 1.04
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  None
///
/// MODULE          :  Motor Driver.
///
/// DRIVER          :  Proce_Balance().
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gnKp_Move, gnKd_Move, gfKi_Move
///                       gfOmegaW, gnDistanceMoveW
///                       gnOmegaW
///                       gfOmegaWSet
///                       gnlDistanceSet
///
/// Description	:  This is a higher level control routine that sits on top of the main
///                balancing routine.  This routine makes the robot move forward or backward
///                according to the linear velocity set by the user processes.
///                At the lower level, to make the robot moves forward, we:
///                1) Increment the distance setting in the robot state, gnlDistanceSet.
///                2) Make the velocity setting positive, gfOmegaWSet.
///                To make the robot moves backward, the reverse of the above are done.
///                The Theta and d(Theta)/dt settings can remain at 0 at all times.
///                For more advanced control, some adjustments to gfROffsets are needed to make 
///                the movement speed smooth and symmetry in both forward and backward directions,
///                and to adapt to different conditions, like slope.
///                
///                To make the linear movement smoother, we add another layer of feedback control
///                on top of the Balancing controller.  This Linear Movement Controller (LMC) reads  
///                the global variable 'gfOmegaSet', which indicates the required linear velocity  
///                as follows:
///                gfOmegaWSet(in rotation/sec)  Movement
///                -------------                 --------
///                   > 0.0                      Forward, the larger the magnitude the faster the movement.
///                   < 0.0                      Backward, larger magnitude indicates faster movement.
///                   0.0                        Stationary.
///
///                The LMC automatically increment the parameter gnlDistanceSet if gfOmegaWSet is > 0.0, 
///                and automatically decrement gnlDistanceSet if gfOmegaWSet is < 0.0.  The time interval
///                where gnlDistanceSet is incremented or decremented is determined by:
///                 1. Working out the expected time interval (based on gfOmegaWSet) between pulses from
///                    the wheel encoders.  
///                 2. This time interval is further adjusted by adding an offset which is a function of
///                    the error between current distance traveled and expected distance based on the 
///                    velocity set.  The time interval offset is computed using a simple PD algorithm
///                    based on parameters 'gnKp_Move', 'gnKd_Move', gnlDistanceSet and gnDistanceMoveW. 
///
///                Finally for forward or backward motion, there are 4 speed standard settings defined in the 
///                robot parameters header file, depending on the magnitude of gfOmegaWSet, e.g.:
///                              Very slow
///                              Slow
///                              Normal
///                              Fast
///                  
///                Recommended speed for typical movement is Normal.  
///                3) Update on 26 June 2018.  Added automatic gradual increment/decrement of set rotational
///                velocity. For instance, if initially gfOmegaWSet = 0.3 and the system is maintaining the robot
///                speed at this setting.  If higher level routines suddenly increases gfOmegaWSet to 0.5 rotation/sec,
///                this process will gradually increase the local set velocity from 0.3 to 0.5 rotation/sec.  
///                Similar principle applies when we reduce the set rotational velocity.  Gradual increment/decrement
///                of local set velocity will prevent abrupt movement and reduce the mechanical stress on the
///                drive components.  Furthermore, to increase the efficiency of the process, this adjustment 
///                of local set velocity is done in integer format.  The average wheel angular velocity gfOmegaW is
///                converted to integer gnOmegaW by multiplying gfOmegaW with system coefficients _NOTICKSROTATION.
///                   
/// Example of usage:
/// 1. To make the robot move forward at constant velocity at moderate speed, set 
/// gfOmegaWSet = 0.3 rotation/sec for forward motion or < -0.3 rotation/sec for backward motion. 
/// 2. To make the robot move forward at constant velocity at higher speed, set 
/// gfOmegaWSet = 0.6 for forward motion or < -0.6 for backward motion.  
/// For all (1) and (2) cases above, to stop, simply set gfOmegaWSet to 0.0.  
/// 3. Maximum speed is capped to +-_MOVELINEAR_MAX_SPEED rotation/sec in the codes.

void Robot_MoveLinear(TASK_ATTRIBUTE *ptrTask)
{
    static int  nMaxTickDifference;
    static int  nDelayTick;
    static int  nSysTick1Second;                            
    int nTemp;
    static int nDistanceError;
    static int nDistanceErrorOld;

    static int  nOmegaWSet;                 // Average required wheel rotation velocity.  
    static int  nCurrentOmegaWSet;          // 
    static int  nMaxOmegaWLimit;            // Maximum allowable speed.
    
#define     _MOVELINEAR_MAX_SPEED   1.0     // In rotation/sec
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                gfOmegaWSet = 0.0;
                nOmegaWSet = 0;
                nCurrentOmegaWSet = nOmegaWSet;
                gnlDistanceSet = 0; 
                nDistanceErrorOld = 0;
                nDelayTick = 1;
                gnKp_Move = _FP2_MOVE_DEFAULT;                              // Initialize the PD loop coefficients.
                gnKd_Move = _FD2_MOVE_DEFAULT;
                
                nSysTick1Second = __NUM_SYSTEMTICK_MSEC*1000;               // Number of system ticks in 1 second.
                nMaxTickDifference = _NOTICKSROTATION*2;                    // Limit the maximum tick between current and expected to be no more
                                                                            // than the ticks for two revolutions of the wheel. 
                OSSetTaskContext(ptrTask, 1, 200*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 200 msec.
            break;
                
            case 1: // State 1 - Continue on initialization, perform some floating calculations here.
                nMaxOmegaWLimit = _MOVELINEAR_MAX_SPEED * _NOTICKSROTATION;
                OSSetTaskContext(ptrTask, 2, 1);                            // Next state = 2, timer = 1.
                break;
                
            case 2: // State 2 - Check if machine is ready before proceeding.  If the balancing routine
                    // is not running keep on waiting.
                if (gnRobotBalance == _DISABLE)                         // Check if the balancing module is enable or not.
                {                                                       // Do not run balancing module.
                    OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.
                }
                else
                {                                                       // Balancing is enabled. 
                    nOmegaWSet = gfOmegaWSet*_NOTICKSROTATION;          // Compute the request linear velocity in integer format, normalized
                                                                        // to the number of ticks per rotation of wheel. This is basically 
                                                                        // expressed as how many pulses (from the motor encoder) expected
                                                                        // in 1 second.  The higher the wheel rotational speed the higher
                                                                        // is the no. of pulses expected in 1 second.
                    
                    nTemp = nCurrentOmegaWSet - nOmegaWSet;             // Determine the difference between required speed and current speed
                                                                        // in integer format.
                    if (nTemp < 0)                                      // Based on the difference, we gradually increase or decrease the 
                    {                                                   // current speed setting to match the required speed.  If the difference
                                                                        // is large, the quantum of increment/decrement will be larger. When  
                                                                        // the difference is small, minimum change of 1 unit is imposed.    
                        nTemp = -nTemp;                                                 
                        nTemp = nTemp >> 3;                             // Compute the quantum of change, by dividing the difference by 8.
                        if (nTemp == 0)
                        {
                            nTemp = 1;                                  // At least 1.
                        }
                        nCurrentOmegaWSet = nCurrentOmegaWSet + nTemp;  // Update current speed setting.
                    }
                    else if (nTemp > 0)
                    {
                        nTemp = nTemp >> 3;                             // Compute the quantum of change, by dividing the difference by 8.
                        if (nTemp == 0)
                        {
                            nTemp = 1;                                  // At least 1.
                        }
                        nCurrentOmegaWSet = nCurrentOmegaWSet - nTemp;  // Update current speed setting.                       
                    }
                    
                    if (nCurrentOmegaWSet > 0)                          // Moving forward.
                    {
                        if (nCurrentOmegaWSet > nMaxOmegaWLimit)        // Cap the maximum speed.
                        {
                            nCurrentOmegaWSet = nMaxOmegaWLimit;
                        }
                        nTemp = nCurrentOmegaWSet;                      // Compute the no. of encoder counts expected in 1 second (e.g. how
                                                                        // many pulses expected in 1 second) at current wheel angular rotation
                                                                        // rate.  The gfOmegaWSet unit is in rotation/second.
                        if (nTemp == 0)                                 // Prevent divide-by-zero math exception.  Also this ensures the routine
                        {                                               // will execute at least once every second.
                            nTemp = 1;
                        }
                        nDelayTick = nSysTick1Second/nTemp;             // Compute the interval in terms of system ticks between consecutive  
                                                                        // encoder pulses.
                                                                        // Adjust the interval.
                        nDistanceError = gnlDistanceSet - gnDistanceMoveW;      // Implement a PD algorithm to determine the interval offset based on
                        nDelayTick = nDelayTick + (gnKp_Move*nDistanceError);   // difference between expected and current distance traveled.
                                                                                // Update Kp term.  At present we do not use Kd term, as experiment on
                                                                                // (30 Sep 2017) V2T1 prototype does not shows significant improvement
                                                                                // on the linear movement dynamics, and instead used up precious machine
                                                                                // cycles.
                        //nDelayTick = nDelayTick + (gnKd_Move*(nDistanceError-nDistanceErrorOld));   // Update Kd term.
                        //nDistanceErrorOld = nDistanceError;             // Update the previous error.
                        if (nDelayTick < __NUM_SYSTEMTICK_MSEC)         // Limit the minimum delay count to not less than 1 msec, e.g. equal
                        {                                               // to the main Balancing Controller interval.            
                            nDelayTick = __NUM_SYSTEMTICK_MSEC;
                        }                                    
                        OSSetTaskContext(ptrTask, 5, 1);                // Next state = 5, timer = 1.
                    }
                    else if (nCurrentOmegaWSet < 0)                      // Moving backward.
                    {
                        if (nCurrentOmegaWSet < -nMaxOmegaWLimit)       // Cap the maximum speed.
                        {
                            nCurrentOmegaWSet = -nMaxOmegaWLimit;
                        }
                        nTemp = nCurrentOmegaWSet;                      // Compute the no. of encoder counts expected in 1 second (e.g. how
                                                                        // many pulses expected in 1 second) at current wheel angular rotation
                                                                        // rate. The gfOmegaWSet unit is in rotation/second.
                        if (nTemp == 0)                                 // Prevent divide-by-zero math exception.
                        {
                            nTemp = -1;
                        }
                        nDelayTick = nSysTick1Second/(-nTemp);          // Compute the interval in terms of system ticks between consecutive  
                                                                        // encoder pulses.
                        nDistanceError = gnDistanceMoveW - gnlDistanceSet;      // Implement a PD algorithm to determine the interval offset based on
                        nDelayTick = nDelayTick + (gnKp_Move*nDistanceError);   // difference between expected and current distance traveled.
                                                                                // Update the Kp term.
                        //nDelayTick = nDelayTick + (gnKd_Move*(nDistanceError-nDistanceErrorOld));   // Update Kd term.
                        //nDistanceErrorOld = nDistanceError;             // Update the previous error.
                        if (nDelayTick < __NUM_SYSTEMTICK_MSEC)         // Limit the minimum delay count to not less than 1 msec, e.g. equal
                        {                                               // to the main Balancing Controller interval.            
                            nDelayTick = __NUM_SYSTEMTICK_MSEC;
                        }                        
                        OSSetTaskContext(ptrTask, 7, 1);                // Next state = 7, timer = 1.                     
                    }
                    else  // nCurrentOmegaWSet = 0                      // Stop, stand still.
                    {
                        nDelayTick = 1;                        
                        OSSetTaskContext(ptrTask, 2, 20*__NUM_SYSTEMTICK_MSEC); // Next state = 2, timer = 20 msec.
                                                                        // The timer needs to be at least 10x the Balancing
                                                                        // algorithm interval.
                    }
                }
            break;
                
            case 5: // State 5 - Move forward sequence, check the delay.
                nDelayTick--;                                           // Decrement interval counter.
                if (nDelayTick == 0)
                {
                    nTemp = gnlDistanceSet - gnDistanceMoveW;           // To prevent harsh movement, we check the
                    if (nTemp < nMaxTickDifference)                     // difference between expected distance traveled
                    {                                                   // and actual difference traveled.  If this is 
                                                                        // within safe limit, then we increase the expected
                                                                        // distance traveled.
                        gnlDistanceSet++;                               // Increment the expected distance.
                    }                                    
                    OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.  
                }
                else
                {
                    gnOmegaW = gfOmegaW*_NOTICKSROTATION;               // Convert current wheel velocity to integer, normalized
                                                                        // to the number of ticks per rotation of wheel. 
                    OSSetTaskContext(ptrTask, 5, 1);                    // Next state = 5, timer = 1.
                }
                break;
                                
            case 7: // State 7 - Move backward sequence, check the delay.
                nDelayTick--;                                           // Decrement interval counter.
                if (nDelayTick == 0)
                {
                    nTemp = gnDistanceMoveW - gnlDistanceSet;           // To prevent harsh movement, we check the
                    if (nTemp < nMaxTickDifference)                     // difference between expected distance traveled
                    {                                                   // and actual difference traveled.  If this is 
                                                                        // within safe limit, then we increase the expected
                                                                        // distance traveled.
                        gnlDistanceSet--;                               // Decrement the expected distance.
                    }
                    OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.                                   
                }                
                else
                {
                    gnOmegaW = gfOmegaW*_NOTICKSROTATION;               // Convert current wheel velocity to integer, normalized
                                                                        // to the number of ticks per rotation of wheel. 
                    OSSetTaskContext(ptrTask, 7, 1);                    // Next state = 7, timer = 1.
                }                
                break;
                                
            case 10: // State 10 - Balancing operation is disabled, continue to monitor the global variable 
                     // gnRobotBalance until balancing routine is enabled again.
                if (gnRobotBalance == _DISABLE)
                {
                    OSSetTaskContext(ptrTask, 10, 1*__NUM_SYSTEMTICK_MSEC);     // Next state = 10, timer = 1 msec.
                }
                else    // back to balancing mode, initialize all internal variables.
                {	
                                       
                    OSSetTaskContext(ptrTask, 0, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 0, timer = 1 msec.
                }
                break;
                
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1);                        // Back to state = 0, timer = 1.
            break;
        }
    }
}

///
/// Function name	: SetMovementStop
///
/// Author          : Fabian Kung
///
/// Last modified	: 27 July 2018
///
/// Code version	: 0.98
///
/// Processor		: Generic
///
/// Description		: Set the robot to stand still from moving.
///
/// Arguments		: none.
///                  
/// Return          : None.
///
/// Global variable	: gfOmegaWSet
///                   gnOmegaW
///                   gnlDistanceSet
///
/// Description     : Call this function whenever we want the robot to stand still.
///                   See design notes. Basically to allow smooth stop dynamics, we add
///                   a small offset to the current position setting to account for
///                   the momentum of the machine.

#define     _SETMOVEMENTSTOP_COEFF  4   // This should be tuned according to robot drive
                                        // characteristics.

void SetMovementStop(void)
{
    // 17 June 2016: By adding a step increment and decrement in set distance, 
    // we can reduce the overshoot and undershoot, e.g. jerking motion when going
    // from moving to standing still.  The step value depends on present velocity
    // , wheel size and motor setup.
    int nTemp;   
    
    if (gfOmegaWSet != 0.0)     // This is to prevent this function being called multiple times,
                                // with an offset in the distance setting being added until the
                                // machine becomes unstable.
    {
        nTemp = gnOmegaW/_SETMOVEMENTSTOP_COEFF;
        gnlDistanceSet = gnlDistanceSet + nTemp;
        gfOmegaWSet = 0.0;        // Stop the robot.
    }
}


///
/// Process name		: Robot_Proce1
///
/// Author              : Fabian Kung
///
/// Last modified		: 25 July 2022
///
/// Code Version		: 1.351
///
/// Processor           : dsPIC33EP256MU80X family.
///
/// Processor/System Resources
/// PINS                : PIN_PSW
///
/// MODULES             : All internal and external modules.
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables    : gnRobotBalance
///                       gnRobotMode
///                       gnTiltOrient_IMU
///                       gnHeadElevationAngleSet_Deg
///                       gnHeadAzimuthAngleSet_Deg
///                       others

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Robot_Proce1: Incompatible OS version"
	#endif
#else
	#error "Robot_Proce1: An RTOS is required with this function"
#endif

/// Description: Robot process 1.  
/// This process is the first layer of the robot processes.  It is like the basic instinct,
/// low level and involuntary movement control in vertebrate animals (e.g. the brain
/// stem and part of the reptilian brain). We can also compare this to the center nervous system
/// (CNS) of insect :P
/// The process main functions:
/// 1. Sets the operating mode of the robot into:
///   Normal mode
///   Test mode 1 - Test the motor driver for left and right wheels and encoders.
///   Test mode 2 - Disable the motor driver for wheels, used for all sensors.
///
/// 2. Set the joint angle and rotation speed of the servo motors (if attached).
///
/// 3. Perform FFT computation on one system parameter, here we chosen the average wheel velocity
///    as the parameter to be transformed into frequency domain.
///
/// 4. Computation of stability indices, i.e. a number of parameters that are used to indicate how
///    stable the robot platform is at any instant.  Here the indices are derived from the result
///    of the FFT. The stability indice indicate if the robot platform is standing still, moving
///    at constant velocity of vibrating violently. 
///
/// 5. Adaptive tuning of state-feedback control coefficients based on robot operating mode or
///    stability indices.
///
/// 6. Adaptive minor adjustment of tilt angle setting to compensate for slow drifting.  This 
///    feature make use of the result of FFT for the velocity, where component XFFT[0] corresponds
///    to the d.c. or average velocity over all the FFT samples.
///
/// 7. Other basic functions that include:
/// a. Initialize the robot to default state upon power up (balancing, movement modes, all
///    servo motors and DC motors etc).
/// b. Monitors the states of the IMU sensors and determines if the robot is toppled or not.
/// c. Check for internal error condition like sensors and wheel encoder registers overflow.
///    Reset the robot if this happen.
/// d. Prevent damage to the robot by shutting down the motor driver circuit if the robot
///    is in the process of toppling over.  Also restore the robot to default stage when 
///    it is upright again.
///
/// The process executes with a periodic interval of 50 msec.  Thus in 1 second the
/// routines in the process will be executed 20 times.
///
/// The high level state of robot is indicated by global variable 'gnRobotMode'.  
/// The change of the mode is as follows:
/// [Power Up] -> [Robot is upright]  <---
///  IDLE          .     |                 |
/// /|\           /|\  INITIALIZE        IDLE
///  |             |     |                 .
///  |     ________|     |                /|\
///  |    |    |         |                 |
///  |   \|/  \|/       \|/                |
///  |    '    '         '                 |
///  |  TEST  TEST     NORMAL ----> [Robot toppled]       
///  |  MODE1 MODE2    /  |                 .   
///  |                /  \|/               /|\   
///  |               /    '             ____|    
///  |              /  [Internal Error]  
///  |            |/_             
///  -- Factory Reset
///                            
/// In 'NORMAL' mode the robot will drive the attached servo motors, check for topple condition
/// and other sensors error,
/// handle message from external controller and adjust the state-feedback coefficients.
///
 
#define     _MAX_DISTANCE_MOVE  2147482648      // (2^31) - 1000, in terms of wheel encoder ticks.
#define     _MIN_DISTANCE_MOVE  -2147482648     // -(2^31) + 1000, in terms of wheel encoder ticks.


#define     _STAB_INDEX1_THRESHOLD_HIGH 60      // Upper and lower threshold level for stability index 1.
#define     _STAB_INDEX1_THRESHOLD_LOW  50      // with approximately 20% hysterisis.
#define     _STAB_INDEX0_THRESHOLD_HIGH 20     // Upper and lower threshold level for stability index 0.
#define     _STAB_INDEX0_THRESHOLD_LOW  16     // with approximately 20% hysterisis.
#define     _STAB_INDEX2_THRESHOLD_HIGH 380     // Upper and lower threshold level for stability index 2.
#define     _STAB_INDEX2_THRESHOLD_LOW  305     // With approximately 20% hysterisis.

#define     _BATT_DETECTOR_COEFFICIENT  4       // The voltage divider ratio in the battery monitoring network.
                                                // Corresponds to 10k in series with 3.3k.
//#define     _BATT_VOLT_LOW_THRESHOLD_mV 4800    // For 4x AAA cells battery pack.
                                                // Across a 10k and 3.3k voltage divider network.  This 
                                                // corresponds to input voltage of 5.95 volts.
#define     _BATT_VOLT_LOW_THRESHOLD_mV 7160    // For 2S 7.4V LiPo battery pack.  

// Note: The formula to workout the temperature in degree Celsuis from the temperature sensor
// output in mV is:
// Tout(Celcius) = (Vout(mV)/Temperature_Coefficient) + Offset
#ifdef __TEMPERATURE_SENSOR_AD22103__
    #define     _TEMP_SENSOR_COEFFICIENT   28      // Coefficient in mV/degree Celcius (for AD22103K). 
    #define     _TEMP_SENSOR_OFFSET        9       // Offset in degree Celcius (for AD22103K))
#else 
    #ifdef __TEMPERATURE_SENSOR_LM35__
        #define     _TEMP_SENSOR_COEFFICIENT   10      // Coefficient in mV/degree Celcius (for LM35). 
        #define     _TEMP_SENSOR_OFFSET        0       // Offset in degree Celcius (for LM35))
    #else
        #warning "Robot_Proce1: No temperature sensor define for this build"
    #endif
#endif

inline int nGetTemperature(void)
{
    return ((gobjDriverADC.unADC2_mV)/_TEMP_SENSOR_COEFFICIENT) + _TEMP_SENSOR_OFFSET;
}   

  void Robot_Proce1(TASK_ATTRIBUTE *ptrTask)
{
    int nAngleElevation;
    int nAngleAzimuth;
    float fTemp1, fTemp2;
    static int nCollisionCount = 0;
    
    static int nStab0Threshold = _STAB_INDEX0_THRESHOLD_HIGH;
    static int nStab1Threshold = _STAB_INDEX1_THRESHOLD_HIGH;
    static int nStab2Threshold = _STAB_INDEX2_THRESHOLD_HIGH;
    // Variables used in DFT or FFT routines.
    static  int     nIndex;
    static  int     nwk = 0;
    static  int     nwj = 0;
    
    static  float   fXReal;
    static  float   fXImag;
    static  float   fTemp;
    static  float   fx[__FFT_N];
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization of on-board modules, drivers and global states.              
                // Set the robot drivers to a known state:
                PIN_PSW = _OFF_ANALOG_POWER;            // Turn off Analog Power Switch.
                PIN_PSW = _OFF_ANALOG_POWER;            // Repeat, as this pin drive a large capacitive
                                                        // load, and the initial instruction may not execute
                                                        // properly.
                gobjDriverADC.unEnADC = 1;              // Enable internal 12-bits ADC module.
                gobjDriverDAC.unEnDAC = 1;              // Enable external 12-bits DAC module.
                gobjDriverPWM.NumChannel = 4;           // Set PWM servo driver output to 4 channels.
                gobjDriverPWM.unEnPWM = 1;              // Enable the PWM servo driver.
                gobjDriverQuadEncoder.unGearRatio = _MOTOR_GEAR_RATIO;       // Set DC motor gearbox ratio.
                gobjDriverQuadEncoder.unEncoder_CPR = _MOTOR_ENCODER_CPR;    // Set DC motor encoder count per rotation (CPR).
                gobjDriverQuadEncoder.unEncoder_Div = _MOTOR_QUAD_DIVIDE;    // Set the quadrature encoder division ratio.
                                                        // This is an integer value that divides the actual
                                                        // quadrature encoder output, otherwise the quadrature
                                                        // encoder output pulses maybe to frequent and fast 
                                                        // for the software to handle.
                gobjDriverQuadEncoder.nQuadEncoderStatus = _QUADENCODER_RESET; // Start the quadrature encoder.
                
                gnRobotBalance = _DISABLE;              // Disable balancing routines.
                gnRobotMode = _ROBOT_MODE_IDLE;         // Set robot's default operation mode.
                
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 1000 msec.
                //OSSetTaskContext(ptrTask, 200, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 200, timer = 1000 msec. 
            break;

            case 1: // State 1 - Wait until IMU and quadrature encoder are ready, initialize all global variables.
                    // Note: 17 Feb 2016, upon power up the data from the IMU sensors and quadrature encoder is not valid.
                    // Thus we need to make sure the modules are ready before using the data.  This also applies to most of
                    // the global variables, thus this is the best state to initialize all system/global variables.
                if ((gnTiltOrient_IMU == _ROR_UPRIGHT)&&(gnStatus_IMU == _READY)&&(gobjDriverQuadEncoder.nQuadEncoderStatus == _QUADENDCODER_READY))
                    // Check if robot is upright, with IMU and wheel encoder outputs are valid.
                {
                    SetSoundPattern(gunSoundUpright,2);                     // Generate a string of tones to indicate
                    gunAudioVolume = 5;                                     // robot is upright.                    
                    gnRobotMode = _ROBOT_MODE_NORMAL;                       // Clear high level modes.                                                                            
                    gnRobotBalance = _ENABLE;                               // Engage balancing routines.
                    
                    gfRoffset = 0.0;                                        // Clear all motion settings.
                    gfOmegaWSet = 0.0;                                      // Present linear velocity and
                    gnlDistanceSet = 0;                                     // distance, and heading settings.
                    gnHeadingSet = 0;
                    gnTurnSpeed = 0;
                    
                    gnHeadElevationAngleSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT;  
                    gnHeadAzimuthAngleSet_Deg = _HEAD_AZIMUTH_ANGLE_DEFAULT;    
                    OSSetTaskContext(ptrTask, 3, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 3, timer = 100 msec.
                }
                else
                {
                   // Check robot operating mode and enable/disable balancing routines accordingly.
                   if (gnRobotMode == _ROBOT_MODE_TEST1)                        // Test Mode 1.  Test motor driver and wheel encoder.
                   {
                       gnRobotBalance = _DISABLE;                               // Disable self-balancing module.
                       PIN_PSW = _ON_ANALOG_POWER;                              // Turn on high voltage analog supply.
                       nPrintln("Engaging Test Mode 1",20);
                       OSSetTaskContext(ptrTask, 110, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 110, timer = 100 msec.
                   }
                   else if (gnRobotMode == _ROBOT_MODE_TEST2)                   // Test Mode 2.  Off motor driver.
                   {
                       gnRobotBalance = _DISABLE;
                       nPrintln("Engaging Test Mode 2",20);
                       OSSetTaskContext(ptrTask, 120, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 120, timer = 100 msec.
                   }     
                   else if (gnRobotMode == _ROBOT_MODE_TEST3)                   // Test Mode 32.  Off motor driver.
                   {
                       gnRobotBalance = _DISABLE;
                       PIN_PSW = _ON_ANALOG_POWER;                              // Turn on high voltage analog supply.
                       nPrintln("Engaging Test Mode 3",20);
                       OSSetTaskContext(ptrTask, 130, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 130, timer = 100 msec.
                   }                        
                   else
                   {                
                       OSSetTaskContext(ptrTask, 2, 1);                         // Next state = 2, timer = 1.
                   }                    
                }
            break;

            case 2: // State 2 - Update battery level, board temperature.
                gobjRobotState.nBatteryLevel_mV = (gobjDriverADC.unADC3_mV)*_BATT_DETECTOR_COEFFICIENT;
                gobjRobotState.nTemperature_C = nGetTemperature();
                OSSetTaskContext(ptrTask, 1, 10*__NUM_SYSTEMTICK_MSEC);         // Next state = 1, timer = 10 msec.
                break;
            
            case 3: // State 3 - Idle.
                OSSetTaskContext(ptrTask, 4, 1);                                // Next state = 4, timer = 1.
                break;
                
            case 4: // State 4 - Idle.         
                OSSetTaskContext(ptrTask, 5, 1);                                // Next state = 5, timer = 1.
                break;   
             
            case 5: // State 5 - Turn on analog power supply snd broadcast the firmware version to remote monitor 
                    // program (if connected). 
                    // Initialize the UART receive buffer 2.
                    // Note 22 Dec 2016: UART2 is connected to the head unit.  The head unit contains multiple sensors
                    // module, for instance the machine vision module (MVM).  When the head unit is powered by the analog 
                    // power supply, and when it is turned on, the UART module on the MVM may output some garbage data.  
                    // So we need clear UART2 input buffer after turning on the MVM.
                PIN_PSW = _ON_ANALOG_POWER;                                     // Turn on high voltage analog supply.    
                nPrintln(gbytFirmwareVersion, 5);                   
                gSCIstatus2.bRXRDY = 0;                                         // Reset valid data flag.
                gbytRXbufptr2 = 0;                                              // Reset pointer.
                OSSetTaskContext(ptrTask, 6, 50*__NUM_SYSTEMTICK_MSEC);         // Next state = 6, timer = 50 msec.
                break;
                
            case 6: // State 6 - Normal mode, continue to monitor robot condition, check if robot is toppled.
                    // Else check for robot operating mode, and drive the elevation and azimuth motors 
                    // of the robot head unit.
                /*
                if (_RF0 == 0)      // Test codes to generate a square pulse at pin RF0, for measuring
                {                   // the sampling rate of the FFT routine.  Comment this out if not used.
                    _RF0 = 1;
                }
                else
                {
                    _RF0 = 0;
                } 
                */
                
                if ((gnTiltOrient_IMU == _ROR_TOPPLE_FRONT) || (gnTiltOrient_IMU == _ROR_TOPPLE_BACK))
                {
                    SetSoundPattern(gunSoundToppled,2);                 // Generate a string of tones to indicate
                    gunAudioVolume = 5;                                 // Robot is toppled.
                    OSSetTaskContext(ptrTask, 50, 1);                   // Next state = 50, timer = 1.
                }
                else if (gnRobotMode == _ROBOT_MODE_IDLE)
                {
                    OSSetTaskContext(ptrTask, 0, 1);                    // Next state = 0, timer = 1.
                }
                else    // Robot is upright, proceed to set head unit servo motors and 
                        // check operating mode.
                {                                             
                   nAngleElevation =  gnHeadElevationAngleSet_Deg;                   
                   nAngleAzimuth = gnHeadAzimuthAngleSet_Deg;                 
                         
                   nSetRCServoMotor(nAngleElevation,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO);     // Drive the elevation motor.                    
                   nSetRCServoMotor(nAngleAzimuth,_MOTORSPEED_FAST,_HEAD_AZIMUTH_SERVO);     // Drive the azimuth motor.                     
                                       
                   // Check robot operating mode and enable/disable balancing routines accordingly.
                   if (gnRobotMode == _ROBOT_MODE_NORMAL)               // Normal mode.
                   {
                       gnRobotBalance = _ENABLE;                        
                       OSSetTaskContext(ptrTask, 7, 1);                 // Next state = 7, timer = 1.
                   }
                   else if (gnRobotMode == _ROBOT_MODE_TEST1)           // Test Mode 1.  Test motor driver and wheel encoder.
                   {                                                    // Turn wheels forward, then backward and repeat.
                       gnRobotBalance = _DISABLE;
                       OSSetTaskContext(ptrTask, 110, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 110, timer = 100 msec.
                   }
                   else if (gnRobotMode == _ROBOT_MODE_TEST2)           // Test Mode 2.  Off motor driver.
                   {
                       gnRobotBalance = _DISABLE;
                       OSSetTaskContext(ptrTask, 120, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 120, timer = 100 msec.
                   }
                    else if (gnRobotMode == _ROBOT_MODE_TEST3)           // Test Mode 3.  Off motor driver.
                   {
                       gnRobotBalance = _DISABLE;
                       OSSetTaskContext(ptrTask, 130, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 130, timer = 100 msec.
                   }                   
                   else if (gnRobotMode == _ROBOT_MODE_ERROR_BATTERY)    // Error.  
                   {
                       nPrintln("Battery Low",11);
                       OSSetTaskContext(ptrTask, 7, 100*__NUM_SYSTEMTICK_MSEC);   // Next state = 7, timer = 100 msec.
                   }                   
                   else                                                 // Factory reset or other error modes.
                   {
                       OSSetTaskContext(ptrTask, 0, 1);                 // Next state = 0, timer = 1.
                   }
                }                
                break;
                
            case 7: // State 7 - Check system status:
                // 1. Battery voltage level, check for under voltage. 
                // 2. Motor driver overload condition. Here we defined overload when either left or right wheels turn
                // beyond certain speed, which is 6.0 rev/sec in this case.
                // 3. Update current heading in degree (with respect to global x-axis) from
                // current heading register kept by Quadrature Encoder driver in ticks. The value
                // of current heading in degree is used by high-level behavioral routines.
#define     _OMEGA_WHEEL_OVERLOAD       5.5                             // Overload limit
                
                // Check motor overload condition.
                fTemp1 = gfOmegaRW;
                if (fTemp1 < 0.0)       // Make sure it's positive.
                {
                    fTemp1 = -fTemp1;
                }
                fTemp2 = gfOmegaLW;
                if (fTemp2 < 0.0)       // Make sure it's positive
                {
                    fTemp2 = -fTemp2;
                }                
                if ((fTemp1 > _OMEGA_WHEEL_OVERLOAD) || (fTemp2 > _OMEGA_WHEEL_OVERLOAD))   // If motor driver driver overload condition
                {                                                       // is detected execute same routines as toppled.
                                                                        // The limit is 6.0 revolution/sec.
                    SetSoundPattern(gunSoundToppled,2);                 // Generate a string of tones to indicate                
                    gunAudioVolume = 5;                                 // Robot is toppled.
                    OSSetTaskContext(ptrTask, 50, 1);                   // Next state = 50, timer = 1.
                    break;                    
                }
                   
                // Calculate current heading of the robot in degree and with respect to the global x-axis.
                fTemp = gnHeading*_TURN_DEG_PER_UNIT;
                gnCurrentHeadingDeg = fTemp + 90;                       // Approximate to integer, add 90 deg offset as 
                                                                        // angle is measured with respect to x-axis.
                while (gnCurrentHeadingDeg > 360)
                {
                    gnCurrentHeadingDeg = gnCurrentHeadingDeg - 360;
                }
                
                // Check for under-voltage.  In future version this threshold should be adjusted based on
                // ambient temperature.  Where we can add an analog temperature sensor to the idle ADC channel.
                // For now we will use the typical discharge curve for LiPo battery at 23-25 degrees celcius.
                // Here we will fixed the cut-off at 3.58 Volts per cell.  Most battery manufacturer sets a 
                // limits between 2.8-3.2 Volts per cell, so we a more conservative here.
                gobjRobotState.nBatteryLevel_mV = (gobjDriverADC.unADC3_mV)*_BATT_DETECTOR_COEFFICIENT;   // Update battery level.
                if (gobjRobotState.nBatteryLevel_mV < _BATT_VOLT_LOW_THRESHOLD_mV)    // Check for battery low threshold.    
                {
                    #ifndef __DEBUG__
                    SetSoundPattern(gunSoundBatteryLow,4);              // Sound an alarm if battery level is too low.   
                    gnRobotMode = _ROBOT_MODE_ERROR_BATTERY;            // Indicate error condition, battery voltage low.    
                    OSSetTaskContext(ptrTask, 8, 20*__NUM_SYSTEMTICK_MSEC);     // Next state = 8, timer = 20 msec.
                    #else
                    OSSetTaskContext(ptrTask, 8, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 8, timer = 1 msec.
                    #endif
                }   
                else
                {
                    if (gnRobotMode = _ROBOT_MODE_ERROR_BATTERY)            // Check if previously battery voltage low error.
                    {
                        gnRobotMode = _ROBOT_MODE_NORMAL;                   // Return robot mode back to normal.    
                    }
                    OSSetTaskContext(ptrTask, 8, 1);                        // Next state = 8, timer = 1.
                }
                break;
                
            case 8: // State 8 - Check if quadrature encoder step counter overflow.  Also update IR distance sensor register.
                    // The step counter gnDistanceMoveW is 32-bits signed integer.  As this parameter 
                    // is used by the Balance Controller, overflow may leads to the Balance Controller
                    // malfunction, causing the robot to topple.  Thus before reaching overflow state,
                    // we need to stop the robot, after the robot regain upright position, reset the
                    // quadrature encoder.
                //2^31 - 1000 = 2147482648
                // -(2^31)+1000 = -2147482648
                if (gnDistanceMoveW > 0)                            // If move forward.
                {
                    if (gnDistanceMoveW > _MAX_DISTANCE_MOVE)       // Check if distance step counter is near overflow limit.
                    {
                        gfOmegaWSet = 0.0;                          // Stop the robot if it is moving.
                                                                    // Wait for sufficiently long period for the robot to stop.
                        gnAudioTone[0] = 1;                         // Generate a string of tones to indicate internal error.
                        gnAudioTone[1] = 2;                         
                        gnAudioTone[2] = 3;
                        gnAudioTone[3] = 4;
                        gnAudioTone[4] = 5;
                        gnAudioTone[5] = 0;                           
                        OSSetTaskContext(ptrTask, 9, 2000*__NUM_SYSTEMTICK_MSEC);  // Next state = 9, timer = 2000 msec.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 11, 1);  // Next state = 11, timer = 1.                        
                    }
                }
                else                                                // Else if move backward.
                {
                    if (gnDistanceMoveW < _MIN_DISTANCE_MOVE)       // Check if distance step counter is near overflow limit.
                    {
                        gfOmegaWSet = 0.0;                          // Stop the robot if it is moving.
                                                                    // Wait for sufficiently long period for the robot to stop.
                        gnAudioTone[0] = 1;                         // Generate a string of tones to indicate internal error.
                        gnAudioTone[1] = 2;                         
                        gnAudioTone[2] = 3;
                        gnAudioTone[3] = 4;
                        gnAudioTone[4] = 5;
                        gnAudioTone[5] = 0;    
                        OSSetTaskContext(ptrTask, 9, 2000*__NUM_SYSTEMTICK_MSEC);  // Next state = 9, timer = 2000 msec.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 11, 1);  // Next state = 11, timer = 1.
                    }                    
                }
                break;         
                
            case 9: // State 9 - Quadrature encoder step counter overflow, sequence 1.
                gobjDriverQuadEncoder.nQuadEncoderStatus = _QUADENCODER_RESET;  // Restart the quadrature encoder.
                OSSetTaskContext(ptrTask, 10, 10*__NUM_SYSTEMTICK_MSEC);        // Next state = 10, timer = 10 msec.
                break;
                
            case 10: // State 10 - Quadrature encoder step counter overflow, sequence 2. Restart.
                gnlDistanceSet = 0;                                             // distance, and heading settings.
                gnHeadingSet = 0;                
                OSSetTaskContext(ptrTask, 1, 1*__NUM_SYSTEMTICK_MSEC);          // Next state = 1, timer = 1 msec.
                break;
                
            case 11: // State 11 - Calculate robot upright stability index and determine the correct feedback
                    // coefficients to use.
                gnStabilityIndexH = gunXmag[4] + gunXmag[5] + gunXmag[6] + gunXmag[7]; // Stability index that comprises of higher order
                                                                                // harmonics is useful for indication of violent
                                                                                // vibration, when the platform is at a risk of toppling
                                                                                // over.
                
                gnStabilityIndexL = gunXmag[1] + gunXmag[2];                    // Stability index that comprises of lower order
                                                                                // harmonics is useful for indication of slow rocking
                                                                                // movement.
                if (gnTurnMode == _TURN_MODE_NONE)              // Only execute adaptive state-feedback coefficient
                {                                               // adjustment if the robot is not in active turning mode.
                    if (gfOmegaWSet == 0.0)                     // Adjustment algorithm if the robot is stationary.
                    {   
                        if ((gunXmag[0] > nStab0Threshold) || (gnStabilityIndexL > nStab2Threshold))    
                        {
                            nStab0Threshold = _STAB_INDEX0_THRESHOLD_LOW;       
                            nStab1Threshold = _STAB_INDEX1_THRESHOLD_HIGH;
                            nStab2Threshold = _STAB_INDEX2_THRESHOLD_LOW;
                            OSSetTaskContext(ptrTask, 12, 1);                   // Next state = 12 (transition to stiff), timer = 1
                        }
                        else if ((gnStabilityIndexH > nStab1Threshold))          // If stability index is above limit.    
                        {                                                       // Change feedback coefficients to soft.    
                            nStab0Threshold = _STAB_INDEX0_THRESHOLD_HIGH;
                            nStab1Threshold = _STAB_INDEX1_THRESHOLD_LOW;
                            nStab2Threshold = _STAB_INDEX2_THRESHOLD_HIGH;                            
                            OSSetTaskContext(ptrTask, 16, 1);                   // Next state = 16 (transition to soft), timer = 1
                        }                         
                        else                                                    // Change feedback coefficients to normal.
                        {   
                            nStab0Threshold = _STAB_INDEX0_THRESHOLD_HIGH;
                            nStab1Threshold = _STAB_INDEX1_THRESHOLD_HIGH;
                            nStab2Threshold = _STAB_INDEX2_THRESHOLD_HIGH;
                            OSSetTaskContext(ptrTask, 14, 1);                   // Next state = 14 (transition to normal), timer = 1
                        }                        
                    }
                    else                                                        // If robot is instructed to move linearly at
                                                                                // constant velocity.
                    {      
                        OSSetTaskContext(ptrTask, 12, 1);                       // Next state = 12 (transition to stiff), timer = 1
                    }
                }       
                else                                                            // Change feedback coefficients to stiff.
                {
                    OSSetTaskContext(ptrTask, 12, 1);                           // Next state = 12, timer = 1. 
                }
                break;
                
            case 12: // State 12 - Transition FB control coefficients to stiff.
                gnFBCTuneLevel = 40;
                #ifndef __DEBUG__
                if (gfP < _FP_STIFF)                    // Value is -4.50, thus values smaller than this 
                {                                       // should be incremented.
                    gfP = gfP + _FP_INTERVAL;
                }
                else if (gfP > _FP_STIFF)
                {
                    gfP = gfP - _FP_INTERVAL;
                }
                 
                if (gfCwg < _FCW_STIFF)
                {
                    gfCwg = gfCwg + _FCW_INTERVAL;
                }
                else if (gfCwg > _FCW_STIFF)
                {
                    gfCwg = gfCwg - _FCW_INTERVAL;
                }
                #endif                  
                OSSetTaskContext(ptrTask, 13, 1);                   // Next state = 13, timer = 1.                 
                break;
                
            case 13: // State 13 - Continue transition FB control coefficients to stiff.
                #ifndef __DEBUG__                            
                if (gfKomega < _FKV_STIFF * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega + (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }
                else if (gfKomega > _FKV_STIFF * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega - (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }
                
                if (gfKnw < _FKX_STIFF * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw + (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }
                else if (gfKnw > _FKX_STIFF * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw - (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }
                #endif                        
                OSSetTaskContext(ptrTask, 18, 1);                   // Next state = 18, timer = 1.
                break;
                
            case 14: // State 14 - Transition FB control coefficients to normal.
                gnFBCTuneLevel = 20;
                #ifndef __DEBUG__
                if (gfP < objCoeffDef[0].fDefault)
                {
                    gfP = gfP + _FP_INTERVAL;
                }                
                else if (gfP > objCoeffDef[0].fDefault)
                {
                    gfP = gfP - _FP_INTERVAL;
                }
                if (gfCwg < objCoeffDef[3].fDefault)
                {
                    gfCwg = gfCwg + _FCW_INTERVAL;
                }
                else if (gfCwg > objCoeffDef[3].fDefault)
                {
                    gfCwg = gfCwg - _FCW_INTERVAL;
                }                
                #endif 
                OSSetTaskContext(ptrTask, 15, 1);                   // Next state = 15, timer = 1.
                break;                    

            case 15: // State 15 - Continue transition FB control coefficients to normal.
                #ifndef __DEBUG__
                if (gfKomega < gfKV_Ori * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega + (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }
                else if (gfKomega > gfKV_Ori * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega - (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }                
                
                if (gfKnw < gfKX_Ori * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw + (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }
                else if (gfKnw > gfKX_Ori * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw - (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }                
                #endif                 
                OSSetTaskContext(ptrTask, 18, 1);                   // Next state = 18, timer = 1.
                break;              

            case 16: // State 16 - Transition FB control coefficients to soft.
                gnFBCTuneLevel = 0;
                #ifndef __DEBUG__
                if (gfP < _FP_SOFT)
                {
                    gfP = gfP + _FP_INTERVAL;
                }                
                else if (gfP > _FP_SOFT)
                {
                    gfP = gfP - _FP_INTERVAL;
                }
                if (gfCwg < _FCW_SOFT)
                {
                    gfCwg = gfCwg + _FCW_INTERVAL;
                }
                else if (gfCwg > _FCW_SOFT)
                {
                    gfCwg = gfCwg - _FCW_INTERVAL;
                }                
                #endif 
                OSSetTaskContext(ptrTask, 17, 1);                   // Next state = 17, timer = 1.
                break;                    

            case 17: // State 17 - Continue transition FB control coefficients to soft.
                #ifndef __DEBUG__
                if (gfKomega < _FKV_SOFT * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega + (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }
                else if (gfKomega > _FKV_SOFT * _KVTOKOMEGA_COEFF)
                {
                    gfKomega = gfKomega - (_FKV_INTERVAL * _KVTOKOMEGA_COEFF);
                }                
                
                if (gfKnw < _FKX_SOFT * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw + (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }
                else if (gfKnw > _FKX_SOFT * _KXTOKNW_COEFF)
                {
                    gfKnw = gfKnw - (_FKX_INTERVAL * _KXTOKNW_COEFF);
                }                
                #endif                 
                OSSetTaskContext(ptrTask, 18, 1);                   // Next state = 18, timer = 1.
                break; 
                
            case 18: // State 18 - Check for collision signature based on stability indices, and set the collision flag.
                // Also check if the robot platform is moving and set the move flag.
                // 30 Oct 2019 - Note that these limits are for typical speed of SLOW and NORMAL.  For faster linear
                // speed settings we should increase the limits.
                if ((gnStabilityIndexH > 50) || (gnStabilityIndexL > 100)) 
                {
                    nCollisionCount++;                              // Increment the collision counter and check if
                    if (nCollisionCount > 3)                        // it breach the threshold value.
                    {
                        gobjRobotFlag.bCOLLISION = 1;               // Once this flag is set, it will only be cleared
                                                                    // when External Controller read it's value.                        
                        if (nCollisionCount > 6)
                        {
                            nCollisionCount = 6;                    // Limit the max value of the collision counter.
                        }
                    }
                }                
                else
                {
                    if (nCollisionCount > 0)
                    {
                        nCollisionCount--;
                    }
                }
                if (gunXmag[0] > _STAB_INDEX0_THRESHOLD_LOW)        // The 'DC' component of the FFT output is the average
                                                                    // velocity of the robot platform.  We arbitrarily set
                                                                    // the threshold at _STAB_INDEX0_THRESHOLD_LOW.
                {
                    gobjRobotFlag.bMOVE = 1;                        // Once this flag is set, it will only be cleared
                                                                    // when External Controller read it's value.                            
                }
                OSSetTaskContext(ptrTask, 19, 1);                   // Next state = 19, timer = 1.
                break;
                

                
            case 19: // State 19 - 1. Get board temperature in degree and output of distance sensor.
                     // 2. Estimate the temperature from precision temperature sensor, LM35DZ.
                     // 3. Perform small adjustment to the tilt angle setting to compensate for slow drifting either
                     // standing still or moving at constant velocity. 
                
                gobjRobotState.nTemperature_C = nGetTemperature(); // Get board temperature.
                
                #ifndef __DEBUG__
                // Adaptive minor tilt angle adjust. This make use of the output of FFT for velocity, where
                // component gfXR[0] corresponds to the average drift velocity over all time domain samples.
                // This adjustment should be done at long interval, which is 50 msec here.
                fTemp1 = gfXR[0] - gfOmegaWSet;
                if (fTemp1 > 0.0)                           // If robot platform drift forward slowly, decrease the
                {                                           // tilt angle setting slightly.
                    gfR = gfR_Ori - _R_INTERVAL;
                }
                else if (fTemp1 < 0.0)                     // If robot platform drift back slowly, increase the
                {                                           // tilt angle setting slightly.
                    gfR = gfR_Ori + _R_INTERVAL;
                }  
                #endif
                OSSetTaskContext(ptrTask, 20, 2);                   // Next state = 20, timer = 2.
                break;
                
            case 20: // State 20 - Compute DFT using the formula
                // X(k) = Sigma[x(n)cos(2.pi.k.n/N)] + jSigma[x(n)sin(2.pi.k.n/N)]
                // FFT sequence: Shift and sample time-domain parameters.
                // In order to keep this process within the time limit of 1 processor tick, we split it
                // into two parts. Each part shift half of the total time-domain samples and should be
                // restricted to no more than 20 samples. Thus this FFT routine can accept a total
                // of 40 samples, or 2.0 seconds.
                for (nIndex = __FFT_N-1; nIndex > __FFT_Ndiv2; nIndex--)
                {
                    fx[nIndex] = fx[nIndex-1];
                }
                /*
                fx[0] = 1.0;                  // Inject test samples to test the algorithm.
                fx[2] = 1.0;
                fx[4] = 1.0;
                fx[8] = 1.0;       
                fx[10] = 0.5;
                fx[13] = 0.4;
                fx[17] = 0.8;
                fx[21] = 0.4;
                fx[23] = 0.6;
                */
                OSSetTaskContext(ptrTask, 21, 1);               // Next state = 21, timer = 1.
                break;
                
            case 21: // State 21 - FFT sequence: Shift and sample time-domain parameters, part 2.
                for (nIndex = __FFT_Ndiv2; nIndex > 0; nIndex--)
                {
                    fx[nIndex] = fx[nIndex-1];
                }
                fx[0] = gfOmegaW_LP;                            // Sample new value, low-passed version. Need to use low-passed
                                                                // version to prevent aliasing.
                fXReal = fx[0];                                 // Prepare to begin DFT computation. For real part the 1st
                                                                // column of DFT coefficients is always by 1.0, while for
                                                                // imaginary part the 1st column is always 0.0.  See the DFT
                                                                // coefficients in "FFT.h", this is a characteristics of the
                                                                // DFT coefficients.
                                                                //  cos(2*pi*nwi*0/N) = 1 and sin(2*pi*nwi*0/N) = 0    
                                                                // So we make use of this property to improve 
                                                                // the computation efficiency.
                fXImag = 0.0;                                   // 
                nwj = 1;                                        // Start with time index = 1 for column, as per above reason. 
                nwk = 0;                                        // Frequency index.
                OSSetTaskContext(ptrTask, 22, 1);               // Next state = 22, timer = 1.
                break;
                
            case 22: // State 22 - Matrix multiplication for real and imaginary matrice elements.               
                fXReal = fXReal + gfFFTCoeffReal[nwk][nwj]*fx[nwj];
                fXImag = fXImag + gfFFTCoeffImag[nwk][nwj]*fx[nwj];
                nwj++;
                if (nwj == __FFT_N)
                {
                    gfXR[nwk] = fXReal;                         // Assign real part of DFT result to global result.
                    gfXI[nwk] = fXImag;                         // Assign imaginary part of DFT result to global result
                    nwk++;                                      // Next frequency index.
                    if (nwk == __FFT_Ndiv2)                     // Check if half of the frequency index are computed.
                                                                // Note: Due to the symmetry property of the DFT output
                    {                                           // we only need to compute half of the spectra.      
                        nwk = 0;                                
                        OSSetTaskContext(ptrTask, 23, 1);       // Next state = 23, timer = 1.
                    }
                    else
                    {
                        fXReal = fx[0];                         // Next row.  Again as per reason above (1st column of of real
                                                                // DFT coefficients is always 1.0 while 1st column of imaginary
                                                                // DFT coefficients is always 0.0.
                        fXImag = 0.0;                        
                        nwj = 1;                                // Reset column index to 1.
                        OSSetTaskContext(ptrTask, 22, 1);       // Next state = 22, timer = 1.
                    }
                }
                else
                {
                    OSSetTaskContext(ptrTask, 22, 1);           // Next state = 22, timer = 1.
                }
                break;    
                
            case 23: // State 23 - Compute the magnitude squared of each frequency components.
                fTemp = gfXR[nwk]*gfXR[nwk];
                gfXmagsq[nwk] = fTemp + (gfXI[nwk]*gfXI[nwk]);  // Compute magnitude squared for each frequency index.
                gunXmag[nwk] = gfXmagsq[nwk];                   // Convert magnitude squared to unsigned integer.
                nwk++;                                          // Goto next component.
                if (nwk == __FFT_Ndiv2)                         // Check if half of the frequency index are computed.
                                                                // Note: Due to the symmetry property of the DFT output
                {                                               // we only need to compute half of the spectra.                                 
                    OSSetTaskContext(ptrTask, 6, 1);            // Next state = 6, timer = 1. For N = 24.
                                                                // This will ensure a FFT sampling rate
                }                                               // of 20 Hz.
                else
                {
                    OSSetTaskContext(ptrTask, 23, 1);           // Next state = 23, timer = 1.
                }
                break;         
                
            case 50: // State 21 - Disable all activities. Robot toppled.
                     // When the robot topples, we need to shut down all servo motors to avoid damaging the motor's gearbox.
                gnRobotBalance = _DISABLE;                          // Set the robot to a known state:
                gnRobotMode = _ROBOT_MODE_IDLE;                     // Disable balancing, Idle, and Stop.          
                nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_HEAD_ELEVATION_SERVO);     // Turn off all motors.
                nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_HEAD_AZIMUTH_SERVO);   
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_SHOULDER_MOTOR_RIGHT);
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_ARM_MOTOR_RIGHT);
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_ELBOW_MOTOR_RIGHT);
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_SHOULDER_MOTOR_LEFT);
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_ARM_MOTOR_LEFT);
                //nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_ELBOW_MOTOR_LEFT);
                OSSetTaskContext(ptrTask, 51, 1);    // Next state = 51, timer = 1.
            break;
   
            case 51: // State 51 - Toppled, a short delay, and restore state-feedback coefficients
                     // to default values.
                #ifndef __DEBUG__
                gfP = objCoeffDef[0].fDefault;         
                gfCwg = objCoeffDef[3].fDefault;        
                gfKomega = gfKV_Ori * _KVTOKOMEGA_COEFF;
                gfKnw = gfKX_Ori * _KXTOKNW_COEFF;          
                #endif               
                OSSetTaskContext(ptrTask, 52, 1*__NUM_SYSTEMTICK_MSEC);         // Next state = 52, timer = 1 msec.
                break;               
                
            case 52: // State 52 - Toppled, initialize FFT registers, part 1.     
                for (nIndex = 0; nIndex < __FFT_N; nIndex++)
                {
                    fx[nIndex] = 0.0;
                    gfXR[nIndex] = 0.0;
                }                
                OSSetTaskContext(ptrTask, 53, 1);                               // Next state = 53, timer = 1.
            break;
            
            case 53: // State 53 - Toppled, initialize FFT registers, part 2.
                for (nIndex = 0; nIndex < __FFT_N; nIndex++)
                {                   
                    gunXmag[nIndex] = 0;
                    gfXI[nIndex] = 0.0;
                }                
                OSSetTaskContext(ptrTask, 54, 1);                                // Next state = 54, timer = 1.
                break;
                
            case 54: // State 54 - Toppled, initialize FFT registers, part 3.
                for (nIndex = 0; nIndex < __FFT_N; nIndex++)
                {                   
                    gfXmagsq[nIndex] = 0.0;
                }              
                gnStabilityIndexH = 0;                                           // Reset robot platform stability indice 
                gnStabilityIndexL = 0;                                          // and their threshold levels.
                nStab0Threshold = _STAB_INDEX0_THRESHOLD_HIGH;
                nStab1Threshold = _STAB_INDEX1_THRESHOLD_HIGH;
                nStab2Threshold = _STAB_INDEX2_THRESHOLD_HIGH;                  
                OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC);       // Next state = 1, timer = 1000 msec.
                break;   

            // --- Robot Test Mode routines ---
            case 110: // State 110 - Test Mode 1, ignore all sensors, rotate the wheels forward and then backward.
                nPrintln("Engaging Test Mode 1",20);
                SetRWheelProperty(_REVERSE, -1.0);                          // Set wheel direction.
                SetLWheelProperty(_REVERSE, -1.0);             
                gobjDriverDAC.unDAC1_mV = 600;
                gobjDriverDAC.unDAC2_mV = 600;                
                OSSetTaskContext(ptrTask, 111, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 111, timer = 1000 msec.
                //OSSetTaskContext(ptrTask, 115, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 115, timer = 1000 msec.
            break;
            
            case 111: // State 111 - Test Mode 1 continues.  
                gobjDriverDAC.unDAC1_mV = 1000;
                gobjDriverDAC.unDAC2_mV = 1000;                
                OSSetTaskContext(ptrTask, 112, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 112, timer = 1000 msec.
            break;
            
            case 112: // State 112 - Test Mode 1 continues.
                SetRWheelProperty(_STOP, -1.0);                             // Stop wheel.
                SetLWheelProperty(_STOP, -1.0);                        
                OSSetTaskContext(ptrTask, 113, 200*__NUM_SYSTEMTICK_MSEC); // Next state = 113, timer = 200 msec.
            break;

            case 113: // State 113 - Test Mode 1 continues.
                SetRWheelProperty(_FORWARD, -1.0);                          // Set wheel direction.
                SetLWheelProperty(_FORWARD, -1.0);             
                gobjDriverDAC.unDAC1_mV = 600;
                gobjDriverDAC.unDAC2_mV = 600;                
                OSSetTaskContext(ptrTask, 114, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 114, timer = 1000 msec.
            break;
 
            case 114: // State 114 - Test Mode 1 continues.
                gobjDriverDAC.unDAC1_mV = 1000;
                gobjDriverDAC.unDAC2_mV = 1000;                
                OSSetTaskContext(ptrTask, 115, 1000*__NUM_SYSTEMTICK_MSEC); // Next state = 115, timer = 1000 msec.
            break;
            
            case 115: // State 115 - Test Mode 1 continued. 
                SetRWheelProperty(_STOP, -1.0);                             // Stop wheel.
                SetLWheelProperty(_STOP, -1.0);                
                if (gnRobotMode == _ROBOT_MODE_TEST1)                           // Check if still in Test Mode 1.
                {
                    OSSetTaskContext(ptrTask, 110, 200*__NUM_SYSTEMTICK_MSEC);  // Next state = 110, timer = 200 msec.    
                }
                else                                                            // No longer in Test Mode 1.
                {
                    SetLWheelProperty(_STOP, -1.0);                             // Stop the motor driver before switching
                    SetRWheelProperty(_STOP, -1.0);                             // to other modes.
                    OSSetTaskContext(ptrTask, 6, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 6, timer = 10 msec.
                }
            break;
            
            case 120: // State 120 - Test Mode 2, stop the wheels, just monitor all sensors and UART2 communication. 
                      // Drive the head elevation and azimuth motors.
                nPrintln("Engaging Test Mode 2",20);
                SetRWheelProperty(_STOP, -1.0);
                SetLWheelProperty(_STOP, -1.0);                       
                nAngleElevation =  gnHeadElevationAngleSet_Deg;                    
                nAngleAzimuth = gnHeadAzimuthAngleSet_Deg;                               
                nSetRCServoMotor(nAngleElevation,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO); // Drive the elevation motor.
                nSetRCServoMotor(nAngleAzimuth,_MOTORSPEED_FAST,_HEAD_AZIMUTH_SERVO);       // Drive the azimuth motor.
                OSSetTaskContext(ptrTask, 121, 1);                              // Next state = 121, timer = 1.                
                break;

            case 121: // State 121 - Test Mode 2 procedures, drive RC servo motors.
                nAngleElevation =  gnHeadElevationAngleSet_Deg;                                              
                nSetRCServoMotor(nAngleElevation,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO);     // Drive the elevation motor.                                     
                OSSetTaskContext(ptrTask, 122, 1);   // Next state = 122, timer = 1.        
                break;
                
            case 122: // State 122 - Test mode 2, drive RC servo motors.
                // Get board temperature in degree.
                gobjRobotState.nBatteryLevel_mV = (gobjDriverADC.unADC3_mV)*_BATT_DETECTOR_COEFFICIENT;
                gobjRobotState.nTemperature_C = nGetTemperature();
                if (gnRobotMode == _ROBOT_MODE_TEST2)                           // Check if still in Test Mode 2.
                {
                    OSSetTaskContext(ptrTask, 121, 50*__NUM_SYSTEMTICK_MSEC);   // Next state = 121, timer = 50 msec.    
                }
                else                                                            // No longer in Test Mode 2.
                {
                    OSSetTaskContext(ptrTask, 6, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 6, timer = 10 msec.
                }                    
                break;
                
            case 130: // State 130 - Test Mode 3, stop the wheels, just monitor all sensors and UART2 communication. 
                      // Drive the head elevation and azimuth motors.
                nPrintln("Engaging Test Mode 3",20);
                SetRWheelProperty(_FORWARD, -1.0);                              // Set wheels direction.
                SetLWheelProperty(_FORWARD, -1.0);             
                gobjDriverDAC.unDAC1_mV = 600;                                  // Drive wheels.
                gobjDriverDAC.unDAC2_mV = 600;                     
                nAngleElevation =  gnHeadElevationAngleSet_Deg;                    
                nAngleAzimuth = gnHeadAzimuthAngleSet_Deg;                               
                nSetRCServoMotor(nAngleElevation,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO); // Drive the elevation motor.
                nSetRCServoMotor(nAngleAzimuth,_MOTORSPEED_FAST,_HEAD_AZIMUTH_SERVO);       // Drive the azimuth motor.
                OSSetTaskContext(ptrTask, 131, 1);                              // Next state = 131, timer = 1.                
                break;

            case 131: // State 131 - Test Mode 3 procedures, drive RC servo motors.
                nAngleElevation =  gnHeadElevationAngleSet_Deg;                                              
                nSetRCServoMotor(nAngleElevation,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO);     // Drive the elevation motor.                                     
                OSSetTaskContext(ptrTask, 132, 1);   // Next state = 132, timer = 1.        
                break;
                
            case 132: // State 132 - Test mode 3, get board temperature in degree.
                gobjRobotState.nBatteryLevel_mV = (gobjDriverADC.unADC3_mV)*_BATT_DETECTOR_COEFFICIENT;
                gobjRobotState.nTemperature_C = nGetTemperature();
                
                if (gnRobotMode == _ROBOT_MODE_TEST3)                           // Check if still in Test Mode 3.
                {
                    OSSetTaskContext(ptrTask, 131, 50*__NUM_SYSTEMTICK_MSEC);   // Next state = 131, timer = 50 msec.    
                }
                else                                                            // No longer in Test Mode 3.
                {
                    OSSetTaskContext(ptrTask, 6, 10*__NUM_SYSTEMTICK_MSEC);     // Next state = 6, timer = 10 msec.
                }                    
                break;
                
            case 250: // State 250 - Idle.
                gnRobotBalance = _DISABLE;                          // Set the robot to a known state:
                gnRobotMode = _ROBOT_MODE_IDLE;                     // Disable balancing, Idle, and Stop.
                nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_HEAD_ELEVATION_SERVO);     // Turn off servo motor.
                nSetRCServoMotor(-180,_MOTORSPEED_MEDIUM,_HEAD_AZIMUTH_SERVO);     // Turn off servo motor.
                SetRWheelProperty(_STOP, -1.0);
                SetLWheelProperty(_STOP, -1.0);
                break;
                
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}

 
///
/// Function name	: FixedPointtoFloatingPoint
///
/// Author          : Fabian Kung
///
/// Last modified	: 12 November 2018
///
/// Code version	: 0.98
///
/// Processor		: Generic
///
/// Description		: This function accept at 16-bits Q8.8 fixed point integer format and 
///                   convert it into the corresponding floating point value.
///
/// Arguments		: 16-bits Q8.8 fixed point format in unsigned integer.
///                  
/// Return          : Floating point.
///
/// Global variable	: 
///
/// Description     : 

float FixedPointtoFloatingPoint(unsigned int unQ8_8)
{
    float fTemp;
    int nCount;
    // Coefficients for Q8.8 fixed point format.
    static  float fFixQ8_8[16] = {0.00390625, 0.0078125, 0.015625, 0.03125, 0.0625, 0.125,
    0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, -128.0};   
                
    nCount = 0;
    fTemp = 0.0;
    while (nCount < 16)
    {
        //Integer to Q8.8 Fixed Point conversion process.
        if ((unQ8_8 & 0x0001)>0)                // Check bit0.
        {
            fTemp = fTemp + fFixQ8_8[nCount];
        }
        unQ8_8 = unQ8_8 >> 1;                   // Right shift fixed point variable by 1 bit.
        nCount++;                               // Increment counter.            
    }
    return fTemp;
} 
 

///
/// Process name		: Robot_ExControllerMessageLoop
///
/// Author              : Fabian Kung
///
/// Last modified		: 12 April 2022
///
/// Code Version		: 0.89
///

/// Processor           : dsPIC33EP256MU80X family.
///
/// Processor/System Resources
/// PINS                : 
///
/// MODULES             :
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables	: 

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Robot_ExControllerMessageLoop: Incompatible OS version"
	#endif
#else
	#error "Robot_ExControllerMessageLoop: An RTOS is required with this function"
#endif

/// Description:
/// This process handles text based information exchange with an External Controller (EC) via
/// UART2 port.  The process can accept text command and also return parameters requested
/// by the EC in alpha-numeric characters.  Parameters returned are either text string or 3-digits BCD 
/// from 0 to 255.
/// --- Command packet from EC --- 
/// [Command] + [Argument1] + [Argument2] + [Argument3] + [Argument4] + [Newline Character]
/// The Newline character corresponds to 0x0A.
///
/// --- Reply to EC ---
/// If [Command] is G:
/// [Description] + [Hundredth] + [Tenth] + [Digit] + [Newline Character]
/// Where [Description] indicates the type of parameters being reported back.
/// 
/// Else:
/// "OK" or "NO", each string ends with [Newline Character]
///


void Robot_ExControllerMessageLoop(TASK_ATTRIBUTE *ptrTask)
{
    static int  nTimer;
    static unsigned int unParam;
    int nIndex;
    static unsigned int unDigit;
    static unsigned int unTen;
    static unsigned int unHundred;
    static int nNextState;
    int nTemp, nTemp2;
    static int  nSign;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                nTimer = 0;                                                     // Initialize timeout timer to 0.
                OSSetTaskContext(ptrTask, 1, 1);                                // Next state = 1, timer = 1.
            break;
                         
            case 1: // State 1 - Check for incoming data.  Here this state also guarded against transaction
                    // that takes too long, e.g. timeout.
                if (gbytRXbufptr2 > 0)
                {
                    nTimer++;                                                   // Increment timer.
                    if (nTimer > (101*__NUM_SYSTEMTICK_MSEC))                   // Check for timeout, about 101 msec. Whenever a byte
                                                                                // of data is received, this timer will run.  If a 
                                                                                // full packet (of 6 bytes) is not received within the
                                                                                // stipulated period, timeout event is assumed.
                    {                                                           // If timeout occurred.
                        nTimer = 0;                                             // Reset timer.
                        PIN_ILED2 = 0;                                          // Off indicator LED2.
                        gSCIstatus2.bRXRDY = 0;                                 // Reset valid data flag.
                        gbytRXbufptr2 = 0;                                      // Reset pointer.   
                        gSCIstatus2.bRXOVF = 0;                                 // Reset overflow error flag.                        
                        OSSetTaskContext(ptrTask, 1, 1);                        // Next state = 1, timer = 1.
                    }
                    else if (gbytRXbufptr2 > 5)                                 // Check if complete string of command is received. Expect
                    {                                                           // 6 bytes of incoming data packet.
                        if (gSCIstatus2.bRXOVF == 0)                            // Make sure no overflow error.
                        {                     
                            OSSetTaskContext(ptrTask, 2, 1);                    // Next state = 2, timer = 1.
                        }
                        else
                        {
                       
                            gSCIstatus2.bRXOVF = 0;                             // Reset overflow error flag.
                            OSSetTaskContext(ptrTask, 1, 1);                    // Next state = 1, timer = 1.
                        }
                        nTimer = 0;                                             // Reset timer.
                        gSCIstatus2.bRXRDY = 0;                                 // Reset valid data flag.
                        gbytRXbufptr2 = 0;                                      // Reset pointer.                             
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 1, 1);                        // Next state = 1, timer = 1.
                    }
                        
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);                            // Next state = 1, timer = 1.
                }                
                break;        
                
            case 2: // State 2 - Decode message from External Controller (EC).  EC is connected to
                // UART2 port and each data packet is 5 bytes in length. 
                // The format is as follows:
                // Length = 6 bytes.
                // Command packet: [Command] + [Argument1] + [Argument2] + [Argument3] + + [Argument4] + [Newline Character]
                // The Newline character corresponds to 0x0A.
                
                switch(gbytRXbuffer2[0])
                {       
                    // Command from EC.
                    // Byte 0 = Command.
                    // Byte 1 = Argument 1 for command.
                    // Byte 2 = Argument 2 for command.
                    // Byte 3 = Argument 3 for command.
                    // Byte 4 = Argument 4 for command.
                    // Byte 5 = Newline character.
                    case 'S': // Sound a beep. 
                              // Argument 1 = Tone value from 1 to 9.
                              // Argument 2 = Tone value from 1 to 9.
                              // Argument 3 = Tone value from 1 to 9.
                              // Argument 4 = Volume, from 0 to 5.
                        gnAudioTone[0] = gbytRXbuffer2[1]-48;           // The argument is ASCII or UTF-8 character, '1' to '9'.
                                                                        // We thus need to convert to normal integer.            
                        if (gbytRXbuffer2[2] > 48)
                        {
                            gnAudioTone[1] = gbytRXbuffer2[2]-48;
                            if (gbytRXbuffer2[3] > 48)
                            {
                                gnAudioTone[2] = gbytRXbuffer2[3]-48;
                                gnAudioTone[3] = 0;
                            }
                            else
                            {
                                gnAudioTone[2] = 0; 
                            }
                        }
                        else
                        {
                            gnAudioTone[1] = 0;      
                        }
                        gunAudioVolume = gbytRXbuffer2[4]-48;           // Set volume.
                        OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
                        break;
                                
                    case 'T': // Turn, between -99 to +99 degree at 1 degree interval, or continuous turning
                              // at constant turning speed.
                              // Positive angle turns the robot facing left, while negative angle turns
                              // the robot facing right.
                              // Argument 1 = '-' or any other character (will be interpreted as positive).
                              // Argument 2 = Hundredth value, Set to '0' for turn to fixed angle or 
                              //              '1' for continuous turning at default speed.
                              //              Any other value for continuous turning at high speed.
                              // Argument 3 = Tenth value, '0' to '9'.  Ignored for continuous turning.
                              // Argument 4 = Digit value, '0' to '9'.  Ignored for continuous turning.
                        if (gnRobotMode == _ROBOT_MODE_NORMAL)          // Make sure robot is in normal mode
                        {                                               // before movement command can be accepted.
                            nTemp = gbytRXbuffer2[2]-48;                // Get hundredth digit.
                                                                        // The argument is ASCII or UTF-8 character.  So
                                                                        // we turn it into numerics between 0-9.
                            if (nTemp > 9)                              // Limit to between 0-9.
                            {
                                OSSetTaskContext(ptrTask, 101, 1);      // Next state = 101, timer = 1.
                                break;
                            }                                    
                            if (nTemp == 0)                             // Check if equals '0'. Turn to 
                            {                                           // fixed heading mode.
                                nTemp2 = gbytRXbuffer2[3]-48;           // Get the tenth digit.
                                if (nTemp2 > 9)                         // Limit to between 0-9.
                                {
                                    OSSetTaskContext(ptrTask, 101, 1);  // Next state = 101, timer = 1.
                                    break;
                                }                   
                                nTemp = nTemp2 * 10;                    // Convert to tenth.
                                nTemp2 = gbytRXbuffer2[4]-48;           // Get unit digit.
                                if (nTemp2 > 9)                         // Limit to between 0-9.
                                {
                                    OSSetTaskContext(ptrTask, 101, 1);  // Next state = 101, timer = 1.
                                    break;
                                }                                     
                                nTemp = nTemp + nTemp2;                 // Compute the total angle in degrees.
                                if (gbytRXbuffer2[1] == '-')            // Check direction.
                                {    
                                    nSetMovementTurn(-nTemp);           // Execute turn right.
                                }
                                else
                                {
                                    nSetMovementTurn(nTemp);            // Execute turn left.
                                }
                                gnTurnMode = _TURN_MODE_FIXED_HEADING;
                            }
                            else                                        // If hundredth digit is not '0',
                            {                                           // it is continuous turn mode.
                                gnTurnMode = _TURN_MODE_CONT_TURN;
                                if (nTemp == 1)
                                {    
                                    if (gbytRXbuffer2[1] == '-')            // Check direction.
                                    {    
                                        gnTurnSpeedSet = -_CONT_TURN_SPEED_DEFAULT;   // Enable continuous turning, and constant turning speed. 
                                    }
                                    else
                                    {
                                        gnTurnSpeedSet = _CONT_TURN_SPEED_DEFAULT;   // Enable continuous turning, and constant turning speed. 
                                    }
                                }
                                else
                                {
                                    if (gbytRXbuffer2[1] == '-')            // Check direction.
                                    {    
                                        gnTurnSpeedSet = -_CONT_TURN_SPEED_HIGH;   // Enable continuous turning, and constant turning speed. 
                                    }
                                    else
                                    {
                                        gnTurnSpeedSet = _CONT_TURN_SPEED_HIGH;   // Enable continuous turning, and constant turning speed. 
                                    }                                    
                                }
                            }                                 
                            OSSetTaskContext(ptrTask, 100, 1);          // Next state = 100, timer = 1.
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 101, 1);          // Next state = 101, timer = 1.
                        }
                        break;
                                    
                    case 'F': // Move forward or backward.
                              // Argument 1 = '-' (for backward) or any other character (forward).
                              // Argument 2 = Speed setting, from '1' to '4'.
                              // Argument 3 = Don't care, can set to '0'. 
                              // Argument 4 = Don't care, can set to '0'.
                        if (gnRobotMode == _ROBOT_MODE_NORMAL)          // Make sure robot is in normal mode
                        {   
                            nTemp = gbytRXbuffer2[2]-48;                // The argument is ASCII or UTF-8 character.  So
                                                                        // we turn it into numerics between 0-9.    
                            if (nTemp == 1)
                            {
                                gfOmegaWSet = _VELOCITY_MOVE_SLOW;
                            }
                            else if (nTemp == 2)
                            {
                                gfOmegaWSet = _VELOCITY_MOVE_NORMAL;
                            }
                            else if (nTemp == 3)
                            {
                                gfOmegaWSet = _VELOCITY_MOVE_FAST;
                            }
                            else if (nTemp == 4)
                            {
                                gfOmegaWSet = _VELOCITY_MOVE_VERYFAST;
                            }
                            else
                            {
                                OSSetTaskContext(ptrTask, 101, 1);      // Next state = 1, timer = 1.
                                break;
                            }                                
                            if (gbytRXbuffer2[1] == '-')                // Check if direction is backward.
                            {
                                gfOmegaWSet = -gfOmegaWSet;
                            }    
                            OSSetTaskContext(ptrTask, 100, 1);          // Next state = 100, timer = 1.
                        }
                        else
                        {
                            OSSetTaskContext(ptrTask, 101, 1);          // Next state = 1, timer = 1.
                        }
                        break;
                                    
                    case 'X': // Command = Stop all movements.
                              // Argument 1 = Don't care, can set to '0'.
                              // Argument 2 = Don't care, can set to '0'.
                              // Argument 3 = Don't care, can set to '0'.
                        SetMovementStop();
                        nSetMovementTurn(0);
                        OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
                        break;                              
                                    
                    case 'D': // Set RC Servo motor angle, differential mode.
                              // Argument 1 = Motor ID, from '0' to '2'.  Currently only 3 RC servo motors
                              //              are supported.
                              // Argument 2 = Motor output angle direction, '-' = negative, else positive. 
                              // Argument 3 = Differential motor output angle between 0-9.
                              // Argument 4 = don't care.
                        nTemp = gbytRXbuffer2[3]-48;                    // The argument is ASCII or UTF-8 character.  So
                                                                        // we turn it into numerics between 0-9.
                        if (nTemp > 9)                                  // Limit to between 0-9.
                        {
                            OSSetTaskContext(ptrTask, 101, 1);          // Next state = 101, timer = 1.
                            break;
                        }                  
                        if (gbytRXbuffer2[2] == 45)                     // If equal '-', negative.
                        {
                            nTemp = -nTemp;
                        }
                        if (gbytRXbuffer2[1] == 48)                     // Motor 0
                        {
                            gnHeadElevationAngleSet_Deg = gnHeadElevationAngleSet_Deg + nTemp;
                            // Drive Motor 0.
                            nSetRCServoMotor(gnHeadElevationAngleSet_Deg,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO);                                    
                        }
                        else if (gbytRXbuffer2[1] == 49)                 // Motor 1.
                        {
                             gnHeadAzimuthAngleSet_Deg = gnHeadAzimuthAngleSet_Deg + nTemp;
                            // Drive Motor 1.
                            nSetRCServoMotor(gnHeadAzimuthAngleSet_Deg,_MOTORSPEED_FAST,_HEAD_AZIMUTH_SERVO);                            
                        }
                        else if (gbytRXbuffer2[1] == 50)                 // Motor 2.
                        {
                            // Drive Motor 2.
                            nSetRCServoMotor(nTemp,_MOTORSPEED_FAST,2);                            
                        }                        
                        else                                            // Motor 1.
                        {
                            // Drive Motor 3.
                            nSetRCServoMotor(nTemp,_MOTORSPEED_FAST,3);                                          
                        }
                        OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
                        break;

                    case 'M': // Set RC Servo motor angle, absolute.
                              // Argument 1 = Motor ID, from '0' to '3'.  Currently only 4 RC servo motors
                              //              are supported.
                              // Argument 2 = Motor output angle direction, '-' = negative, else positive. 
                              // Argument 3,4 = Motor output angle from 0 to 99.
                        nTemp = gbytRXbuffer2[3]-48;                    // The argument is ASCII or UTF-8 character.  So
                                                                        // we turn it into numerics between 0-9.
                        nTemp2 = gbytRXbuffer2[4]-48;                   // The argument is ASCII or UTF-8 character.  So
                                                                        // we turn it into numerics between 0-9.
                        if ((nTemp > 9) || (nTemp2 >9))                 // Limit both characters to between 0-9.
                        {
                            OSSetTaskContext(ptrTask, 101, 1);          // Next state = 101, timer = 1.
                            break;
                        }                  
                        if (gbytRXbuffer2[2] == 45)                     // If equal '-', negative.
                        {
                            nTemp = -(nTemp*10 + nTemp2);               // Combine both digits into a negative integer value.
                        }
                        else
                        {
                            nTemp = nTemp*10 + nTemp2;                  // Combine both digits into a positive integer value.
                        }
                        if (gbytRXbuffer2[1] == 48)                     // Motor 0
                        {
                            gnHeadElevationAngleSet_Deg = nTemp;
                            // Drive Motor 0.
                            nSetRCServoMotor(gnHeadElevationAngleSet_Deg,_MOTORSPEED_FAST,_HEAD_ELEVATION_SERVO);                                    
                        }
                        else if (gbytRXbuffer2[1] == 49)                 // Motor 1.
                        {
                             gnHeadAzimuthAngleSet_Deg = nTemp;
                            // Drive Motor 1.
                            nSetRCServoMotor(gnHeadAzimuthAngleSet_Deg,_MOTORSPEED_FAST,_HEAD_AZIMUTH_SERVO);                            
                        }
                        else if (gbytRXbuffer2[1] == 50)                 // Motor 2.
                        {
                            // Drive Motor 2.
                            nSetRCServoMotor(nTemp,_MOTORSPEED_FAST,2);                            
                        }                        
                        else                                            // Motor 1.
                        {
                            // Drive Motor 3.
                            nSetRCServoMotor(nTemp,_MOTORSPEED_FAST,3);                                          
                        }
                        OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
                        break;                        
                        
                    case 'G': // Command - Get parameters.
                        // Argument 1 = Type of parameters.
                        // Argument 2 = don't care.
                        // Argument 3 = don't care.
                        // Argument 4 = don't care.
                        if (gbytRXbuffer2[1] == 'D')                    // If it is get distance sensor output.
                        {
                            OSSetTaskContext(ptrTask, 3, 1);            // Next state = 3, timer = 1.
                        }
                        else if (gbytRXbuffer2[1] == 'B')               // If it is get battery level output. 
                        {
                            OSSetTaskContext(ptrTask, 5, 1);            // Next state = 5, timer = 1.
                        }
                        else if (gbytRXbuffer2[1] == 'F')               // If it is the firmware version.
                        {
                            OSSetTaskContext(ptrTask, 10, 1);           // Next state = 10, timer = 1.
                        }
                        else if (gbytRXbuffer2[1] == 'R')               // If it is get robot status.
                        {
                            OSSetTaskContext(ptrTask, 20, 1);           // Next state = 20, timer = 1.
                        }
                        else if (gbytRXbuffer2[1] == 'V')               // Wheel velocity.
                        {
                            OSSetTaskContext(ptrTask, 25, 1);           // Next state = 25, timer = 1.
                        }
                        else if (gbytRXbuffer2[1] == 'A')               // Tilt angle.
                        {
                            OSSetTaskContext(ptrTask, 27, 1);           // Next state = 27, timer = 1.
                        }     
                        else if (gbytRXbuffer2[1] == 'b')               // Send robot status and parameters in binary format.
                        {
                            OSSetTaskContext(ptrTask, 30, 1);           // Next state = 30, timer = 1.
                        }                         
                        else
                        {
                            OSSetTaskContext(ptrTask, 101, 1);          // Next state = 101, timer = 1.
                        }
                        break;
                                
                    case 'R': // Command - External sensors report back their status.
                        // Argument 1 = ID of sensor, 0-255.
                        // Argument 2 = Output byte 1, process ID.     
                        // Argument 3 = Output byte 2, Data 1.
                        // Argument 4 = Output byte 3, Data 2.   
                        if (gbytRXbuffer2[1] == '1')                    // Check sensor ID, if it is machine-vision module (MVM).
                        {
                            gnMVMStatus = gbytRXbuffer2[3];             // Update MVM status register.
                            gunMVMAveLuminance = gbytRXbuffer2[4];      // Update MVM average luminance register.
                        }
                        OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
                        //OSSetTaskContext(ptrTask, 1, 1);                // Next state = 1, timer = 1.
                        break;    
                        
                    default:
                        OSSetTaskContext(ptrTask, 1, 1);                // Next state = 1, timer = 1.
                        break;    
                }
            break;
                
            case 3: // State 3 - Send distance sensor output to remote controller, part 1.
                unParam = (gobjDriverADC.unADC1_mV) >> 3;                       // Divide by 8, as the ADC output is from 0 to 2047 (11 bits). 
                nNextState = 4;                                                 // After converting unParam to BCD, should return to state 4.
                OSSetTaskContext(ptrTask, 200, 1);                              // Next state = 200, timer = 1.
                break;  
                
            case 4: // State 4 - Send distance sensor output to remote controller, part 2.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'D';         // Load data.
                    gbytTXbuffer2[1] = unHundred + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[2] = unTen + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[3] = unDigit + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[4] = 0x0A; // Newline character.
                    gbytTXbuflen2 = 5;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.
                break;
                
            case 5: // State 5 - Send battery level output to External Controller, part 1.
                unParam = (gobjDriverADC.unADC3_mV) >> 3;                       // Divide by 8, as the ADC output is from 0 to 2047 (11 bits). 
                nNextState = 6;                                                 // After converting unParam to BCD, should return to state 6.
                OSSetTaskContext(ptrTask, 200, 1);                              // Next state = 200, timer = 1.
                break;
                
            case 6: // State 6 - Send battery level output to External Controller, part 2.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'B';         // Load data.
                    gbytTXbuffer2[1] = unHundred + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[2] = unTen + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[3] = unDigit + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[4] = 0x0A; // Newline character.
                    gbytTXbuflen2 = 5;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.
                break;
                
            case 10: // State 10 - Send firmware version to EC.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'F';	// Load data.
                    gbytTXbuffer2[1] = gbytFirmwareVersion[0];	// Load data.
                    gbytTXbuffer2[2] = gbytFirmwareVersion[1];	// Load data.
                    gbytTXbuffer2[3] = gbytFirmwareVersion[2];	// Load data.
                    gbytTXbuffer2[4] = gbytFirmwareVersion[3];	// Load data.
                    gbytTXbuffer2[5] = 0x0A; // Newline character.                   
                    gbytTXbuflen2 = 6;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);          // Next state = 1, timer = 2 msec.
                break;

            case 20: // State 20 - Report robot mode to EC.
                if (gnRobotMode == _ROBOT_MODE_NORMAL)
                {

                    OSSetTaskContext(ptrTask, 100, 1);                          // Next state = 100, timer = 1.  
                }
                else
                {
                    OSSetTaskContext(ptrTask, 101, 1);                          // Next state = 101, timer = 1.
                }
                break;                
                
            case 25: // State 25 - Send robot average wheel velocity to EC, part 1.
                     // Convert average wheel velocity to unsigned integer
                     //  1.0 revolution per second = 20 units.
                     //  0 = -5.0 revolution per second.
                     //  100 = 0 revolution per second.
                     //  200 = 5.0 revolution per second.
                     //  > 200 = Overflow or underflow.
                nSign = 1;                              // Default to positive sign.
                nTemp = gfOmegaW*20;
                if (nTemp < 0)
                {
                    nTemp = -nTemp;
                    nSign = -1;                         // Set to negative sign.
                }
                unParam = nTemp;                                                                
                nNextState = 26;                                                // After converting unParam to BCD, should return to state 6.
                OSSetTaskContext(ptrTask, 200, 1);                              // Next state = 200, timer = 1.                
                break;
                
            case 26: // State 26 - Send robot average wheel velocity to EC, part 2.                 
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'V';         // Load data.
                    if (nSign > 0)
                    {
                        gbytTXbuffer2[1] = '+';    // Set sign.
                    }
                    else
                    {
                        gbytTXbuffer2[1] = '-';
                    }
                    gbytTXbuffer2[2] = unTen + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[3] = unDigit + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[4] = 0x0A; // Newline character.
                    gbytTXbuflen2 = 5;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.                
                break;
                
            case 27: // State 27 - Send robot tilt angle to EC, part 1.
                nSign = 1;                              // Default to positive sign.
                nTemp = gnTheta_Deg_MPU6050;
                if (nTemp < 0)
                {
                    nTemp = -nTemp;
                    nSign = -1;                         // Set to negative sign.
                }
                unParam = nTemp;                                                                
                nNextState = 28;                                                // After converting unParam to BCD, should return to state 6.
                OSSetTaskContext(ptrTask, 200, 1);                              // Next state = 200, timer = 1.                 
                break;
                
            case 28: // State 28 - Send robot tilt angle to EC, part 2.              
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'A';         // Load data.
                    if (nSign > 0)
                    {
                        gbytTXbuffer2[1] = '+';    // Set sign.
                    }
                    else
                    {
                        gbytTXbuffer2[1] = '-';
                    }
                    gbytTXbuffer2[2] = unTen + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[3] = unDigit + 0x30;    // Convert number to ASCII.
                    gbytTXbuffer2[4] = 0x0A; // Newline character.
                    gbytTXbuflen2 = 5;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.                
                break;
                
            case 30: // State 30 - Send back robot status and parameters in binary format.  The return packet consists of 8
                     // bytes, with a header and 7 bytes of payload.  The length payload can be updated in future.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'b';                                     // Indicate all parameters in binary.
                    nTemp = gnRobotMode & 0x0F;                                 // bit3-0 represents the Robot Mode.
                                                                                // Bit4 indicates whether Robot Controller
                                                                                // is in manual (0) or auto mode (1).
                                                                                // Bit 5 indicates whether Robot platform collides
                                                                                // with object or not (based on stability indices
                                                                                // signature).
                                                                                // Bit 6 indicates whether Robot platform is moving
                                                                                // or not (voluntarily or involuntarily).
                    if (gnRobotMoveMode == _ROBOT_MOVE_AUTO)                    // Set robot move mode.
                    {
                        nTemp = nTemp | 0x10;                                   // Set bit4.
                    }
                    if (gobjRobotFlag.bCOLLISION == 1)
                    {
                        nTemp = nTemp | 0x20;                                   // Set bit5.
                        gobjRobotFlag.bCOLLISION = 0;                           // Clear flag.
                    }
                    if (gobjRobotFlag.bMOVE == 1)
                    {
                        nTemp = nTemp | 0x40;                                   // Set bit5.
                        gobjRobotFlag.bMOVE = 0;                                // Clear flag.                        
                    }
                    gbytTXbuffer2[1] = nTemp;                                   // Set robot controller status.
                    gbytTXbuffer2[2] = gnTheta_Deg_MPU6050 + 127;               // Get tilt angle, offset by 127.
                    gbytTXbuffer2[3] = gnHeading>>8;                            // Get upper 8 bits of heading. 
                    gbytTXbuffer2[4] = gnHeading;                               // Get lower 8 bits of heading
                    gbytTXbuffer2[5] = gobjRobotState.nFrontObjDistance_mm;     // Get the front distance sensor output, unit in mm.
                    gbytTXbuffer2[6] = gnDistanceMoveW>>8;                      // Get upper 8 bits of distance moved.
                    gbytTXbuffer2[7] = gnDistanceMoveW;                         // Get lower 8 bits of distance moved.
                    gbytTXbuflen2 = 8;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 5*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 5 msec.                    
                break;
                
            case 100: // State 100 - Send acknowledgement to EC.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'O';	// Load data.
                    gbytTXbuffer2[1] = 'K';	// Load data.
                    gbytTXbuffer2[2] = 0x0A; // Newline character.                   
                    gbytTXbuflen2 = 3;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.
                break;  
                
            case 101: // State 100 - Send non-acknowledgement to EC.
                if (gSCIstatus2.bTXRDY == 0)	// Check if any data to send via UART.
                {
                    gbytTXbuffer2[0] = 'N';	// Load data.
                    gbytTXbuffer2[1] = 'O';	// Load data.
                    gbytTXbuffer2[2] = 0x0A; // Newline character.                   
                    gbytTXbuflen2 = 3;		// Set TX frame length.
                    gSCIstatus2.bTXRDY = 1;	// Initiate TX.
                }
                OSSetTaskContext(ptrTask, 1, 2*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 2 msec.
                break;                   
                
            case 200: // State 200 - Using the double-dabble method (also known as shift and add 3) 
                      // to convert 8-bits unsigned integer to 3 digits BCD.  The unsigned integer 
                      // value to convert is unParam.
                unDigit = 0;
                unTen = 0;
                unHundred = 0;
                for (nIndex = 0; nIndex < 8; nIndex++) // 8-bits unsigned integer.
                {
                    if (unHundred > 4)
                    {
                        unHundred = unHundred + 3;
                    }
                    if (unTen > 4)
                    {
                        unTen = unTen + 3;
                    }
                    if (unDigit > 4)
                    {
                        unDigit = unDigit + 3;
                    }                    
                    unHundred = unHundred<<1;
                    // Add bit 3 of unTen to unHundred.
                    if ((unTen & 0x0008)>0)
                    {
                        unHundred++;
                    }
                    unTen = (unTen<<1) & 0x000F;  // Shift and only retain lower 4 bits.
                    // Add bit 3 of unTen to unHundred.
                    if ((unDigit & 0x0008)>0)
                    {
                        unTen++;
                    }        
                    unDigit = (unDigit<<1) & 0x000F;  // Shift and only retain lower 4 bits.
                    // Add bit 7 of parameter to unDigit.
                    if ((unParam & 0x0080)>0)
                    {
                        unDigit++;
                    }                    
                    unParam = (unParam<<1) & 0x00FF;                   
                }            
                OSSetTaskContext(ptrTask, nNextState, 1);  // Next state = nNextState, timer = 1.
                break;
                
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}


///
/// Process name		: User_Proce1
///
/// Author              : Fabian Kung
///
/// Last modified		: 27 July 2019
///
/// Code Version		: 0.84
///

/// Processor           : dsPIC33EP256MU80X family.
///
/// Processor/System Resources
/// PINS                : 
///
/// MODULES             :
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables	: 

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "User_Proce1: Incompatible OS version"
	#endif
#else
	#error "User_Proce1: An RTOS is required with this function"
#endif

/// Description:
/// This process performs two important functions:
/// 1. It loads important global constants of the robot stored in the Program Memory into global
/// variables residing in Data Memory.
/// 2. It initiate connection to the a remote host via RF wireless link or wired link.  The 
/// global constants is send to the remote host to enable the user to adjust these constants if
/// needed.  Another process, "HC05_CommHandler( )" is tasked with capture commands from the user
/// and tune the global constant values.
/// The remote host should be a PC or Tablet computer or similar devices with 
/// high computing power and large display.  The wireless transceiver can be Bluetooth or other custom
/// devices.  The process polls the global variable gnRFLinkState until RF link is established by
/// the RF driver (Please refer to the header file for the respective RF link driver for further
/// details).  After the RF link is established, this process will transmit setup
/// data and robot physical parameters to the remote host so that the remote host knows the 
/// default values of the feedback control and physical parameters of the robot.  
/// This allows a user on the remote host to monitor the robot instantaneous state and tune 
/// the robot feedback control algorithm if necessary.
/// Note that the data transmitted includes Q8.8 fixed point format and unsigned integers.


void User_Proce1(TASK_ATTRIBUTE *ptrTask)
{
    static unsigned int unTemp;
    static int nCount = 0;
    static unsigned int nTableOffset;
    int nTblpag_bak;
        
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                gnEnHC05Init = 0;                    // Disable initialization of HC-05 Bluetooth module.
                //gnEnHC05Init = 1;                    // Enable initialization of HC-05 Bluettoth module.
                OSSetTaskContext(ptrTask, 1, 1);    // Next state = 1, timer = 1.
            break;
                         
            case 1: // State 1 - Load 16-bits unsigned constants from Program Memory and setup tunable coeffcients structure 1.
                nTblpag_bak = TBLPAG;                                   // The datasheet recommends backing up TBLPAG before 
                                                                        // modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address, table page.
                nTableOffset = __builtin_tbloffset(progData);           // Load the lower 16-bits of the 24-bits address, the offset.
                gunProgDataKP_Q8_8 = __builtin_tblrdl(nTableOffset);    // Load value for coefficient KP in Q8.8
                                                                        // fixed point format.
                gunProgDataKP_Ori_Q8_8 = gunProgDataKP_Q8_8;            // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.
                
                // Initialize description of Kp coefficent.
                objCoeffDef[0].fDefault = FixedPointtoFloatingPoint(gunProgDataKP_Q8_8);  // Calculate the floating point value as stored
                                                                                    // in program memory.
                objCoeffDef[0].unInterval = _FP_INTERVAL_FP;            // Backup interval in Q8.8 fixed point format.     
                objCoeffDef[0].unDefault = gunProgDataKP_Q8_8;          // Backup Q8.8 fixed point value.
                objCoeffDef[0].pbytDes = gbytCeoffKp;                   // Load description string.
                OSSetTaskContext(ptrTask, 2, 1);                        // Next state = 2, timer = 1.
                break;
                
            case 2: // State 2 - Load 16-bits unsigned constants from Program Memory to Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataKX_Q8_8 = __builtin_tblrdl(nTableOffset);    // Value for coefficient KX in Q8.8
                                                                        // fixed point format.
                gunProgDataKX_Ori_Q8_8 = gunProgDataKX_Q8_8;            // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.
                
                // Initialize description of Kx coefficent.
                objCoeffDef[1].fDefault = FixedPointtoFloatingPoint(gunProgDataKX_Q8_8);  // Calculate the floating point value as stored
                                                                                    // in program memory.
                objCoeffDef[1].unInterval = _FKX_INTERVAL_FP;           // Backup interval in Q8.8 fixed point format. 
                objCoeffDef[1].unDefault = gunProgDataKX_Q8_8;          // Backup Q8.8 fixed point value.
                objCoeffDef[1].pbytDes = gbytCoeffKx;                   // Load description string.
                OSSetTaskContext(ptrTask, 3, 1);                        // Next state = 3, timer = 1.
                break; 

            case 3: // State 3 - Load 16-bits unsigned constants stored in Program Memory into Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataKV_Q8_8 = __builtin_tblrdl(nTableOffset);    // Value for coefficient KP in Q8.8
                                                                        // fixed point format.
                gunProgDataKV_Ori_Q8_8 = gunProgDataKV_Q8_8;            // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.
                
                objCoeffDef[2].fDefault = FixedPointtoFloatingPoint(gunProgDataKV_Q8_8);  // Calculate the floating point value as stored
                                                                                    // in program memory.
                objCoeffDef[2].unInterval = _FKV_INTERVAL_FP;
                objCoeffDef[2].unDefault = gunProgDataKV_Q8_8;
                objCoeffDef[2].pbytDes = gbytCoeffKv;                   // Load description string.    
                OSSetTaskContext(ptrTask, 4, 1);                        // Next state = 4, timer = 1.
                break; 
                
            case 4: // State 4 - Load 16-bits unsigned constants stored in Program Memory into Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataKWg_Q8_8 = __builtin_tblrdl(nTableOffset);   // Value for coefficient KP in Q8.8
                                                                        // fixed point format.
                gunProgDataKWg_Ori_Q8_8 = gunProgDataKWg_Q8_8;          // Also backup the initial value.                
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.
                
                objCoeffDef[3].fDefault = FixedPointtoFloatingPoint(gunProgDataKWg_Q8_8); // Calculate the floating point value as stored
                                                                                    // in program memory.
                objCoeffDef[3].unInterval = _FCW_INTERVAL_FP;
                objCoeffDef[3].unDefault = gunProgDataKWg_Q8_8;
                objCoeffDef[3].pbytDes = gbytCoeffKw;                   // Load description string.
                OSSetTaskContext(ptrTask, 5, 1);                        // Next state = 51, timer = 1.
                break;                 

            case 5: // State 5 - Load 16-bits unsigned constants stored in Program Memory into Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataR_Q8_8 = __builtin_tblrdl(nTableOffset);     // Value for coefficient KP in Q8.8
                                                                        // fixed point format.
                gunProgDataR_Ori_Q8_8 = gunProgDataR_Q8_8;              // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.

                objCoeffDef[4].fDefault = FixedPointtoFloatingPoint(gunProgDataR_Q8_8);   // Calculate the floating point value as stored
                                                                                    // in program memory.
                objCoeffDef[4].unInterval = _R_INTERVAL_FP;
                objCoeffDef[4].unDefault = gunProgDataR_Q8_8;         
                objCoeffDef[4].pbytDes = gbytCoeffRs;                   // Load description string.
                
                OSSetTaskContext(ptrTask, 6, 1);                        // Next state = 6, timer = 1.
                break; 

            case 6: // State 6 - Load 16-bits unsigned constants stored in Program Memory into Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataLMotorOffset_Q8_8 = __builtin_tblrdl(nTableOffset);     // Value for L motor offset in unsigned integer.
                                                                        // fixed point format.
                gunProgDataLMotorOffset_Ori_Q8_8 = gunProgDataLMotorOffset_Q8_8;   // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.

                objCoeffDef[5].fDefault = FixedPointtoFloatingPoint(gunProgDataLMotorOffset_Q8_8);      // Calculate the floating point value as stored
                                                                        // in program memory.
                objCoeffDef[5].unInterval = _MOS_INTERVAL_FP;
                objCoeffDef[5].unDefault = gunProgDataLMotorOffset_Q8_8;     // Initialize the unsigned integer value, for this    
                objCoeffDef[5].pbytDes = gbytCoeffMoL;                  // case the fDefault and unDefault are the same.    
                
                OSSetTaskContext(ptrTask, 7, 1);                        // Next state = 7, timer = 1.
                break; 
                
            case 7: // State 7 - Load 16-bits unsigned constants stored in Program Memory into Data Memory.
                nTblpag_bak = TBLPAG;                                   // Backing up TBLPAG before modifying it.
                TBLPAG = __builtin_tblpage (progData);                  // Set the upper 8-bits of the 24-bits address.
                nTableOffset += 2;                                      // Point to next instruction word in Program Memory.
                gunProgDataRMotorOffset_Q8_8 = __builtin_tblrdl(nTableOffset);     // Value for L motor offset in unsigned integer.
                                                                        // fixed point format.
                gunProgDataRMotorOffset_Ori_Q8_8 = gunProgDataRMotorOffset_Q8_8;   // Also backup the initial value.
                TBLPAG = nTblpag_bak;                                   // Restore TBLPAG.

                objCoeffDef[6].fDefault = FixedPointtoFloatingPoint(gunProgDataRMotorOffset_Q8_8);      // Calculate the floating point value as stored
                                                                        // in program memory.
                objCoeffDef[6].unInterval = _MOS_INTERVAL_FP;
                objCoeffDef[6].unDefault = gunProgDataRMotorOffset_Q8_8;     // Initialize the unsigned integer value, for this   
                objCoeffDef[6].pbytDes = gbytCoeffMoR;                  // case the fDefault and unDefault are the same.
                
                OSSetTaskContext(ptrTask, 9, 1);                        // Next state = 9, timer = 1.
                break; 
                
            case 9: // State 9 - Initialize the rest of the tuning coefficient structures.                                    
                //objCoeffDef[5].fDefault = _MOTOR_L_OFFSET_MILIVOLT; // Initialize description of motor offset, left.
                //objCoeffDef[5].unInterval = _MOS_INTERVAL_FP;
                //objCoeffDef[5].unDefault = _MOS_DEFAULT_FP;
                //objCoeffDef[5].pbytDes = gbytCoeffMoL;    
 
                //objCoeffDef[6].fDefault = _MOTOR_R_OFFSET_MILIVOLT;  // Initialize description of motor offset, right.
                //objCoeffDef[6].unInterval = _MOS_INTERVAL_FP;
                //objCoeffDef[6].unDefault = _MOS_DEFAULT_FP;
                //objCoeffDef[6].pbytDes = gbytCoeffMoR;    
                
                objCoeffDef[7].fDefault = 0.0;        // Initialize description of coefficient 8.
                objCoeffDef[7].unInterval = 0b0000000100000000;
                objCoeffDef[7].unDefault = 0b0000000000000000;
                objCoeffDef[7].pbytDes = gbytCoeffKd2;
                
                objCoeffDef[8].fDefault = 0.0;        // Initialize description of coefficient 9.
                objCoeffDef[8].unInterval = 0b0000000100000000;
                objCoeffDef[8].unDefault = 0b0000000000000000;
                objCoeffDef[8].pbytDes = gbytCoeffKi2;
                
                objCoeffDef[9].fDefault = 0.0;        // Initialize description of coefficient 10.
                objCoeffDef[9].unInterval = 0b0000000100000000;
                objCoeffDef[9].unDefault = 0b0000000000000000;
                objCoeffDef[9].pbytDes = gbytCoeffEle;        
                
                OSSetTaskContext(ptrTask, 10, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 1 msec.
                break;
                
            case 10: // State 10 - Wait until RF link is established with remote PC or similar devices before 
                     // proceeding, otherwise keep waiting.
                     // Note: if RF link with remote PC or Tablet computer is established, the robot will first 
                     // transmits the setup information for all tuning parameters to the remote host.  Then
                     // periodic data transmission to the remote PC and will also enabled by setting gnEnRFTxPeriodicData = 1.  
                     // Otherwise if RF link is established with remote host with smaller
                     // display such as smartphone, it is not possible to show robot status periodically and accommodate 
                     // the controls for coefficient tuning.  
                if (gobjDriverHC05.nRFLinkState == 1)     // Check if RF link is established with remote PC.  
                {
                    nCount = 0;       
                    SetSoundPattern(gunSoundConnectedPC,5);     // Generate a string of tones to indicate the RF link established
                                                                // with PC, tablet or similar devices.    
                    OSSetTaskContext(ptrTask, 11, 500*__NUM_SYSTEMTICK_MSEC); // Next state = 11, timer = 500 msec.
                }
                else if (gobjDriverHC05.nRFLinkState == 2)
                {
                    SetSoundPattern(gunSoundConnectedSP,5);     // Generate a string of tones to indicate the RF link established
                                                                // with smart phone or similar devices.                        
                    OSSetTaskContext(ptrTask, 22, 500*__NUM_SYSTEMTICK_MSEC); // Next state = 22, timer = 500 msec.
                }                    
                else
                {
                    OSSetTaskContext(ptrTask, 10, 100*__NUM_SYSTEMTICK_MSEC); // Next state = 10, timer = 100 msec.
                }
                break;

            case 11: // State 11 - Transmit the start and interval for tunable coefficients.
                // 16-bits Fixed point format for representing real numbers
                // Q8.8 format is adopted.  The first 8 bits are the fractions, while the
                // subsequent 8 bits are the integers.
                if (gSCIstatus.bTXRDY == 0)                         // Check if UART1 is idle.
                {    
                    gbytTXbuffer[0] = gobjDriverHC05.bytRFAdd;      // Set destination address.
                    gbytTXbuffer[1] = 0x55;                         // Indicate Command.
                    gbytTXbuffer[2] = _TUNABLE_COEFFICIENT_INFO;
                    gbytTXbuffer[3] = nCount+1;                     // Indicate coefficient no.
                    unTemp = objCoeffDef[nCount].unDefault;         // Load start value in Q8.8 format.
                                                                    // (relative to initial value), 0.0.
                    gbytTXbuffer[4] = unTemp>>8;                    // Send upper byte of integer data.
                    gbytTXbuffer[5] = unTemp;                       // Send lower byte of integer data.
                    unTemp = objCoeffDef[nCount].unInterval;        // Load interval, in Q8.8 format.
                    gbytTXbuffer[6] = unTemp>>8;
                    gbytTXbuffer[7] = unTemp;
                    gbytTXbuffer[8] = *(objCoeffDef[nCount].pbytDes);    // Load description of coefficients.
                    gbytTXbuffer[9] = *((objCoeffDef[nCount].pbytDes)+1);
                    gbytTXbuffer[10] = *((objCoeffDef[nCount].pbytDes)+2);
                    gbytTXbuflen = 64;
                    gSCIstatus.bTXRDY = 1;                          // Initiate TX.
                
                    nCount++;                                       // Point to next tuning coefficient.
                    if (nCount == 10)                               // Check for end of list.
                    {
                        OSSetTaskContext(ptrTask, 12, 300*__NUM_SYSTEMTICK_MSEC); // Next state = 12, timer = 300 msec.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 11, 300*__NUM_SYSTEMTICK_MSEC); // Next state = 11, timer = 300 msec.
                    }
                }
                else
                {
                    OSSetTaskContext(ptrTask, 11, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 11, timer = 1 msec.
                }
                break;
                
            case 12: // State 12 - Transmit other constant coefficients (wheel physical properties).
                gbytTXbuffer[0] = gobjDriverHC05.bytRFAdd;                  // Set destination address.
                gbytTXbuffer[1] = 0x55;                                     // Indicate Command.
                gbytTXbuffer[2] = _COEFFICENT_INFO;
                gbytTXbuffer[3] = _TURN_DEG_PER_100;
                gbytTXbuffer[4] = _WHEEL_DIAM_MM;
                gbytTXbuffer[5] = _NOTICKSROTATION;                         // This might be an issue as transmit buffer is only 8 bits, while
                                                                            // constant _NOTICKSROTATION can be larger than 255.
                gbytTXbuflen = 64;
                gSCIstatus.bTXRDY = 1;                                      // Initiate TX.                
                OSSetTaskContext(ptrTask, 21, 300*__NUM_SYSTEMTICK_MSEC);   // Next state = 21, timer = 300 msec.
            break;
            
            case 21: // State 21 - Enable sending of periodic binary data.
                gobjDriverHC05.nEnRFTxPeriodicData = 1;         // This is to indicate to HC-05 module driver that system is ready,
                                                                // periodic binary data transmission can commence.
                OSSetTaskContext(ptrTask, 22, 10*__NUM_SYSTEMTICK_MSEC);   // Next state = 22, timer = 10 msec.
            break;

            case 22: // State 22 - Check for RF linkage status with Remote Host.  If RF link is lost
                     // reset this process.
                if (gobjDriverHC05.nRFLinkState > 0)                            // Check if RF link is still established.
                {
                    if (gnRobotMode == _ROBOT_MODE_FAC_DEFAULT)                 // Check if set to factory default request is asserted.
                    {                  
                        gobjDriverHC05.nEnRFTxPeriodicData = 0;                 // Disable periodic binary data transmission to 
                                                                                // prevent collision of wireless data.
                        OSSetTaskContext(ptrTask, 30, 250*__NUM_SYSTEMTICK_MSEC);    // Next state = 30, timer = 250 msec.
                    }   
                    else
                    {
                        OSSetTaskContext(ptrTask, 22, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 22, timer = 10 msec.
                    }                    
                }
                else                                                            // RF link is lost, reset.
                {
                    SetSoundPattern(gunSoundDisconnected,6);                    // Generate a string of tones to indicate the RF link is lost.      
                    OSSetTaskContext(ptrTask, 10, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 10, timer = 10 msec.
                }
            break;
                
            case 30: // State 30 - Restore coefficients to factory default.                
                gunProgDataKP_Q8_8 = _FP_DEFAULT_FP;
                gunProgDataKP_Ori_Q8_8 = gunProgDataKP_Q8_8;
                objCoeffDef[0].fDefault = FixedPointtoFloatingPoint(gunProgDataKP_Q8_8);;      // Initialize description of Kp.
                objCoeffDef[0].unInterval = _FP_INTERVAL_FP;
                objCoeffDef[0].unDefault = _FP_DEFAULT_FP;
                objCoeffDef[0].pbytDes = gbytCeoffKp;                
                
                gunProgDataKX_Q8_8 = _FKX_DEFAULT_FP;
                gunProgDataKX_Ori_Q8_8 = gunProgDataKX_Q8_8;
                objCoeffDef[1].fDefault = FixedPointtoFloatingPoint(gunProgDataKX_Q8_8);     // Initialize description of Kx.
                objCoeffDef[1].unInterval = _FKX_INTERVAL_FP;
                objCoeffDef[1].unDefault = _FKX_DEFAULT_FP;
                objCoeffDef[1].pbytDes = gbytCoeffKx;

                OSSetTaskContext(ptrTask, 31, 1);   // Next state = 31, timer = 1c.
                break;

            case 31: // State 31 - Continue to restore coefficients to factory default.                
                gunProgDataKV_Q8_8 = _FKV_DEFAULT_FP;       
                gunProgDataKV_Ori_Q8_8 = gunProgDataKV_Q8_8;
                objCoeffDef[2].fDefault = FixedPointtoFloatingPoint(gunProgDataKV_Q8_8);     // Initialize description of Kv.
                objCoeffDef[2].unInterval = _FKV_INTERVAL_FP;
                objCoeffDef[2].unDefault = _FKV_DEFAULT_FP;
                objCoeffDef[2].pbytDes = gbytCoeffKv; 
                
                gunProgDataKWg_Q8_8 = _FCW_DEFAULT_FP;
                gunProgDataKWg_Ori_Q8_8 = gunProgDataKWg_Q8_8;                
                objCoeffDef[3].fDefault = FixedPointtoFloatingPoint(gunProgDataKWg_Q8_8);     // Initialize description of Kcw.
                objCoeffDef[3].unInterval = _FCW_INTERVAL_FP;
                objCoeffDef[3].unDefault = _FCW_DEFAULT_FP;
                objCoeffDef[3].pbytDes = gbytCoeffKw;    
                
                OSSetTaskContext(ptrTask, 32, 1);       // Next state = 32, timer = 1c.
                break;
                
            case 32: // State 32 - Continue to restore coefficients to factory default.  
                gunProgDataR_Q8_8 = _R_DEFAULT_FP;    
                gunProgDataR_Ori_Q8_8 = gunProgDataR_Q8_8;
                objCoeffDef[4].fDefault = FixedPointtoFloatingPoint(gunProgDataR_Q8_8);        // Initialize description of R.
                objCoeffDef[4].unInterval = _R_INTERVAL_FP;
                objCoeffDef[4].unDefault = _R_DEFAULT_FP;             
                objCoeffDef[4].pbytDes = gbytCoeffRs;  
            
                OSSetTaskContext(ptrTask, 33, 1);       // Next state = 33, timer = 1c.
                break;
                
            case 33: // State 33 - Continue to restore coefficients to factory default.  
                gunProgDataLMotorOffset_Q8_8 = _L_MOT_OFFSET_DEFAULT_FP;    
                gunProgDataLMotorOffset_Ori_Q8_8 = gunProgDataLMotorOffset_Q8_8;
                objCoeffDef[5].fDefault = FixedPointtoFloatingPoint(gunProgDataLMotorOffset_Q8_8);        // Initialize description of R.
                objCoeffDef[5].unInterval = _MOS_INTERVAL_FP;
                objCoeffDef[5].unDefault = _L_MOT_OFFSET_DEFAULT_FP;             
                objCoeffDef[5].pbytDes = gbytCoeffMoL;  

                gunProgDataRMotorOffset_Q8_8 = _R_MOT_OFFSET_DEFAULT_FP;    
                gunProgDataRMotorOffset_Ori_Q8_8 = gunProgDataRMotorOffset_Q8_8;
                objCoeffDef[6].fDefault = FixedPointtoFloatingPoint(gunProgDataLMotorOffset_Q8_8);        // Initialize description of R.
                objCoeffDef[6].unInterval = _MOS_INTERVAL_FP;
                objCoeffDef[6].unDefault = _R_MOT_OFFSET_DEFAULT_FP;             
                objCoeffDef[6].pbytDes = gbytCoeffMoR;                
                
                OSSetTaskContext(ptrTask, 34, 1);       // Next state = 34, timer = 1c.
                break;                
                
            case 34: // State 34 - Finalize steps to restore coefficients to factory default.
                
                objCoeffDef[7].fDefault = 0.0;        // Initialize description of coefficient 8.
                objCoeffDef[7].unInterval = 0b0000000100000000;
                objCoeffDef[7].unDefault = 0b0000000000000000;
                objCoeffDef[7].pbytDes = gbytCoeffKd2;
                
                objCoeffDef[8].fDefault = 0.0;        // Initialize description of coefficient 9.
                objCoeffDef[8].unInterval = 0b0000000100000000;
                objCoeffDef[8].unDefault = 0b0000000000000000;
                objCoeffDef[8].pbytDes = gbytCoeffKi2;
                
                objCoeffDef[9].fDefault = 0.0;        // Initialize description of coefficient 10.
                objCoeffDef[9].unInterval = 0b0000000100000000;
                objCoeffDef[9].unDefault = 0b0000000000000000;
                objCoeffDef[9].pbytDes = gbytCoeffEle;                        
                
                gnRobotMode = _ROBOT_MODE_IDLE;                         // Set the robot mode back to idle.
                OSSetTaskContext(ptrTask, 10, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 1 msec.
                break;
                
            case 100: // State 100 - Idle.
                OSSetTaskContext(ptrTask, 100, 100); // Next state = 100, timer = 100.
            break;

            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}


///
/// Process name		: User_ProceTest2
///
/// Author              : Fabian Kung
///
/// Last modified		: 25 July 2022
///
/// Code Version		: 0.64
///

/// Processor           : dsPIC33EP256MU80X family.
///
/// Processor/System Resources
/// PINS                : 
///
/// MODULES             : All other modules.
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables	: 

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "User_ProceTest2: Incompatible OS version"
	#endif
#else
	#error "User_ProceTest2: An RTOS is required with this function"
#endif

/// Description
/// Test user process, that simulate higher level behavior of the robot. For instance,
/// obstacle avoidance.

void User_ProceTest2(TASK_ATTRIBUTE *ptrTask)
{       
    static unsigned int unClearCounter;
    static int  nManualHeadElevationSet_Deg;
    
    float fTemp;
    int nTemp;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization of all high level parameters.
                // --- Initialize high level navigation parameters and robot pose ---
                gnPreferHeadingDeg = 90;                                        // This is measured with respect to x-axis, as 
                                                                                // per the convention in coordinate geometry.
                gnCurrentHeadingDeg = gnPreferHeadingDeg;
                nManualHeadElevationSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT + 10; // Set head elevation position for 
                                                                                      // dynamic movement, look down.   
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);        // Next state = 1, timer = 100 msec.
                break;
                
            case 1: // State 1 - Wait for robot in NORMAL state.
                if (gnRobotMode == _ROBOT_MODE_NORMAL) // Make sure robot platform is under Normal mode before proceed.
                {
                    OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 2, timer = 1 msec.
                }
                else
                {
                    if (gunExperimentalMode == 1)
                    {
                        gunExperimentalMode = 0;
                        OSSetTaskContext(ptrTask, 100, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 100, timer = 10 msec.
                    }
                    else
                    {    
                        OSSetTaskContext(ptrTask, 1, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 10 msec.
                    }
                }   // if (gnRobotMode == _ROBOT_MODE_NORMAL)                
                break;
                
            case 2: // State 2 - Wait until robot is in autonomous mode before proceeding.
                    // During waiting, monitor the MVM sensor output (if connected), and adjust the robot head unit 
                    // to default orientation if changed. 
                if (gnRobotMoveMode == _ROBOT_MOVE_AUTO)
                {
                    unClearCounter = 0;
                    gnHeadElevationAngleSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT + 4; // Set head elevation position for 
                                                                                    // dynamic movement, look down a bit.
                    OSSetTaskContext(ptrTask, 5, 500*__NUM_SYSTEMTICK_MSEC);    // Next state = 5, timer = 500 msec.
                }
                else
                {
                    // Move the head unit gradually back to default pose if it is not.
                    /*
                    if (gnHeadElevationAngleSet_Deg > nManualHeadElevationSet_Deg)
                    {
                        gnHeadElevationAngleSet_Deg--;  
                    }
                    else if (gnHeadElevationAngleSet_Deg < nManualHeadElevationSet_Deg)
                    {
                        gnHeadElevationAngleSet_Deg++;  
                    }  */
                    // Calculate current heading of the robot in degree and with respect to the global x-axis.
                    //fTemp = gnHeading*_TURN_DEG_PER_UNIT;
                    //gnCurrentHeadingDeg = fTemp + 90;                       // Approximate to integer, add 90 deg offset as 
                                                                                // angle is measured with respect to x-axis.
                    //while (gnCurrentHeadingDeg > 360)
                    //{
                    //    gnCurrentHeadingDeg = gnCurrentHeadingDeg - 360;
                    //}
                        
                    // Check obstacle sensors status and update robot status flag.
                    gobjRobotFlag.bSENL = 0;                            // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 0;
                    gobjRobotFlag.bSENR = 0;                        
                    if (gobjRobotState.nFrontObjDistance_mm < 75)
                    {
                        gobjRobotFlag.bSENM = 1;
                    }
                    if (gnMVMStatus == 1)                                   // Obstacle on left region.
                    {
                        gobjRobotFlag.bSENL = 1;                            // Update robot sensor status flags.
                    }
                    else if (gnMVMStatus == 2)                              // Obstacle on right region.
                    {
                        gobjRobotFlag.bSENR = 1;
                    }
                    else if (gnMVMStatus == 3)                              // Obstacle in front.
                    {
                        gobjRobotFlag.bSENM = 1;
                    }
                    else if (gnMVMStatus == 4)                              // In front is blocked.
                    {
                        gobjRobotFlag.bSENL = 1;                            // Update robot sensor status flags.
                        gobjRobotFlag.bSENM = 1;
                        gobjRobotFlag.bSENR = 1;
                    }
                   
                    // If MVM sensor detects any obstacle cancel any impending forward movement.
                    if (gfOmegaWSet > 0.0)     // Check if robot platform is engaged in any forward linear movement,
                    {
                        if (gnMVMStatus > 0)        // Check MVM sensor for obstacle.
                        {
                            gnAudioTone[0] = 2;      // Sound a beep and cancel movement.
                            gnAudioTone[1] = 0;
                            //SetMovementStop();                        
                        }
                    }
                    
                    if (gunExperimentalMode == 1)
                    {
                        gunExperimentalMode = 0;                    
                        OSSetTaskContext(ptrTask, 110, 10*__NUM_SYSTEMTICK_MSEC);    // Next state = 110, timer = 10 msec.
                    }
                    else
                    {    
                        OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 100 msec.
                    }
                } // if (gnRobotMoveMode == _ROBOT_MOVE_AUTO)
            break;
                
            case 5: // State 5 - Check MVM sensor and IR distance sensor status and take action.
                if (gobjRobotFlag.bCOLLISION == 1)                              // Check for collision.
                {
                    SetMovementStop();
                    OSSetTaskContext(ptrTask, 11, 500*__NUM_SYSTEMTICK_MSEC);   // Next state = 11, timer = 500 msec.
                    break;
                }
                
                if (gnMVMStatus == 1)                                           // Obstacle on left region.
                {
                    gobjRobotFlag.bSENL = 1;                                    // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 0;
                    gobjRobotFlag.bSENR = 0;
                    
                    unClearCounter = 0;
                    gfOmegaWSet = _VELOCITY_MOVE_SLOW;
                    nSetMovementTurn(-5);                                       // Turn right a bit.
                    OSSetTaskContext(ptrTask, 5, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 5, timer = 100 msec.
                }
                else if (gnMVMStatus == 2)                                      // Obstacle on right region.
                {
                    gobjRobotFlag.bSENL = 0;                                    // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 0;
                    gobjRobotFlag.bSENR = 1;
                                        
                    unClearCounter = 0;
                    gfOmegaWSet = _VELOCITY_MOVE_SLOW;
                    nSetMovementTurn(5);                                        // Turn left a bit.
                    OSSetTaskContext(ptrTask, 5, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 5, timer = 100 msec.
                }
                else if ((gnMVMStatus == 3) || (gobjRobotState.nFrontObjDistance_mm < 75))  // Obstacle in front.
                {
                    gobjRobotFlag.bSENL = 0;                                    // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 1;
                    gobjRobotFlag.bSENR = 0;
                    
                    unClearCounter = 0;
                    SetMovementStop();
                    nSetMovementTurn(-10);                                      // Turn right more.
                    OSSetTaskContext(ptrTask, 6, 500*__NUM_SYSTEMTICK_MSEC);    // Next state = 6, timer = 500 msec.
                }
                else if (gnMVMStatus == 4)                                      // In front is blocked.
                {
                    gobjRobotFlag.bSENL = 1;                                    // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 1;
                    gobjRobotFlag.bSENR = 1;
                    
                    unClearCounter = 0;
                    SetMovementStop();
                    OSSetTaskContext(ptrTask, 6, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 6, timer = 100 msec.
                }
                else                                                            // No obstacle, keep moving.
                {
                    gobjRobotFlag.bSENL = 0;                                    // Update robot sensor status flags.
                    gobjRobotFlag.bSENM = 0;
                    gobjRobotFlag.bSENR = 0;
                    
                    gfOmegaWSet = _VELOCITY_MOVE_NORMAL;
                    unClearCounter++;
                    if (unClearCounter > 15)
                    {
                        unClearCounter--;           // Decrement by 1 so that unClearCounter is always at 15.
                        gnHeadElevationAngleSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT - 4; // Look up a bit (to look further).
                    }
                    else
                    {
                        gnHeadElevationAngleSet_Deg = _HEAD_ELEVATION_ANGLE_DEFAULT + 4; // Look down a bit (to look nearer).
                    }
                    // Lightly steer the robot towards the preferred direction.
                    //fTemp = gnHeading*_TURN_DEG_PER_UNIT;
                    //gnCurrentHeadingDeg = fTemp + 90;                                   // Approximate to integer, add 90 deg offset as 
                                                                                    // angle is measured with respect to x-axis.
                    //while (gnCurrentHeadingDeg > 360)
                    //{
                    //    gnCurrentHeadingDeg = gnCurrentHeadingDeg - 360;
                    //}
                    
                    if (gnCurrentHeadingDeg > gnPreferHeadingDeg)
                    {
                        gnHeadingSet--;
                    }
                    else if (gnCurrentHeadingDeg < gnPreferHeadingDeg)
                    {
                        gnHeadingSet++;
                    }
                    
                    if (gnRobotMoveMode == _ROBOT_MOVE_AUTO)                        // Make sure it is still autonomous mode.
                    {
                        OSSetTaskContext(ptrTask, 5, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 5, timer = 100 msec.
                    }
                    else
                    {
                        SetMovementStop();                                          // No longer autonomous mode, stop all movements.
                        OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 100 msec.
                    }
                }
                break;
                
            case 6: // State 6 - Calculate current heading in degree with respect to x-axis.
                //fTemp = gnHeading*_TURN_DEG_PER_UNIT;
                //gnCurrentHeadingDeg = fTemp + 90;                                   // Approximate to integer, add 90 deg offset as 
                //                                                                    // angle is measured with respect to x-axis.
                //while (gnCurrentHeadingDeg > 360)
                //{
                //    gnCurrentHeadingDeg = gnCurrentHeadingDeg - 360;
                //}
                OSSetTaskContext(ptrTask, 7, 1);                                    // Next state = 7, timer = 1.
                break;
                
            case 7: // State 7 - Obstacle detected in front, part 1.
                    // Turn to avoid obstacle.
                
                if (gobjRobotFlag.bCOLLISION == 1)                              // Check for collision.
                {
                    SetMovementStop();
                    OSSetTaskContext(ptrTask, 11, 500*__NUM_SYSTEMTICK_MSEC);   // Next state = 11, timer = 500 msec.
                    break;
                }
                
                if ((gnMVMStatus > 0) || (gobjRobotState.nFrontObjDistance_mm < 75))  // Check if either IR distance sensor or vision 
                                                                                // sensor is triggered.
                {
                    nTemp = gnCurrentHeadingDeg - gnPreferHeadingDeg;
                    if (nTemp > 0) // gnCurrentHeadingDeg > gnPreferHeadingDeg
                    {
                        if (nTemp < 180)
                        {
                            nSetMovementTurn(-15);                              // Turn right.
                        }
                        else
                        {
                            nSetMovementTurn(15);                              // Turn left.
                        }
                    }
                    else
                    {
                        if (nTemp < -180) // gnCurrentHeadingDeg < gnPreferHeadingDeg
                        {
                            nSetMovementTurn(15);                               // Turn left.
                        }
                        else
                        {
                            nSetMovementTurn(-15);                              // Turn right.
                        }
                    }
                    OSSetTaskContext(ptrTask, 6, 500*__NUM_SYSTEMTICK_MSEC);    // Next state = 6, timer = 500 msec.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 5, timer = 100 msec.
                }
                break;
                
            case 11: // State 11 - Collision detected, Reverse a bit.
                gfOmegaWSet = -_VELOCITY_MOVE_SLOW;
                OSSetTaskContext(ptrTask, 12, 500*__NUM_SYSTEMTICK_MSEC);       // Next state = 12, timer = 500 msec.
                break;
                
            case 12: // State 12 - Collision detected, stop.
                SetMovementStop();
                gnRobotMoveMode = _ROBOT_MOVE_MANUAL;                
                OSSetTaskContext(ptrTask, 13, 1500*__NUM_SYSTEMTICK_MSEC);       // Next state = 13, timer = 1500 msec.
                break;
                
            case 13: // State 13 - Collision detected, restart routine.
                gobjRobotFlag.bCOLLISION = 0;
                OSSetTaskContext(ptrTask, 1, 10*__NUM_SYSTEMTICK_MSEC);        // Next state = 1, timer = 10 msec.
                break;
                
            case 100: // State 100 - Get up from rest. 
                PIN_PSW = _ON_ANALOG_POWER;                                     // Turn on high voltage analog supply if it is not turned on.
                SetLWheelProperty(_REVERSE, -1.0);                              // Set both wheels to rotate in reverse direction
                SetRWheelProperty(_REVERSE, -1.0);                              // at low torgue.
                gobjDriverDAC.unDAC2_mV = 870;          
                gobjDriverDAC.unDAC1_mV = 870;      
                OSSetTaskContext(ptrTask, 101, 220*__NUM_SYSTEMTICK_MSEC);      // Next state = 101, timer = 220 msec.
                break;
                
            case 101: // State 101 - Get up from rest. 
                SetLWheelProperty(_STOP, -1.0);                                 // Stop both wheels, return to 
                SetRWheelProperty(_STOP, -1.0);                                 // normal operation.
                OSSetTaskContext(ptrTask, 0, 1000*__NUM_SYSTEMTICK_MSEC);       // Next state = 0, timer = 1000 msec.
                break;
                
            case 110: // State 110 - Park: Move backward first. 
                gfOmegaWSet = -_VELOCITY_MOVE_NORMAL;  
                OSSetTaskContext(ptrTask, 111, 1100*__NUM_SYSTEMTICK_MSEC);     // Next state = 111, timer = 1100 msec.
                break;
                
            case 111: // State 111 - Park: Disable active balancing.
                gnRobotBalance = _DISABLE;
                //OSSetTaskContext(ptrTask, 112, 1*__NUM_SYSTEMTICK_MSEC);       // Next state = 111, timer = 1 msec.
                OSSetTaskContext(ptrTask, 115, 2000*__NUM_SYSTEMTICK_MSEC);       // Next state = 111, timer = 2000 msec.
                break;
                
            case 112: // State 112 - Park: Stop both motors.               
                SetLWheelProperty(_STOP, -1.0);                                 // Stop both wheels, return to 
                SetRWheelProperty(_STOP, -1.0);                                 // normal operation.
                OSSetTaskContext(ptrTask, 113, 100*__NUM_SYSTEMTICK_MSEC);      // Next state = 113, timer = 100 msec.
                break;
                
            case 113: // State 113 - Park.
                SetLWheelProperty(_FORWARD, -1.0);                              // Set both wheels to rotate in forward direction
                SetRWheelProperty(_FORWARD, -1.0);                              // at mid-torgue.
                gobjDriverDAC.unDAC2_mV = 870;          
                gobjDriverDAC.unDAC1_mV = 870;      
                OSSetTaskContext(ptrTask, 114, 1500*__NUM_SYSTEMTICK_MSEC);      // Next state = 114, timer = 500 msec.
                break;           
                
            case 114: // State 114 - Park.
                SetLWheelProperty(_STOP, -1.0);                                 // Stop both wheels, return to 
                SetRWheelProperty(_STOP, -1.0);                                 // normal operation.
                gnRobotMode = _ROBOT_MODE_IDLE;                                 // Set robot's default operation mode.
                OSSetTaskContext(ptrTask, 115, 500*__NUM_SYSTEMTICK_MSEC);      // Next state = 115, timer = 500 msec.                
                break;
                
            case 115: // State 115 - 
                SetSoundPattern(gunSoundToppled,2);                             // Generate a string of tones.
                OSSetTaskContext(ptrTask, 0, 1500*__NUM_SYSTEMTICK_MSEC);       // Next state = 0, timer = 1500 msec.              
                break;
            
            default:
                gunOSError = gunOSError | 0x0001;   // Set OS error bit 0.
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;            
        }
    }
}

///
/// Function name	: nPrintln
///
/// Author          : Fabian Kung
///
/// Last modified	: 6 Sep 2016
///
/// Code version	: 0.90
///
/// Processor		: Generic
///
/// Description:
/// 1. Prints data to the remote host via wireless link as human-readable ASCII text
/// followed by a carriage return character (ASCII 13, or '\r') and a newline 
/// character (ASCII 10, or '\n').
/// 2. Requires that a wireless link using serial port protocol has been formed with a 
/// remote PC host (or similar class of devices).
/// 3. Maximum string length is 50 characters excluding the newline character.
///
/// Arguments:
/// ptrText = Address of character string.
/// nTextLength = Length of text string exclusive of the newline character (0xD).
/// Return:
/// The number of bytes send.
/// Usage example:
/// nPrintln("Hello",5);
///
/// or
/// char *ptrChar;
///
/// ptrChar = "Hello";
/// nPrintln(ptrChar,5);

int nPrintln(char *ptrText, int nTextLength)
{
    gptrSystemText = ptrText;                       // Assign string address and length.
    gnSystemTextLength = nTextLength;
    if (gobjDriverHC05.nRFLinkState == 1)           // Make sure a PC type remote device is connected
    {                                               // via wireless link.
        gobjDriverHC05.nEnRFTxPeriodicData = 2;     // Enable periodic data transmission, set ASCII packet.
        return nTextLength;
    }   
    else
    {
        return 0;
    }
} 

 
 