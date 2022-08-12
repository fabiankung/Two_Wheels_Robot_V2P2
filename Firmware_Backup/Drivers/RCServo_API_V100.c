// Filename		: RCServo_API_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 18 July 2016


// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\osmain.h"
// Include header file for driver used.  Here absolute path is used, to edit if one changes folder.
//include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_PWMServo_V100.h"
#include "C:\Users\User\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_PWMServo_V101.h"
// --- Local Functions ---
int nMotorSpeed(int);

///
/// Function name	: nSetupServoMotor
///
/// Author		: Fabian Kung
///
/// Last modified	: 25 Feb 2015
///
/// Code version	: 1.00
///
/// Processor		: Generic
///
/// Processor/System Resources
/// PINS		: As setup in Driver_PWMServo_V100 (8 motors) or V105 (10 motors)
///
/// MODULES		: 1. Driver_PWMServo_V100 or higher.
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Description		: This function sets the parameters used in the PWM driver for driving servo
///                       motors, e.g. Driver_PWMServo_V1xx.  It sets the pulse width and
///                       increment/decrement steps of each PWM output in usec, i.e. gunRCServoPW_us[]
///                       and gunPWStep_us[] registers.  It acts as an interface between the
///                       user routines and Driver_PWMServo_V1xx.
///                       Each servo motor is related to a joint on the robot, and is identified by
///                       an integer value.  Of course this integer value is mapped to a name.
///                       Associated with each joint is the joint angle, max & min limits for the
///                       joint angle, and how fast the servo motor transitions from the old joint
///                       angle to the new.  All these settings are handled by this routine.  The
///                       list of arguments below will make clear the concept.
///
/// Arguments		: nAngle = the angle of the angle in degree (with repect to the axis parallel to the motor
///                                or similar relative reference).
///                       nSpeed = the angular velocity of the motor.
///                                Note: At the moment as we are using servo motors, we cannot control to high level of
///                                accuracy the angular velocity, so instead we have levels, e.g. _MOTORSPEED_SLOW,
///                                _MOTORSPEED_MEDIUM, _MOTORSPEED_FAST, _MOTORSPEED_VERYFAST.
///                       nJoint = The corresponding joint, e.g.
///                                _MOTOR0_LEFT
///                                _MOTOR1_LEFT
///                                _MOTOR2_LEFT
///                                _MOTOR3_LEFT
///                                _MOTOR4_LEFT
///                                _MOTOR0_RIGHT
///                                _MOTOR1_RIGHT
///                                _MOTOR2_RIGHT
///                                _MOTOR3_RIGHT
///                                _MOTOR4_RIGHT
///
/// Return		: 0 if successful.
///                       1 if overlimit
///
/// Example of usage 1:  Set motor to specified angle.
/// nSetRCServoMotor(24,_MOTORSPEED_MEDIUM,_MOTOR1_RIGHT);
/// This sets the servo motor MOTOR1_RIGHT to +24 degree, and the transition
/// from current angle to +24 degree to proceed at medium speed (_MOTORSPEED_MEDIUM).
///
/// Example of usage 2: Turn off motor.
/// nSetRCServoMotor(-180, _MOTORSPEED_SLOW,_MOTOR1_RIGHT);
/// This will turn off servo motor _MOTOR1_RIGHT.  The speed parameter is ignored.
/// Typical servo motor normally turns between -100 to +100 degrees.

/// Numbering of servo motors with respect to the joint, this
/// depends on the wiring connection between the main controller
/// and the motors.
/// NOTE: To have a more meaning mapping of joint name to motor number, you can
/// customized the motor name in the header file.

#define     _MOTOR0_LEFT            0
#define     _MOTOR1_LEFT            1
#define     _MOTOR2_LEFT            2
#define     _MOTOR3_LEFT            3
#define     _MOTOR4_LEFT            8

#define     _MOTOR0_RIGHT           4
#define     _MOTOR1_RIGHT           5
#define     _MOTOR2_RIGHT           6
#define     _MOTOR3_RIGHT           7
#define     _MOTOR4_RIGHT           9

// Motor rotational speed.  These rotational speed
// settings will be mapped to actual increment/decrement
// rate of the motor angle, which is related to the
// control pulse width (see below).
#define     _MOTORSPEED_SLOW                0
#define     _MOTORSPEED_MEDIUM              1
#define     _MOTORSPEED_FAST                2
#define     _MOTORSPEED_VERYFAST            3

// Actual increment/decrement in control pulse width
// for speed setting for various servo motors.
#define     __SLOW_INTERVAL     4
#define     __MEDIUM_INTERVAL   8
#define     __FAST_INTERVAL     14
#define     __VERYFAST_INVERVAL 24


// --- Servo motor parameters ---
#define     __TOWERPRO_SG51R_GAIN   10 // 15usec/deg
#define     __TOWERPRO_SG51R_OFFSET   1500 // 1500 usec
#define     __TAHMAZOO_GAIN         9      // controller gain, 12 usec/deg
#define     __TAHMAZOO_OFFSET       1500 // 1500 usec

#define     __HITEC_HS35HD_GAIN     10 // 10usec/deg

#define     __HITEC_HS45HB_GAIN     10 // 10 usec/deg
#define     __HITEC_HS45HB_OFFSET   1400    // median pulse width, 1400 usec
#define     __HITEC_HS45HB_MIN_ANGLE -75
#define     __HITEC_HS45HB_MAX_ANGLE +75


#define     __HITEC_HS85MG_GAIN     10      // controller gain, 10 usec/deg
#define     __HITEC_HS85MG_OFFSET   1400    // median pulse width, 1400 usec
#define     __HITEC_HS85MG_MIN_ANGLE    -79 // Minimum output shaft angle.
#define     __HITEC_HS85MG_MAX_ANGLE    +79 // Maximum output shaft angle.

// Gain (in usec/degree) for various servo motors to relate

#define     __MOTOR0_LEFT_GAIN        __HITEC_HS45HB_GAIN 
#define     __MOTOR1_LEFT_GAIN        __HITEC_HS45HB_GAIN    
#define     __MOTOR2_LEFT_GAIN        __HITEC_HS45HB_GAIN
#define     __MOTOR3_LEFT_GAIN        __HITEC_HS85MG_GAIN
#define     __MOTOR4_LEFT_GAIN        __HITEC_HS45HB_GAIN

#define     __MOTOR0_RIGHT_GAIN        __HITEC_HS85MG_GAIN
#define     __MOTOR1_RIGHT_GAIN        __HITEC_HS85MG_GAIN
#define     __MOTOR2_RIGHT_GAIN        __HITEC_HS85MG_GAIN
#define     __MOTOR3_RIGHT_GAIN        __HITEC_HS85MG_GAIN
#define     __MOTOR4_RIGHT_GAIN        __HITEC_HS45HB_GAIN

// Control pulse width for 0 degree or neutral position in microseconds.
// These are values obtained after calibration.
#define     __MOTOR0_LEFT_OFF   __TOWERPRO_SG51R_OFFSET
#define     __MOTOR1_LEFT_OFF   __TAHMAZOO_OFFSET
#define     __MOTOR2_LEFT_OFF   __HITEC_HS45HB_OFFSET
#define     __MOTOR3_LEFT_OFF   __HITEC_HS85MG_OFFSET
#define     __MOTOR4_LEFT_OFF   __HITEC_HS45HB_OFFSET

#define     __MOTOR0_RIGHT_OFF  __HITEC_HS85MG_OFFSET
#define     __MOTOR1_RIGHT_OFF  __HITEC_HS85MG_OFFSET
#define     __MOTOR2_RIGHT_OFF  __HITEC_HS85MG_OFFSET
#define     __MOTOR3_RIGHT_OFF  __HITEC_HS85MG_OFFSET
#define     __MOTOR4_RIGHT_OFF  __HITEC_HS45HB_OFFSET

// Limits for joints, these depends on the mechanical constructions of the joints and motors.
#define     __MOTOR0_LEFT_MIN_ANG        __HITEC_HS45HB_MIN_ANGLE
#define     __MOTOR0_LEFT_MAX_ANG        __HITEC_HS45HB_MAX_ANGLE
#define     __MOTOR1_LEFT_MIN_ANG        __HITEC_HS45HB_MIN_ANGLE
#define     __MOTOR1_LEFT_MAX_ANG        __HITEC_HS45HB_MAX_ANGLE
#define     __MOTOR2_LEFT_MIN_ANG        __HITEC_HS45HB_MIN_ANGLE
#define     __MOTOR2_LEFT_MAX_ANG        __HITEC_HS45HB_MAX_ANGLE
#define     __MOTOR3_LEFT_MIN_ANG        __HITEC_HS85MG_MIN_ANGLE
#define     __MOTOR3_LEFT_MAX_ANG        __HITEC_HS85MG_MAX_ANGLE
#define     __MOTOR4_LEFT_MIN_ANG        __HITEC_HS45HB_MIN_ANGLE
#define     __MOTOR4_LEFT_MAX_ANG        __HITEC_HS45HB_MAX_ANGLE

#define     __MOTOR0_RIGHT_MIN_ANG       __HITEC_HS85MG_MIN_ANGLE
#define     __MOTOR0_RIGHT_MAX_ANG       __HITEC_HS85MG_MAX_ANGLE
#define     __MOTOR1_RIGHT_MIN_ANG       __HITEC_HS85MG_MIN_ANGLE
#define     __MOTOR1_RIGHT_MAX_ANG       __HITEC_HS85MG_MAX_ANGLE
#define     __MOTOR2_RIGHT_MIN_ANG       __HITEC_HS85MG_MIN_ANGLE
#define     __MOTOR2_RIGHT_MAX_ANG       __HITEC_HS85MG_MAX_ANGLE
#define     __MOTOR3_RIGHT_MIN_ANG       __HITEC_HS85MG_MIN_ANGLE
#define     __MOTOR3_RIGHT_MAX_ANG       __HITEC_HS85MG_MAX_ANGLE
#define     __MOTOR4_RIGHT_MIN_ANG       __HITEC_HS45HB_MIN_ANGLE
#define     __MOTOR4_RIGHT_MAX_ANG       __HITEC_HS45HB_MAX_ANGLE

int nSetRCServoMotor(int nAngle, int nSpeed, int nJoint)
{
    int nTemp, nState;

    nState = 0;                                     // Set error flag to normal.

    switch (nJoint)
    {
        case _MOTOR0_LEFT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR0_LEFT_MAX_ANG)          // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR0_LEFT_MAX_ANG;          // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR0_LEFT_MIN_ANG)
            {
                nAngle = __MOTOR0_LEFT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR0_LEFT_GAIN*nAngle;           // Compute the rotation angle.
            nTemp = nTemp + __MOTOR0_LEFT_OFF;      // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR1_LEFT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR1_LEFT_MAX_ANG)          // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR1_LEFT_MAX_ANG;          // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR1_LEFT_MIN_ANG)
            {
                nAngle = __MOTOR1_LEFT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR1_LEFT_GAIN*nAngle;           // Compute the rotation angle.
            nTemp = nTemp + __MOTOR1_LEFT_OFF;      // Compute the actual control pulse.
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR2_LEFT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR2_LEFT_MAX_ANG)     // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR2_LEFT_MAX_ANG;     // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR2_LEFT_MIN_ANG)
            {
                nAngle = __MOTOR2_LEFT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR2_LEFT_GAIN*nAngle;      // Compute the rotation angle.
            nTemp = nTemp + __MOTOR2_LEFT_OFF;       // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR3_LEFT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR3_LEFT_MAX_ANG)   // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR3_LEFT_MAX_ANG;   // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR3_LEFT_MIN_ANG)
            {
                nAngle = __MOTOR3_LEFT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR3_LEFT_GAIN*nAngle;    // Compute the rotation angle.
            nTemp = nTemp + __MOTOR3_LEFT_OFF;     // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
           break;

        case _MOTOR4_LEFT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR4_LEFT_MAX_ANG)   // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR4_LEFT_MAX_ANG;   // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR4_LEFT_MIN_ANG)
            {
                nAngle = __MOTOR4_LEFT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR4_LEFT_GAIN*nAngle;    // Compute the rotation angle.
            nTemp = nTemp + __MOTOR4_LEFT_OFF;     // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
           break;

        case _MOTOR0_RIGHT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR0_RIGHT_MAX_ANG)  // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR0_RIGHT_MAX_ANG;  // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR0_RIGHT_MIN_ANG)
            {
                nAngle = __MOTOR0_RIGHT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR0_RIGHT_GAIN*nAngle;   // Compute the rotation angle.
            nTemp = __MOTOR0_RIGHT_OFF - nTemp;   // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR1_RIGHT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR1_RIGHT_MAX_ANG)       // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR1_RIGHT_MAX_ANG;       // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR1_RIGHT_MIN_ANG)
            {
                nAngle = __MOTOR1_RIGHT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR1_RIGHT_GAIN*nAngle;         // Compute the rotation angle.
            nTemp = __MOTOR1_RIGHT_OFF - nTemp;         // Compute the actual control pulse.
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR2_RIGHT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR2_RIGHT_MAX_ANG)     // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR2_RIGHT_MAX_ANG;     // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR2_RIGHT_MIN_ANG)
            {
                nAngle = __MOTOR2_RIGHT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR2_RIGHT_GAIN*nAngle;       // Compute the rotation angle.
            nTemp = __MOTOR2_RIGHT_OFF - nTemp;       // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
            break;

        case _MOTOR3_RIGHT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR3_RIGHT_MAX_ANG)   // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR3_RIGHT_MAX_ANG;   // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR3_RIGHT_MIN_ANG)
            {
                nAngle = __MOTOR3_RIGHT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR3_RIGHT_GAIN*nAngle;    // Compute the rotation angle.
            nTemp = nTemp + __MOTOR3_RIGHT_OFF;    // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
           break;

        case _MOTOR4_RIGHT:
            if (nAngle == -180)
            {
                gunRCServoPW_us[nJoint] = 0;        // Turn off motor.
                break;
            }
            if (nAngle > __MOTOR4_RIGHT_MAX_ANG)   // Check if the set angle
            {                                       // is larger or smaller than
                nAngle = __MOTOR4_RIGHT_MAX_ANG;   // the allowable mechanical
                nState = 1;                         // limit of the hand.
            }                                       // Set error flag.
            else if (nAngle < __MOTOR4_RIGHT_MIN_ANG)
            {
                nAngle = __MOTOR4_RIGHT_MIN_ANG;
                nState = 1;
            }
            nTemp = __MOTOR4_RIGHT_GAIN*nAngle;    // Compute the rotation angle.
            nTemp = nTemp + __MOTOR4_RIGHT_OFF;    // Compute the actual control pulse
            gunRCServoPW_us[nJoint] = nTemp;
            gunPWStep_us[nJoint] = nMotorSpeed(nSpeed);
           break;

        default:
           break;
    }
    
    return nState;
}

// Function to set the output shaft speed of
// servo motor.
int nMotorSpeed(int nSpeed)
{
    switch (nSpeed)
    {
        case _MOTORSPEED_SLOW:
            return __SLOW_INTERVAL;
            
        case _MOTORSPEED_MEDIUM:
            return __MEDIUM_INTERVAL;

        case _MOTORSPEED_FAST:
            return __FAST_INTERVAL;

        default:
            return __VERYFAST_INVERVAL;
    }
}