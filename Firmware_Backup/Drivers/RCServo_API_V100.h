// Author			: Fabian Kung
// Date				: 5 June 2016
// Filename			: RCServo_API_V100.h

#ifndef _RCSERVO_API_dsPIC33E_H
#define _RCSERVO_API_dsPIC33E_H

// Can be used for 8 or 10 motors.
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



// Motor rotational speed
#define     _MOTORSPEED_SLOW                0
#define     _MOTORSPEED_MEDIUM              1
#define     _MOTORSPEED_FAST                2
#define     _MOTORSPEED_VERYFAST            3

int nSetRCServoMotor(int, int, int);

#endif