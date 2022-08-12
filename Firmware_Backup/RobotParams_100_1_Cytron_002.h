 /* 
 * File         : RobotParams_100_1_Cytron_002.h
 * Author       : Fabian Kung
 * Date         : 15 March 2022
 * Description  : Constants for electromechanical parameters of the robot.
 *                For production V2P2 with 100:1 HP Micro-metal gear motors
 *                from Cytron Technologies and 42mm diamater rubber wheels.
 *                Serial number V2P2002
 */

#ifndef ROBOTPARAMS1_H
#define	ROBOTPARAMS1_H

#ifdef	__cplusplus
extern "C" {
#endif

    #define     _ROBOT_VERSION_V2P2     // Robot mechanical design version.    
    #define     _HEAD_SINGLE_DOE        // The number of degree of freedom in the head.
                
// ******************* TUNABLE PARAMETERS **************************************************
    
// --- Primary parameters: State-feedback coefficients and DC motors non-linear parameters ---
   
// Here fixed point format for representing real numbers, Q8.8 format is adopted
// for the coefficients.
    
    // Coefficients with emphasis on theta and d(theta)/dt, used during standing still and 
    // small angle turning.
    #define     _FP_DEFAULT_FP      0b1111100000000000 // -8.00, Q8.8 format.
    #define     _FKX_DEFAULT_FP     0b1111110100000000 // -3.00, Q8.8 format.
    #define     _FKV_DEFAULT_FP     0b1111110000000000 // -4.00, Q8.8 format.
    #define     _FCW_DEFAULT_FP     0b1111111110110010 // -0.30469, Q8.8 format.
    #define     _R_DEFAULT_FP       0b0000000000000011 // 0.01172, Q8.8 format.
    #define     _L_MOT_OFFSET_DEFAULT_FP    0b0100000000000000 // +64.00, Q8.8 format.
    #define     _R_MOT_OFFSET_DEFAULT_FP    0b0100000000000000 // +64.00, Q8.8 format.    

    /*
    #define     _FP_DEFAULT_FP      0b0000000000000000 // 0.00, Q8.8 format.
    #define     _FKX_DEFAULT_FP     0b0000000000000000 // 0.00, Q8.8 format.
    #define     _FKV_DEFAULT_FP     0b0000000000000000 // 0.00, Q8.8 format.
    #define     _FCW_DEFAULT_FP     0b0000000000000000 // 0.00, Q8.8 format.
    #define     _R_DEFAULT_FP       0b0000000000000000 // 0.00, Q8.8 format.
    #define     _MOS_DEFAULT_FP     0b0000001000000000 // +2.00, Q8.8 format.
    #define     _L_MOT_OFFSET_DEFAULT_FP    0b0010000000000000 // +32.00, Q8.8 format.
    #define     _R_MOT_OFFSET_DEFAULT_FP    0b0010000000000000 // +32.00, Q8.8 format.       
     */
// Tuning interval, represented in Q8.8 fixed point format and floating point format.    
    
    #define     _FP_INTERVAL_FP     0b0000000001000000 // +0.25, Q8.8 format.
    #define     _FP_INTERVAL        0.25               // +0.25, floating point format.
    #define     _FKX_INTERVAL_FP    0b0000000000001000 // +0.03125, Q8.8 format.
    #define     _FKX_INTERVAL       0.03125             // +0.03125, floating point format.
    #define     _FKV_INTERVAL_FP    0b0000000000001000 // +0.03125, Q8.8 format.
    #define     _FKV_INTERVAL       0.03125             // 0.03125, floating point format.
    #define     _FCW_INTERVAL_FP    0b0000000000000100 // +0.015625, Q8.8 format.
    #define     _FCW_INTERVAL       0.015625           // +0.015625, floating point format.
    #define     _R_INTERVAL_FP      0b0000000000000001 // +0.00390625, Q8.8 format.
    #define     _R_INTERVAL         0.00390625          // +0.0078125, floating point format.
    #define     _MOS_INTERVAL_FP    0b0000001000000000 // +2, Q8.8 format.
    #define     _MOS_INTERVAL       2                  // +2.00, floating point format.    
    
// ******************* CONSTANT PARAMETERS **************************************************
    
    // Coefficients with emphasis on x and v state parameters, used during linear movement
    // and large angle or continuous turning.
    //#define     _FP_STIFF         -4.50   
    #define     _FP_STIFF         -4.25 
    //#define     _FKX_STIFF        -3.75      
    #define     _FKX_STIFF        -3.50   
    #define     _FKV_STIFF        -4.50         
    //#define     _FCW_STIFF        -0.265625  
    #define     _FCW_STIFF        -0.2890625  
    
    // Coefficients with even more emphasis on theta and d(theta)/dt state parameters, used 
    // during standing still on uneven or soft surface.
    #define     _FP_SOFT         -6.75      
    #define     _FKX_SOFT        -0.875      
    #define     _FKV_SOFT        -1.40     
    #define     _FCW_SOFT        -0.46875  
    
// --- Secondary Coefficients for linear movement and turning motions ---      
    #define     _FP2_TURN_DEFAULT   8  
    #define     _FD2_TURN_DEFAULT   -24  
    #define     _FP2_TURNCONT_DEFAULT   100
    #define     _FI2_TURNCONT_DEFAULT   1
    #define     _CONT_TURN_SPEED_DEFAULT  1     // 
    #define     _CONT_TURN_SPEED_HIGH  2        //
    
    #define     _FP2_MOVE_DEFAULT   2           // PD secondary feedback loop for linear motion, update delay.
    #define     _FD2_MOVE_DEFAULT   0    

    #define     _VELOCITY_MOVE_VERYSLOW     0.26        // In rotation per second.
    #define     _VELOCITY_MOVE_SLOW         0.34  
    #define     _VELOCITY_MOVE_NORMAL       0.44
    #define     _VELOCITY_MOVE_FAST         0.55     
    #define     _VELOCITY_MOVE_VERYFAST     0.65
   
// --- Parameters for DC motors, motor drivers and wheels ---
    #define     _MOTOR_GEAR_RATIO           100         // 
    #define     _MOTOR_ENCODER_CPR          12          // Motor encoder continous pulse per rotation.
    #define     _MOTOR_QUAD_DIVIDE          8           // Division ratio for motor encoder pulse
    //#define     _MOTOR_QUAD_DIVIDE          4
    
    // For V2P2_002
    #define     _WHEEL_PITCH_MM             80          // Wheel pitch.
    #define     _WHEEL_DIAM_MM              44          // Wheel diameter
    #define     _TURN_DEG_PER_UNIT          0.660        // Rotation of wheel (in degree) per unit change of gnHeading.
    #define     _TURN_DEG_PER_100           66         // _TURN_DEG_PER_UNIT * 100 

    // 27 Feb 2017: Due to the structure of the quadrature encoder, it is more convenient to express the outputs of the quadrature encoder 
// in terms of no. of ticks and rotation velocity in rotation/sec.  Assuming the quadrature encoders are connected to wheels (or motors
// attached with wheels), these can be converted to linear distance and velocity traveled by the wheels using the constants defined above.
// In dynamical modeling it is more convenient to use the parameters v (linear velocity) and x (linear distance).  Thus the v and x can be
// converted to rotation velocity and angle of the wheel by the above constants and vice versa.

    #define     _NOTICKSROTATION            150     // This is no. of ticks per 1 complete rotation of each wheel, (_MOTOR_ENCODER_CPR*_MOTOR_GEAR_RATIO)/_MOTOR_QUAD_DIVIDE               
    // For V2P2_002
    #define     _KVTOKOMEGA_COEFF           0.13832 // This is 2xpix(Radius of wheel)             
    #define     _KXTOKNW_COEFF              0.00092 // This is [(Degree per tick)/180]xPIx(Radius of wheel)

    // 1) For this setup, each pulse from the motor quadrature encoder corresponds to
    //    0.4 degree [e.g. 360/(75x12) = 0.4].
    // 2) The pulses are then divide by _MOTOR_QUAD_DIVIDE = 8.  Thus each
    //    tick from the output of the divider corresponds to 3.2 degree (or 0.0559 radian)
    //    turn of the output shaft axle.
    // 3) The distance traveled by each wheel for 1/2 tick is thus = (0.0559/2) * 42 = 1.1739 mm.
    // 4) If each left and right wheels travel in the opposite direction for 1/2 tick,
    //    this will make the robot turn left or right at 1.1739/80 = 0.0146 radian.   
    // 5) So 1 unit change in tick or gnHeading will cause the robot to turn 0.0146 radian
    //    or 0.84 degree.

    #define     _MOTOR_DRIVER_MAX_mV            2000    // Max and minimum input to motor driver circuit.
    #define     _MOTOR_DRIVER_MIN_mV            -2000   // These are system indepedent.
    #define     _MOTOR_DRIVER_TURN_MAX_mV       550     // Typically this should be limited to 25% of maximum motor
    #define     _MOTOR_DRIVER_TURN_MIN_mV       -550    // driver voltage output.

// --- Other parameters ---     
    #define     _HEAD_AZIMUTH_ANGLE_DEFAULT         0
    //#define     _HEAD_ELEVATION_ANGLE_DEFAULT       0
    #define     _HEAD_ELEVATION_ANGLE_DEFAULT       20
    #define     _MOTOR_DRIVER_SETTING_DEFAULT       0x000000
                                                    // bit0 - Polarity of left motor driver. 0 = normal, 1 = inverse.
                                                    // bit1 = Polarity of right motor driver. 0 = normal, 1 = inverse.
    #define     __TEMPERATURE_SENSOR_AD22103__
#ifdef	__cplusplus
}
#endif

#endif	/* ROBOTPARAMS1_H */

