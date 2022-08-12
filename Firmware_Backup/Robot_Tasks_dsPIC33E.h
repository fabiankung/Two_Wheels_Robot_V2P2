// Author			: Fabian Kung
// Date				: 30 April 2022
// Filename			: Robot_Tasks_dsPIC33E.h
// Description      : Function prototypes declaration for Robot Tasks.

#ifndef _ROBOT_TASKS_dsPIC33E_H
#define _ROBOT_TASKS_dsPIC33E_H

void Robot_Sensor_MPU6050_CF(TASK_ATTRIBUTE *); // Complementary filter version.
void Robot_Sensor_MPU6050_KF(TASK_ATTRIBUTE *); // Kalman filter version
void Robot_Balance(TASK_ATTRIBUTE *);
void Robot_Proce1(TASK_ATTRIBUTE *);
void Robot_MoveTurn(TASK_ATTRIBUTE *);
void Robot_MoveLinear(TASK_ATTRIBUTE *);
void Robot_ExControllerMessageLoop(TASK_ATTRIBUTE *);
void User_Proce1(TASK_ATTRIBUTE *);
void User_ProceTest2(TASK_ATTRIBUTE *);
#endif