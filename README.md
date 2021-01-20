# Two-Wheels_Robot_V2P2
V2P2 is a small two-wheels self-balancing mobile robot platform. It contains a single-core on-board computer as the robot controller. The robot controller handles low-level process such as sampling and processing of sensors output, implementing discrete state feedback control to balance the robot body upright and perform secondary tasks etc. User can communicate with the robot controller using UART port, at 3.3V CMOS logic, baud rate at 57600 bps. A simple two-way protocols allow the user controller to send instructions to the robot controller to perform simple tasks, for instance asking the robot to move at constant linear speed, turn, drive RC servo motors, beep a sound and many more. Thus using this robot platform, in combination with an external controller of the user, an intelligent robot system can be constructed. 

In this repository, the following files are provided:
1. Getting started guide.
2. Using Arduino Nano or Micro as the user controller, examples to communicate with the Robot Controller.
3. Andriod APK file for apps to control the V2P2 robot platform directly via Bluetooth.
4. 3D STL files of the mechanical design.

More details will be disclosed in future.
