/*  File Name: bb_shared.cpp
 *  Described: C++ definitions of common, global, shared variables
 *             for all parts of bb2
 *  Developer: Bud Ryerson
 *  Inception: 23 NOV 2014
 *  8MAR2016 - added program structure
 *  10FEB17 - removed all digital compass code
 */

#include <Arduino.h>    //  It is always important to include this!

#include <bb_defines.h> // global budbot2 definitions
#include <bb_shared.h>  // global budbot2 variables

//  Define these only for the Robot to save memory
#ifdef __AVR_ATmega2560__
  //  Eye Structure: Left and Right IR eye sensor data
  // irStruct irDat;
                //  Digital Compass data structure
                //dcStruct dcDat;
  //  Servo Structure: Pan servo data
  srvStruct srvDat = { SERVO_MID, 0, servoPin, SERVO_MIN, SERVO_MAX, servoOff};
  //  Encoder Structure: Right and Left encoder data
  encStruct eDatR, eDatL;
  //  Motor Structures: Left and Right motor data
  motStruct motDatR, motDatL;
  //  9DoF IMU Motion Shield data
  imuStruct imuDat;
  // shadow structures
  shadowStruct shad0, shad1, shad2;
#endif  // End of exclusive AVR_ATmega2560 definitions

//  global data structures
joyStruct jDat;       //  joystick data structure sent to robot platform
//radioDataStruct rxBuffer, txBuffer;
roboStruct1 rDat1;     //  robot data structure returned to joystick platform
roboStruct2 rDat2;    //  extra robot data returned to joystick platform
radioBufferStruct rBuf1, rBuf2;

//  global flag variable definitions
bool centerDrift = true;    //  TRUE = nudge servo toward straight and level

int evadeStep = 0;
int programStep = 0;
//int saveCourse = 0;
//int course = 180;    //  robot present course in compass degrees
int scale;             //  something for the servo