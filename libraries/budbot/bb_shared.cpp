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
  //  Servo Structure: azimuth (pan) servo data
  //  position, direction, pin nmumber, max angle, min angle, offset
  srvStruct srvDat;
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
roboStruct1 rDat1;    //  robot data structure returned to joystick platform
roboStruct2 rDat2;    //  extra robot data returned to joystick platform
roboStruct3 rDat3;    //  extra robot data NOT returned to joystick platform
radioBufferStruct rBuff, rBuf2;


