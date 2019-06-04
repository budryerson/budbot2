/* File Name: bb_servo.h
 * Described: declarations for 'bb_servo.cpp', the servo code file
     This controls setup, testing and control of the servo panning motion
     of the IR sensor as well as interpretation of data from the sensor.
 * Developer: Bud Ryerson
 * First Day: 13DEC15?
 */

#ifndef BB_SERVO_H
#define BB_SERVO_H

#include <Arduino.h>  // It is very important to always remember this!

class bb_servo
{
    public:
        bb_servo();
        ~bb_servo();

        void setup();
        void resetServo();
        void sweep();        // Pan servo back and forth
        void scanOnce();
   //     void checkScanTimer();
    private:
        bool firstScan;      // for 'scanOnce()' routine
        void servoSet();     // move servo to new position
        void tilt();
        void getTargets();   // get near and far distance values
        void printServoData();
        void servoTest();
};

#endif

