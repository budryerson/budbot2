/* File Name: bb_servo.h
 * Described: declarations for 'bb_servo.cpp', the servo code file
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
        void irRead();     //  Read IR sensor and return range in centimeters.
        void sweep();      //  Move servo to new position
        void scanOnce();
        void test();
    private:
        bool firstScan;
        void servoPulse();
        void getTarget();  //  find the position of the nearest range value
        void printServoData();
};

#endif

