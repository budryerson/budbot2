/* File Name: bb_motor.h
 * First Day: 25NOV14
 * Developer: Bud Ryerson
 * Last Work: 6SEP2015
 * Described: Motor routines header file containing
 *            declarations that define the bb_motor class.
 *            This was developed from the BB2_Motor files.
 * 6SEP15 - changed polarity of rear motors.
 * DO NOT USE VERSIONS BEFORE 6SEP15
*/

#include <Arduino.h>   //  It's like a bellybutton

#ifndef BB_MOTOR_H
#define BB_MOTOR_H

class bb_motor {
    public:
        bb_motor();
        ~bb_motor();
        void stop();
        void motorSetup();
        void control();
        void motorReset();
        void motorTest();
        void readMotorCurrent();
        void printData();
};

#endif

