/* File Name: bb_control.h
 * Developer: Bud Ryerson
 * First Day: 31DEC15
 * Last Work: 31DEC15
 * Described: declarations for 'bb_control.cpp',
 *            the control routines library
 */

#ifndef BB_CONTROL_H    //  only compile one time
#define BB_CONTROL_H

#include <Arduino.h>    // Always remember this. It is very important.

class bb_control
{
    public:
        bb_control();
        ~bb_control();
        void setup();
        void control();
 //        bool isHalted();  // no motor movement for 5 seconds
 //       void joystick();
        void autoControl();
        void evade();
        void program();
        void controlReset();
 //      void getBear();
 //       void setRotation();
     private:
        bool firstAuto; //  Set true before first pass through Auto
                        //  Set false at first pass through Auto
                        //  Set true again when Auto flag falls
        bool firstEvade;
        bool firstProgram;
        int  programStep;
        int  evadeStep;
        void resetMotion();
        void setEvade();
        void setProgram();
};

#endif

