/*  File Name: bb_compass.h
 *  Developer: Bud Ryerson
 *  Described: Common, global, shared variables for all parts of budbot2
 *  Last Work: 28NOV2015 - added print compass values routine
 *    10FEB17 - removed all digital compass code
 *    28JUN17 - corrected and rewrote the dead reckoning formulas
                i.e. should have used 'heading' instead of 'course'
 */

#ifndef BB_COMPASS_H     //  'include guard' macro to prevent errors
#define BB_COMPASS_H     //  at compilation time from multiple calls

#include <Arduino.h>    //  It is always important to include this!
#include <bb_hmc5883.h>

class bb_compass
{
    public:
        bb_compass();
        ~bb_compass();
       
        //  'drDist' is the Dead Reckoning Distance traveled by robot
        //   in one T3 period (50ms). It is derived from an average of
        //   the two wheel encoder counts and the wheel circumference.
        float drDist;
        int deltaPosX;  // change in X and Y dead reckoning position
        int deltaPosY;  // these are shared by at least two routines

        void setupIMU();
        void testIMU();
        void getImuValues();        //  get Inertial Motion values from IMU
        void printImuValues();      //  test print IMU values
        void getPosition();         //  calculate dead reckoning values from encoder
        void printDeadReckValues(); //  test print dead reckoning values
};

#endif  // BB_COMPASS_H