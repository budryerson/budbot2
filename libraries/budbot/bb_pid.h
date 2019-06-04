/*  File Name: bb_pid.h
 *  Developer: Bud Ryerson
 *  Last Work: 21OCT2015
 *  Described: variable declarations for a simple
 *             Proportional Integral Derivative controller
 */

#ifndef BB_PID_H       //  'include guard' macro to prevent errors
#define BB_PID_H       //  at compilation time from multiple calls

#include <Arduino.h>   //  It is important to always include this!
#include <bb_defines.h>

/* setPoint =  calculated speed
   inPut    =  measured speed
   outPut   =  PWM value sent to motors
*/
class bb_pid
{

    public:
        bb_pid();
        ~bb_pid();
        //  Five Main PID variables
        float setPoint;
        float inPut;
        float outPut_f;
        int  outPut;

        void setupPID( float setP, float setI, float setD, 
                       float inPutMin, float inPutMax,
                       float outPutMin, float outPutMax);
        void Compute();
        void setTunings( float Kp, float Ki, float Kd);
        void setSampleTime( int NewSampleTime);
        void setControlMode( bool mode);
        void setControlDirection( uint8_t Direction);
        void serialRX();   //  Receive from Processing GUI
        void serialTX();   //  Send data to Processing GUI
        

        float pidOutMin, pidOutMax;
        float pidInMin, pidInMax;  // not implemented yet
        uint8_t  controlMode;
        uint8_t  controlDirection;

    private:
        bool  inAuto;
        float kp, ki, kd;           // 'setTunings' working variables
        float error, lastInPut;     // 'Compute' working variables
        float PTerm, ITerm, DTerm;  //  more 'Compute' working variables
        float SampleTime;           // 'setSampleTime' working variable

        void  resetPidVar();             //
        float minMax( float value);     // limit a float value

};

#endif