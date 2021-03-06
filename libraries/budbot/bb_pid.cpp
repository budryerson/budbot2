/*  File Name: bb_pid.cpp
 *  Developer: Bud Ryerson
 *  Creation:  21OCT15(?)
 *  Last Work: 31OCT2015 - add the Processing GUI
 *  Described: function and variable definitions for a simple
 *             Proportional Integral Derivative controller
 */

#include <Arduino.h>        //  It is important to always include this!
//#include <avr/pgmspace.h>   //  Used to access Flash Memory
#include <bb_defines.h>
#include <bb_routines.h>
#include <bb_pid.h>

#define MANUAL    0    //  Manual Mode = FALSE
#define AUTOMATIC 1    //  Auto Mode  =  TRUE

#define DIRECT    0    //  Proportion Direction = Direct
#define REVERSE   1    //  Proportion Direction = Inverse

bb_pid::bb_pid(){}
bb_pid::~bb_pid(){}

//  reset the main PID variables
void bb_pid::resetPidVar()
{ 
    ITerm = outPut_f;
    lastInPut = inPut;
    ITerm = minMax( ITerm);
    inAuto = false;
}

void bb_pid::setupPID( float setP, float setI, float setD,
     float inPutMin, float inPutMax, float outPutMin, float outPutMax)
{
    // public floats
    SampleTime = 50;  // 50ms. time base is set by Timer3 ISR  
    setPoint = 0;     //  setPoint is public and assigned in motor.cpp
    inPut = 0;
    pidInMin = inPutMin;
    pidInMax = inPutMax;
    pidOutMin = outPutMin;
    pidOutMax = outPutMax;

    //deltaInPut = 0;
    // private floats
    error = 0;        //  'Compute' working values are private
    lastInPut = 0;
    PTerm = 0;
    ITerm = 0;
    DTerm = 0;
    setControlDirection( DIRECT);   // 0 = DIRECT, 1 = REVERSE, else = ? error ?
    setControlMode( AUTOMATIC);     // 0 = MANUAL, 1 = AUTO, else = ? error ?
    setTunings( setP, setI, setD);  // Set tunings as determined by experiment
    
    inAuto = false;
}

float bb_pid::minMax( float value)
{
    if( value < pidOutMin) return pidOutMin;
        else if ( value > pidOutMax) return pidOutMax;
    return value;
}

//  Compute all the working error variables
void bb_pid::Compute()
{
    // If Auto is OFF, cancel the function.
    if( !inAuto) return;

    //  Try to guess the slope of change.  When setPoint is constant, use
    //  the derivative of the Input rather than the derivative of the Error.
    error = setPoint - inPut;       //  error = desired value - actual value

    PTerm = kp * error;

    ITerm += ( ki * error);
    ITerm = minMax( ITerm);       //  Constrain ITerm to min/max output values

    DTerm = kd * ( inPut - lastInPut); // DTerm = fraction of input change
    lastInPut = inPut;              //  save 'inPut' value for next time

    //-------------  Compute PID outPut  ----------------
    outPut_f = PTerm + ITerm - DTerm;    //  outPut as a float
    outPut_f = minMax( outPut_f);        //  constrain outPut
    outPut = floatToInt( outPut_f);      //  round to int in 'routines.cpp'
    //---------------------------------------------------
}

void bb_pid::setTunings( float Kp, float Ki, float Kd)
{
    // If any tuning is less than zerro, cancel the function
    if( Kp < 0 || Ki < 0 || Kd < 0) return;

//    float SampleTimeInSec = (float)( SampleTime / 1000);
    kp = Kp;
    ki = Ki;
    kd = Kd;

    if( controlDirection == REVERSE)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void bb_pid::setSampleTime( int NewSampleTime)
{
    if( NewSampleTime > 0)
    {
        float ratio = (float)NewSampleTime / (float)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void bb_pid::setControlMode( bool autoFlag)
{
    if( ( autoFlag == true) &&  // If switching from MANUAL to AUTO...
        ( inAuto == false))
    {
        resetPidVar();          // reset the PID variables.
    }
    inAuto = autoFlag;          // Save mode for next time
}

void bb_pid::setControlDirection( uint8_t Direction)
{
    controlDirection = Direction;
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/
#if( PID_CALIBRATE)     // Compile if set in "defines.h"

  union {               // This data structure allows
    byte asBytes[24];   // an array of 24 bytes and an
    float asFloat[6];   // array of six 4 byte floats to
  }                     // occupy the same memory space.
  foo;

  // Store the floating point values from Processing
  // as byte series indexed in the following format:
  // ------------------------------------------------------------
  //        0: 0 = MANUAL, 1 = AUTO, else = ? error ?
  //        1: 0 = DIRECT, 1 = REVERSE, else = ? error ?
  //    2 - 5: float setPoint
  //    6 - 9: float inPut
  //  10 - 13: float outPut
  //  14 - 17: float P_Param
  //  18 - 21: float I_Param
  //  22 - 25: float D_Param

  void bb_pid::serialRX()
  {
      // read the bytes sent from Processing
      int index = 0;
      byte Auto_Man = -1;
      byte Direct_Reverse = -1;
      while( Serial.available() && index < 26)
      {
          if( index == 0) Auto_Man = Serial.read();
              else if( index == 1) Direct_Reverse = Serial.read();
              else foo.asBytes[ index - 2] = Serial.read();
          index++;
      }

      // Parse the 26 character input string from Processing...
      if( index == 26  &&                              // if the length is correct...
        ( Auto_Man == 0 || Auto_Man == 1) &&           // and the first and second...
        ( Direct_Reverse == 0 || Direct_Reverse == 1)) // bytes are valid
      {
  //      setPoint = float( foo.asFloat[ 0]);    //  Sending setPoint and inPut values
  //      inPut = float( foo.asFloat[ 1]);       //  from Processing is not necessary

          if( Auto_Man == 0)                       //  In MANUAL mode, change the outPut
          {                                        //  or we'll get an outPut blip,
              outPut_f = float( foo.asFloat[ 2]);  //  and the controller will overwrite.
          }

          float p, i, d;                         //  Read in controller Tunings
          p = float(foo.asFloat[ 3]);
          i = float(foo.asFloat[ 4]);
          d = float(foo.asFloat[ 5]);

          setTunings( p, i, d);                   //  Set controller Tunings
          if( Auto_Man == 0) setControlMode( MANUAL);    //  Set controller Mode
              else setControlMode( AUTOMATIC);
          if( Direct_Reverse == 0) setControlDirection( DIRECT); //  Set controller Direction
              else setControlDirection( REVERSE);
      }
      Serial.flush();                           //  Clear random data from serial buffer
  }

  // Send type floats as strings to Processing
  // Processing will convert strings to floats
  void bb_pid::serialTX()
  {
      Serial.print("PID ");
      Serial.print(setPoint);
      Serial.print(" ");
      Serial.print(inPut);
      Serial.print(" ");
      Serial.print(outPut_f);
      Serial.print(" ");
      Serial.print(kp);
      Serial.print(" ");
      Serial.print(ki);
      Serial.print(" ");
      Serial.print(kd);
      Serial.print(" ");
      if( controlMode == AUTOMATIC) Serial.print("Automatic");
          else Serial.print("Manual");
      Serial.print(" ");
      if( controlDirection == DIRECT) Serial.println("Direct");
          else Serial.println("Reverse");
  }
#endif   // Serial Communication functions / helpers