/* File Name: bb_motor.cpp
 * First Day: 25NOV2014
 * Developer: Bud Ryerson
 * Described: Part of the library for for Budbot2
 *            This controls the Budbot2 motors
 *            Also formats motor data for return
 *            to the joystick controller module.
 * Last Work: 07AUG2015 - brought "follow" routine over from main loop
 * 29AUG2015 - changed motor speed from direct set to "chase the target"
 * 30AUG - changed it back again - too slow
 * 6SEP15 - changed polarity of rear motors.
 * DO NOT USE VERSIONS BEFORE 6SEP15
 * 3OCT15 - modified motStruct to create Left and Right structures
 *         then consolidated speed calculation to use pointers
 * 28OCT15 - included bb_PID library to create two PID controllers
 *         and pointers to the controllers to use in Motor Speed Calculation
 * 07NOV15 - New concept in joystick control: Y value provides amount of thrust
 *           and X value indicates rate of rotation. Try to modify Y with X
 *           to get robot to turn smoothly in the proper direction
 * 20FEB16 - added code to set PID controllers OFF if Motors are OFF
 *           otherwise sending setPoit info to contorllers with no inPut
 *           sets ITerm way out of whack
 *  9JUL16 - relocated motor current sensing to bb_Alarm library
 *           relocated Motor library to be under Control library
 */

/*  Find target pulse width (pcR) for any given speed (mSpdR) according to the sacred formula:
         pulseWidth = encWidMin * pow( 2, ( 8 - ( log( (*mDat).mSpd + 1) / log( 2))))
    The fastest speed produces the shortest pulse (encWidMin), which in this case is 1838ms.
    -------------------------------
          Speed  Pulse    Formula
    2^8 = 256 =   1838  = 1838 * 1
    2^7 = 128 =         = 1838 * 2
    2^6 =  64 =         = 1838 * 4
    2^5 =  32 =         = 1838 * 8
    2^4 =  16 =         = 1838 * 16
    2^3 =   8 =         = 1838 * 32
    2^2 =   4 =         = 1838 * 64
    2^1 =   2 =         = 1838 * 128
    2^0 =   1 =  470528 = 1838 * 256

    Motor Structure
-------------------------------------------------------
    unsigned long epv;  // Encoder pulse width average
    float epCalc;       // Encoder pulse width calculated
    unsigned long eps;  // Encoder pulse count saved
    int  mSpd;          // Motor Speed
    int  mSpdTarget;    // Motor Speed Target
    int  mSpdOffset;    // Motor Speed Offset
    bool mDir;          // Motor Direction: true = FORWARD, false = REVERSE
*/

//  Compile this entire file only for the MEGA2560
//  Some motor pins (A8, A9, etc.) are not on ProMini
#ifdef __AVR_ATmega2560__

#include <Arduino.h>     //  Every damn place you go.

#include <bb_defines.h>  //  Needed by setMotorPins
#include <bb_shared.h>
#include <bb_routines.h>
#include <bb_compass.h>
#include <bb_motor.h>    //  Declarations for this library

#include <bb_encoder.h>
bb_encoder bbEncoder;    // instantiate the two Encoder ISRs and routines
#include <bb_pid.h>
bb_pid bbPidMR, bbPidML; // instantiate two separate PID controllers
                         // one for Right and another for Left Motors
bb_motor::bb_motor(){}
bb_motor::~bb_motor(){}

//  Every Motor Array is ordered: Right Front, Left Front, Right Rear, Left Rear
const byte MSpin[ 4] = { RFMSpin, RRMSpin, LFMSpin, LRMSpin};  //  Motor Speed pins
const byte MDpin[ 4] = { RFMDpin, RRMDpin, LFMDpin, LRMDpin};  //  Motor Direction pins
//const byte MCpin[ 4] = { RFMCpin, LFMCpin, RRMCpin, LRMCpin};  //  Motor Current pins

// Stop all motors and clear Motor Mode
void bb_motor::stop()
{
    for( int x = 0; x < 4; x++)
    {
        digitalWrite( MDpin[ x], 1);  // set all direction pins to 1 (forward)
        analogWrite( MSpin[ x], 0);   // set all motor speed pins to ZERO.
    }
//    fbCLR( fbMotor);
}

void bb_motor::setup()
{
    for( int x = 0; x < 4; x++)   //  set motor direction, speed and current pins
    {
        pinMode( MDpin[ x], OUTPUT);    // Direction pins
        digitalWrite( MDpin[ x], 1);    //  set all direction pins to 1 (forward)
        pinMode( MSpin[ x], OUTPUT);    // Speed pins
        analogWrite( MSpin[ x], 0);     //  set all speed pins to 0 (stopped)
    }

    memset( &motDatR, 0, sizeof( motDatR));   //  clear motor data structures
    memset( &motDatL, 0, sizeof( motDatL));

    bbEncoder.encoderSetup();   //  setup the Encoders in "bb_encoder.h" file

    //  Set parameters for the PID Controller
    //  Kp, Ki, Kd, Min & Max inPut, Min & Max outPut
    bbPidMR.setupPID( 0.15, 0.04, 0.05, 0, 255, 0, 100);
    bbPidML.setupPID( 0.15, 0.04, 0.05, 0, 255, 0, 100);
}

//=========================== Set motor speed and direction ==============================

int motorSpeedCalculate( struct motStruct* mDat, bb_pid* mPid)
{
    (*mPid).setPoint = (float)abs((*mDat).mSpd);  //  setPoint is the commanded speed

    //  PID Controller inPut is the actual speed (0-255) of a motor.  This figure
    //  is the binary anti-log (power of 2) derived from the reciprocal of the
    //  binary logarithm of the fractional value of the average of the last eight
    //  measured Encoder pulse widths (epv) divided by the maximum pulse width.
    //  Subtract 1 from the resulting range of 1 - 256 to get 0 - 255.
    (*mPid).inPut = pow( 2, ( 8 - ( log( (float)(*mDat).epv / encWidMin) / log( 2)))) - 1;

    (*mPid).Compute();        //  Tell PID controller to compute an output
//    if( (*mPid).setPoint <= 0) (*mPid).outPut_f = 0.0;  //  Clamp the outPut if setPoint is zero
//    (*mDat).epCalc = (*mPid).outPut_f;              //  save in mDat as "encoder pulse calculated"
//    (*mDat).mSpdTarget = (int)(*mPid).outPut;

    if( (*mPid).setPoint <= 0) (*mDat).mSpdTarget = 0;  //  Clamp the outPut if setPoint is zero
    else (*mDat).mSpdTarget = (*mPid).outPut;                //  set target speed as integer value
    intConstrain( (*mDat).mSpdTarget, 0, 255);          //  and constrain
    (*mDat).mDir = !( (*mDat).mSpd < 0);  //  Motor direction TRUE (Forward) if mSpd not less than zero
    intConstrain( (*mDat).mSpd, -255, 255);
    return (*mDat).mSpd;
}

void bb_motor::control()
{
    //  modify motor velocity value with rotational value
    motDatR.mSpd = rDat1.velocity - rDat1.rotation;
    //  NOTICE!! This is a plus +!
    motDatL.mSpd = rDat1.velocity + rDat1.rotation;
    
    bbEncoder.pulseAverage();  //  find average of last eight pulse durations for both encoders
                               //  save encoder counts in rDat1 structure as "intCount"
    motDatR.epv = eDatR.epv;   //  store those encoder pulse averages in the motor data structure
    motDatL.epv = eDatL.epv;

    bbPidMR.setControlMode( fbCHK( fbMotor));  // set Right PID control OFF if Motor flagBit is OFF
    rDat1.motorSpeedRight = motorSpeedCalculate( &motDatR, &bbPidMR);
#if( PID_MOTOR_RIGHT_CALIBRATE)
    //  This to tune the PID formula for the Right Motor only,
    //  Enable only if PID_CALIBRATE is set to (1), otherwise
    //  the following code will throw a compile error.
    bbPidMR.serialRX();
    bbPidMR.serialTX();
#endif
 
    bbPidML.setControlMode( fbCHK( fbMotor));    // set Left PID control OFF if Motor flagBit is OFF
    rDat1.motorSpeedLeft =  motorSpeedCalculate( &motDatL, &bbPidML);
#if( PID_MOTOR_LEFT_CALIBRATE)
    //  This is for tuning the PID formula for Left motor only,
    bbPidML.serialRX();
    bbPidML.serialTX();
#endif

// =-=-=-=-=-=-    apply calculated Motor values to motors   -=-=-=-=-=-=-=-=
    if ( fbCHK( fbMotor))   //  if Motor flagBit is TRUE, set the motor speed
    {
        // set Motor Direction
        digitalWrite( RFMDpin, motDatR.mDir);  //  TRUE (1) = Forward
        digitalWrite( RRMDpin, motDatR.mDir);  // FALSE (0) = Reverse
        digitalWrite( LFMDpin, motDatL.mDir);
        digitalWrite( LRMDpin, motDatL.mDir);
        // set Motor Speed
        analogWrite( RFMSpin, motDatR.mSpdTarget);    // set Right Front
        analogWrite( RRMSpin, motDatR.mSpdTarget);    // set Right Rear
        analogWrite( LFMSpin, motDatL.mSpdTarget);    // set Left Front
        analogWrite( LRMSpin, motDatL.mSpdTarget);    // set Left Rear
    }
    else             //  otherwise stop the motors
    { 
        for (int x = 0; x < 4; ++x)
        {
            digitalWrite( MDpin[ x], 1);  // set all direction pins to 1 (forward)
            analogWrite( MSpin[ x], 0);   // set all motor speed pins to ZERO.
        }
    }
//    readMotorCurrent();
//    printData();
}

void bb_motor::reset()
{
    rDat1.velocity = 0;
    rDat1.rotation = 0;
    //isMotorOn = false;   //  set Motor Run mode to OFF
    fbCLR( fbMotor);     //  clear the motorOn flagBit
}

void bb_motor::motorTest()
{
    reset();                           //  set all motors to forward at zero
    for (int x = 0; x < 4; ++x)        //  set all motor PWM pins to forward at zero
    {
        digitalWrite( MDpin[ x], 1);  // set all direction pins to 1 (forward)
        analogWrite( MSpin[ x], 0);   // set all motor speed pins to ZERO.
    }
    //  set right motors to half speed for 0.5 second and then stop
    for( int x = 0; x < 2; ++x) analogWrite( MSpin[ x], 128);   // set Right motor PWM pins to half speed.
        milliDelay( 500);
    for( int x = 0; x < 2; ++x) analogWrite( MSpin[ x], 0);     // set Right motor PWM pins to ZERO.
        milliDelay( 500);

    //  set left motors to half speed for 0.5 second and then stop
    for( int x = 2; x < 4; ++x) analogWrite( MSpin[ x], 128);   // set Left motor PWM pins to half speed.
        milliDelay( 500);
    for( int x = 2; x < 4; ++x) analogWrite( MSpin[ x], 0);   // set Left motor PWM pins to half speed.
        milliDelay( 500);
    reset();        //  motor reset
}

void bb_motor::printData()
{
    printf("Motor Data: ");
    printf( "Speed=% 4i |% 4i ", motDatR.mSpd, motDatL.mSpd);
    //printf( "Dir=% 2i |% 2i ", motDatR.mDir, motDatL.mDir);

    printf( "PID Out= %");
    printFloat( motDatR.epCalc, 2);
    printf( " | ");
    printFloat( motDatL.epCalc, 2);

    printf( " Pulse=% 7lu |% 7lu ", eDatR.epv, eDatL.epv);
    printf( "Offset=% 4i |% 4i ", motDatR.mSpdOffset, motDatL.mSpdOffset);
    printf( "Target=% 4i |% 4i ", motDatR.mSpdTarget, motDatL.mSpdTarget);
    printf( "Count=% 3u |% 3u ", rDat1.intCountR, rDat1.intCountL);
    printf( "\r\n");
}

#endif  // End of #ifdef __AVR_ATmega2560__