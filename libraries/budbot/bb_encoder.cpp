/*  FILE NAME: bb_encoder.cpp
    FIRST DAY: 23SEP2015
    LAST DATE:
    04NOV15 - set period of timer 3 to 0.05 seconds
              moved timer 3 ISR to main program
    18NOV15 - moved encoder ISR to pins 2 & 3 (INT 0 & 1)
              because we need pins 20 & 21 for I2C
    07AUG16 - moved Timer 3 register setup code to 'routines.cpp'
*/

//  Timer 3 labels do not exist in the Arduino IDE for the ProMini
//  Therefor this file is defined to * COMPILE ONLY FOR MEGA2560 *
#ifdef __AVR_ATmega2560__

/*  Rover 5 Encoder and Motor Specifications
        Motor speed: 8,500RPM @ 7.2V
        Motor stall current: 2.5A
        Output shaft stall torque: 10Kg/cm
        Gearbox ratio: 86.8:1
            4.166:1 motor -> encoder
            20.833:1 encoder -> wheel (4.166:1 & 5:1)
        Encoder type: Quadrature, hall effect magnetic
        Encoder resolution: 166.66 state changes per wheel rotation
        Encoder Speed: 272 state changes/sec at 8,500RPM
        Drive Wheel Speed: 1.632 rev/sec @ 7.2V
        Drive Wheel Circumference = 7.5" roughly (7.4 - 7.8)
        Rover 5 Speed: roughly 12in/sec (30.5cm/sec)
        Scope read: 189Hz fwd, 184Hz rev

        The same mSpd values produce different motor speeds
        due to differences over time in the applied voltage
        and mechanical differences in the motors themselves.
        
        clicks per T3 (50ms) * 20 = clicks/sec * 60 = clicks/min
        clicks per min / 166.66 = wheel rpm
*/
/*  The Dual Modes of velocity measurement:
    1) Counter - Measure number of  pulses at fast speed
    2) Timer  - Measure interval of pulses at slow speed
*/

#include <Arduino.h>           //  like a bellybutton

#include <bb_defines.h>        //  global budbot2 definitions
#include <bb_shared.h>         //  global budbot2 variables
#include <bb_routines.h>       //  used for 'longDiv()' routine
#include <bb_encoder.h>        //  header declarations for this file

bb_encoder::bb_encoder(){}     //  Class constructor defined
bb_encoder::~bb_encoder(){}    //  Class destructor defined

//  = = = =  Encoder Interrupt Service Routines (ISRs)  = = = =
//  Encoders count up whether wheels are turning backwards or forwards.
//  Pulse counter values (epc) never reset. At full speed (272/sec), 
//  unsigned long integers can hold about 180 days of counts.
//  = = = = = = = = =  ISR for Left encoder   = = = = = = = = =
void eiSR_L()
{
    eDatL.epti = micros();  //  get immediate time
    ++eDatL.epc;            //  increment Left encoder pulse count.
    eDatL.epf = true;       //  set Left motor turning flag
}
//  = = = = = = = = =  ISR for Right encoder  = = = = = = = = =
void eiSR_R()
{
    eDatR.epti = micros();  // mark the time of the interrupt
    ++eDatR.epc;            // increment Right encoder pulse count.
    eDatR.epf = true;       // set Right motor turning flag
}

void bb_encoder::encoderSetup()
{   //  Mega2560 board interrupts:
    //  0 = pin 2 - green wire
    //  1 = pin 3 - blue wire,
    //  2 = pin 21,
    //  3 = pin 20,
    //  4 = pin 19,
    //  5 = pin 18
    attachInterrupt( 0, eiSR_L, CHANGE);  // trigger int_0 on LF Motor Int state change
    attachInterrupt( 1, eiSR_R, CHANGE);  // trigger int_1 on RF Motor Int state change

    //  Timer 3 is set to interrupt every 50 milliseconds in 'routines.cpp'

    // initialize all values of both encoder structures
    memset( &eDatL, 0, sizeof( eDatL));     //  clear Left  encoder data structure
    memset( &eDatR, 0, sizeof( eDatR));     //  clear Right encoder data structure
}

//  Subtract the saved time (epts) of the last interrupt of the prior sequence
//  from the immediate time (epti) of the last interrupt of the current sequence
//  and divide by the number of interrupts since the last call of this routine
//  to get an average pulse duration time for both the Left and Right encoders.
//  eDatR & eDatL are Encoder Data Structures.  See "shared.h".
void bb_encoder::pulseAverage()
{
    if( eDatL.epf)    //  If Left encoder pulse flag true then motor turning
    {
        //  save counts since last Timer 3 interrupt (50ms)
        rDat1.intCountL = (byte)( eDatL.epc - eDatL.eps);
        //  calculate the average time value in microseconds per interrupt
        eDatL.epv = longDiv( ( eDatL.epti - eDatL.epts), rDat1.intCountL);
        eDatL.epts = eDatL.epti;   // save the micros() time of the last interrupt
        eDatL.eps = eDatL.epc;     // save the number of interrupts
        eDatL.epf = false;         // reset encoder pulse flag (epf)
    }
    else
    {
      eDatL.epv = encWidMax;         //  or return the maximum value
      rDat1.intCountL = 0;           //  and make sure count is zero
    }

    if( eDatR.epf)    //  If Right encoder pulse flag true then motor turning
    {
        //  save counts since last Timer 3 interrupt
        //  epc (count) and eps (saved) are both unsigned long integers
        rDat1.intCountR = (byte)( eDatR.epc - eDatR.eps);
        //  and calculate the average value
        eDatR.epv = longDiv( ( eDatR.epti - eDatR.epts), rDat1.intCountR);
        eDatR.epts = eDatR.epti;
        eDatR.eps = eDatR.epc;          //  save the current encoder pulse count
        eDatR.epf = false;
    }
    else
    {
      eDatR.epv = encWidMax;         //  or return the maximum value
      rDat1.intCountR = 0;           //  and make sure count is zero
    }
}

#endif  // End of #ifdef __AVR_ATmega2560__