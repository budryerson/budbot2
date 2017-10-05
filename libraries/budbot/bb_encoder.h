/* File Name: bb_encoder.h
 * First Day: 23SEP14
 * Developer: Bud Ryerson
 * Last Work:
 * Described: Encoder routines header file containing
 *            declarations that define the bb_encoder class.
 *            This serve the interrupts in the main file
*/

#include <Arduino.h>   //  It's like a bellybutton
                       //  (Everybody's got one.)

#ifndef BB_ENCODER_H
#define BB_ENCODER_H

class bb_encoder{
    public:
        bb_encoder();      //  Class constructor declared
        ~bb_encoder();     //  Class destructor declared
        void encoderSetup();
        void pulseAverage();
};

#endif
