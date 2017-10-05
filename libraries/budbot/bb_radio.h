/* File Name: bb_radio.h
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Last Work: 10MAY2015
 * Described: Radio routines header file containing
 *            declarations to define the bb_radio class

  - TX/RX data over a nRF24L01 Module, using the RF24 library.

    RF24  Color   ProMini   Mega
    --------------------------------------
    IRQ   Orange     2       -
    CE    Gray       9      49
    CSN   Violet    10      53
    MOSI  Green     11      51
    MISO  Yellow    12      50
    SCK   Blue      13      52

 */

#ifndef BB_RADIO_H   //  'include guard' macro to prevent
#define BB_RADIO_H   //  compilation errors from multiple calls

#include <Arduino.h> //  It is very important to remember this!

class bb_radio
{
    public:
        bb_radio();   //  library constructor
        ~bb_radio();  //  library destructor
        void setupJoystickRadio();
        void setupPlatformRadio();
        void joystickRadioAck();
        void platformRadioAck();
    private:
        uint32_t raTimer2;
};

#endif

