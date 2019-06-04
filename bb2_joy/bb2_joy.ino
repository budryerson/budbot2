/* File Name: bb2_joy.ino
 * Described: THIS IS THE SLAVE/RF RECEIVER
 * Developer: Bud Ryerson
 * Inception: 19APR2015
 * 03AUG15 - reworked the button debounce code
 * 13SEP15 - switched radio to use acknowledge packet
 *           rather than separate TX and RX pipes
 * 27MAY16 - synchronize with 50ms platform loop
 *           wait for rDat to complete before
 *           writing data to processing display
 * 03AUG16 - changed the name
 * 16AUG16 - made 'joystick' library exclusive to 'joy.ino'
 * 05NOV16 - reversed sense of Radio light. Now ON is okay.
 * 16JAN17 - jDat.flagBits became 'sigBits' or signal bits
 *           any bit 'true' toggles a platform flag bit
 * 11FEB17 - Severely rearranged commands in attempt
             to synchronize joystick with platform
   20JUL17 - Both running 50ms loops but NOT synchronized
   18NOV17 - Got rid of loop timer and replace with radio IRQ
             to synchronize the controller with the platform.
             The Loop runs free and services the radio upon request.
   15DEC17 - Modified joytick.cpp to send HEX numbers in effort
             to shorten serial transmission time.
   08MAR19 - Working in the coal mine.. going down, down, down.
             Trying to rehabilitate budbot2.
             Getting radio IRQs. Yippee!
 */

#include <Arduino.h>
#include <SPI.h>          //  nRF24 Radio SPI Communication
#include <printf.h>       //  printf is used everywhere
#include <nRF24L01.h>     //  nRF24 Radio Library
#include <RF24.h>
#include <RF24_config.h>

/* - - - - - - - - budbot libraries - - - - - - - */
#include <bb_defines.h>
#include <bb_shared.h>    // global variables
#include <bb_routines.h>  // global functions

#include <bb_joystick.h>   // load the joystick library
bb_joystick bbJoystick;    // initialize a jostick object
#include <bb_radio.h>      // load the radio library
bb_radio bbRadio;          // initalize a radio object
/* - - - - - - - - - - - - - - - - - - - - - - - - - */

//  = = = Joystick Radio Interrupt Service Routines (ISR) = = =
//  Set flag TRUE when radio service is requested
volatile bool radioFlag;
void jriSR()       // attached to Interrupt 0 below
{
   radioFlag = true;
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - */

void setup()
{
    Serial.begin( 115200);   //  initialize the serial port
    Serial.flush();          //  flush the serial buffer
    printf_begin();          //  initialize the printf module
    SPI.begin();             //  initiate the SPI object for the Radio

    bbRadio.setupJoystickRadio();
    bbJoystick.setup();       //  initialize ProMini pins
    clearDataStructures();    //  clear all platform and joystick data structures
                              //  and set boolean values to false - 'routines.cpp'
    jDat.mpxID = JDAT_ID;     //  set to 0x00CD or 0b11001101 - 'defines.h'

    attachInterrupt( 0, jriSR, FALLING);   // Attach joystick radio ISR
                                           // to Interrupt 0 (pin 2) going low.
    radioFlag = false;                     // wait for Radio INT 0 set set true
}

void loop()
{
      jDat.joyClock = millis(); // save system clock in milliseconds
      bbJoystick.readData();    // Get joystick position data, and test
                                // buttons to set jDat signal bits.

      //  If two rDat packets have been received and their loTime bytes agree
      //  (jDat.sync),then do 'putPaneData' and 'getPaneData' routines to and
      //  from the PC display Processing package
      if( jDat.sync)
      {
          bbJoystick.putPaneData();    //  Send platform (rData) to the PC;
          jDat.sigBits = 0;            //  Clear joystick signal bits
          bbJoystick.getPaneData();    //  then get serial signal data from PC
          digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
  
          jDat.sync = false;    // reset the sync flag
          jDat.radio1 = false;
          jDat.radio2 = false;
      }

      // If a radio IRQ occurs, then:
      // 1) Call the 'joystickRadioAck()' function to:
      //   a) read a platform data packets (rDat1) and send a
      //   joystick data packet (jDat) as acknowledgment (ACK);
      //   b) wait for a second platform data packet (rDat2)
      //   and acknowledg that;
      // 2) show the lights;
      // 3) reset the radio IRQ flag and
      // 4) return to Main Loop.
      if( radioFlag)
      {
          // I don't know why I changed from 'radioFlag' to
          // 'jDat.radioIRQ' so I'm toggling it and counting
          //  them up to 1000 for the hell of it.
          jDat.radioIRQ = !jDat.radioIRQ;
          if( jDat.rIrqCount < 1000) ++jDat.rIrqCount;
              else jDat.rIrqCount = 0;

          bbRadio.joystickRadioAck();  // Run the radio routine.
          bbJoystick.showLights();     // Light LEDS to match rDat flag bits.
          jDat.radioIRQ = false;
          radioFlag = false;           // Reset the radio IRQ flag.
      }

//    printAllData();    // print all data structs in Hex code - 'routines.cpp'
      ++jDat.joyLoop;      // advance joystick loop counter
      jDat.loopTime = (uint16_t)( millis() - jDat.joyClock);
//    printTimeData();
//    bbJoystick.printJoyData();
//    delay(20);
//    digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
//    bbJoystick.showLights();     // Light LEDS to match rDat flag bits.
}

//  This is for testing purposes
void printTimeData()
{
      printf( "jLoop %07u", jDat.joyLoop);
      printf( " | ");
      printf( "jClk %010u", jDat.joyClock);  //   Joystick Clock in milliseconds
      printf( " | ");
      printf( "jt %03u", jDat.loopTime);  //   Joystick loop time
      printf( " | ");
      printf( "jRad: ");
      if( jDat.radioIRQ) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "jRad1: ");
      if( jDat.radio1) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "jRad2: ");
      if( jDat.radio2) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "jSync: ");
      if( jDat.sync) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "r1 %03u", rDat1.loTime);   //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "r2 %03u", rDat2.loTime);     //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "bRadio %03u", rDat2.radioTime);  //   Platform loop time
      printf( " | ");
      printf( "bLoop %04u", rDat2.loopTime);  //   Platform loop time
      printf( "\r\n"); // carriage return/line feed
}
