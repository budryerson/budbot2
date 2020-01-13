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
   28OCT19 - Working at Fort Bud.
             Deleted all the radioIRQ stuff.
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
    Serial.flush();          //  flush the serial output buffer
    printf_begin();          //  initialize the printf module
    SPI.begin();             //  initiate the SPI object for the Radio

    bbRadio.setupJoystickRadio();
    bbJoystick.setup();       //  initialize ProMini pins
    clearDataStructures();    //  clear all platform and joystick data structures
                              //  and set boolean values to false - 'routines.cpp'
    jDat.mpxID = JDAT_ID;     //  set to 0x00CD or 0b11001101 - 'defines.h'

    attachInterrupt( 0, jriSR, FALLING);   // Attach joystick radio ISR
                                           // to Interrupt 0 (pin 2) going low.
    radioFlag = false;                     // Initialize flag to 'False'.
                                           // Gets set 'true' when INT 0 falls
}

void loop()
{
      jDat.joyClock = millis(); // save system clock in millis - uint32
      bbJoystick.readData();    // Get joystick position data, and test
                                // buttons to set jDat signal bits.

      //  If two rDat packets have been received and their loTime bytes agree
      //  (jDat.sync),then do 'putPaneData' and 'getPaneData' routines to and
      //  from the PC display Processing package.  Reset radio1 & 2 and sync.
      if( jDat.sync)
      {
          bbJoystick.putPaneData();    //  Send platform (rData) to the PC;
          jDat.sigBits = 0;            //  Clear joystick signal bits
          bbJoystick.getPaneData();    //  then get serial signal data from PC
          digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
  
          jDat.sync = false;    // reset the sync flag
         // jDat.radio1 = false;  // this stuff gets reset in 'joystickRadioAck()'
         // jDat.radio2 = false;
      }

      //  How can we be sure that panedata gets to and from the PC before the
      //  next set of rdata packets are ready?  And what do we do while we're
      //  waiting?
      
      // When a radio IRQ occurs, and sets 'radioFlag' to 'true':
      // 1) Call the 'joystickRadioAck()' function to:
      //  a) read a platform data packet (rDat1) and
      //  b) return a joystick data packet (jDat) as acknowledgment (ACK).
      //  c) Set radio1 'True,' wait for a second data packet (rDat2), and
      //  d) acknowledge that.
      //  e) Set radio2 'True', compare loTime bytes and
      //  f) Set sync 'True'.
      // 2) Show the joystick lights, set radioFlag 'false' and
      // return to the Main Loop.
      if( radioFlag)
      {
          bbRadio.joystickRadioAck();  // Run the radio routine.
          bbJoystick.showLights();     // Light LEDS to match rDat flag bits.
          radioFlag = false;           // Reset the radio IRQ flag.
          //jDat.radioIRQ = true;
      }

      ++jDat.joyLoop;      // advance joystick loop counter - uint32
      jDat.loopTime = (uint16_t)( millis() - jDat.joyClock);

//    printAllData();    // print all data structs in Hex code - 'routines.cpp'
//    printTimeData();
//    bbJoystick.printJoyData();
//    delay(20);
//    digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
//    bbJoystick.showLights();     // Light LEDS to match rDat flag bits.
}

//  This is for testing purposes
void printTimeData()
{
      printf( "Loop %07u", jDat.joyLoop);
      printf( " | ");
      printf( "Clk %010u", jDat.joyClock);  //   Joystick Clock in milliseconds
      printf( " | ");
      printf( "dT %03u", jDat.loopTime);  //   Joystick loop time
      printf( " | ");
      printf( "Radio1 ");
      if( jDat.radio1) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "Radio2 ");
      if( jDat.radio2) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "Sync ");
      if( jDat.sync) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "loT1 %03u", rDat1.loTime);   //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "loT2 %03u", rDat2.loTime);     //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "botRadT %03u", rDat2.radioTime);  //   Platform loop time
      printf( " | ");
      printf( "botLoopT %04u", rDat2.loopTime);  //   Platform loop time
      printf( "\r\n");      // carriage return/line feed
      delay(20);
}
