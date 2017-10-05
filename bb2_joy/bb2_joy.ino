
/* File Name: bb2_joy.ino
 * Described: THIS IS THE TRANSMITTER
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
 */

#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#include <bb_defines.h>
#include <bb_shared.h>
#include <bb_routines.h>
#include <printf.h>

//  include BudBot libraries
#include <bb_joystick.h>
bb_joystick bbJoystick;
#include <bb_radio.h>
bb_radio bbRadio;

//  25ms Timer Counter 2A Interrupt Service Routine
//  TCNT2 is set to 12.5ms in 'routines.cpp'
bool t2Flag = false;
int t2Count = 0;
ISR( TIMER2_COMPA_vect)   //  Timer 2A ISR triggers every 12.5ms
{
    t2Flag = true;
    ++t2Count;
}

void setup()
{
    Serial.begin( 115200);   //  initialize the serial port
    printf_begin();          //  initialize the printf module
    SPI.begin();             //  initiate the SPI object for the Radio

    bbRadio.setupJoystickRadio();
    bbJoystick.setup();       //  initialize ProMini pins
    setupTimer2();            //  Setup Timer 3 is also in 'routines.cpp'    
    clearDataStructures();    //  clear all platform and joystick data structures
                              //  set boolean values to false
                              //  in 'routines.cpp'
}

void loop()
{
/* 
    bbRadio.joystickRadioAck();
    if( jDat.radio)
    {
      jDat.loopTime = (uint8_t)( millis() - jDat.joyClock);
      jDat.joyClock = millis();  // get the time
      ++jDat.joyLoop;         // increase the joystick loop counter

      bbJoystick.readData();   // Get joystick position data, check
                               // buttons and set jDat signal bits.
      bbJoystick.checkJoyButtons();
      printTimeData();

      //bbJoystick.putPaneData();    //  Send platform (rData) to the PC;
      //bbJoystick.getPaneData();    //  then get serial signal data from PC
      //  printAllData();              //  data struct in Hexs - 'routines.cpp'

      bbJoystick.showLights(); // Light LEDS to match rDat data flag bits
    }
}
*/

  //bbJoystick.checkJoyButtons();
  //  Every 12.5ms, the Timer2 Interrupt Service Routine sets tc2Flag TRUE,
  //  and advances the t2 counter.  The last statement of the loop resets
  //  the T2 flag to FALSE.
  if( t2Flag)
  {
    //  COM loop runs at start of every 3 count sequence.
    //  (3 * 12.5ms = 37.5ms)
    if( t2Count < 2)   // counter equals 1, run communication routine
    {
      jDat.loopTime = (uint8_t)( millis() - jDat.joyClock);
      jDat.joyClock = millis();  // get system time in milliseconds
      ++jDat.joyLoop;            // advance joystick loop counter

      bbJoystick.readData();   // Get joystick position data, check
                               // buttons and set jDat signal bits.
      bbJoystick.checkJoyButtons();
      //  printTimeData();

      bbJoystick.putPaneData();    //  Send platform (rData) to the PC;
      bbJoystick.getPaneData();    //  then get serial signal data from PC
      // printAllData();              //  data struct in Hexs - 'routines.cpp'

      //  Radio write joystick data packet (jDat) twice and
      //  read two platform packets (rDat) as acknowledgments.
      bbRadio.joystickRadioAck();
      bbJoystick.showLights(); // Light LEDS to match rDat data flag bits      
    }
    else if( t2Count < 3);    // counter equals 2, do nothing
    else if( t2Count < 4);    // counter equals 3, do nothing
    else t2Count = 0;         // counter equals 4 or more, reset
    t2Flag = false;           // reset timer 2 flag
  }
}

void printTimeData()
{
      printf( "jClk %010u", jDat.joyClock);  //   Joystick Clock in milliseconds
      printf( " | ");
      printf( "jt %03u", jDat.loopTime);  //   Joystick loop time
      printf( " | ");
      printf( "jRad: ");
      if( jDat.radio) printf( "True "); else printf( "False");
      printf( " | ");
      printf( "r1 %03u", rDat1.loTime);   //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "r2 %03u", rDat2.loTime);     //   Short Platform Clock in milliseconds
      printf( " | ");
      printf( "bLoop %04u", rDat2.loopTime);  //   Platform loop time
      printf( " | ");      
      printf( "jSync: ");
      if( jDat.sync) printf( "True "); else printf( "False");
      printf( "\r\n"); // carriage return/line feed
      
}
