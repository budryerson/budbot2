 /*  File Name: bb2_bot.ino
 *  Described: THIS IS THE RECEIVER
 *  Developer: Bud Ryerson
 *  Inception: 23 NOV 2014
 */

#include <Arduino.h>       //  This is everywhere
#include <EEPROM.h>
#include <Wire.h>          //  BBO055 NDOF IMU I2C Communication
#include <SPI.h>           //  nRF24 Radio SPI Communication

#include <nRF24L01.h>      //  nRF24 Radio Library
#include <RF24.h>
#include <RF24_config.h>

#include <bb_shared.h>     //  global variables
#include <bb_routines.h>   //  and routines
#include <bb_notes.h>      //  musical notes and songs
#include <bb_alarm.h>      //  musical notes and songs
#include <printf.h>
#include <bb_servo.h>
bb_servo bbServo;
#include <bb_radio.h>
bb_radio bbRadio;
#include <bb_compass.h>
bb_compass bbCompass;
#include <bb_control.h>
bb_control bbControl;


//  25ms Timer Counter 3A Interrupt Service Routine
//  TCNT3 is set to 25ms in 'routines.cpp'
bool t3Flag = false;       //  Timer 3A flag
int t3Count = 0;
ISR( TIMER3_COMPA_vect)    //  Timer 3A ISR triggers every 50ms
{
    t3Flag = true;   //  set Timer3 Flag = TRUE
    ++t3Count;
}


#ifdef DEBUG
  //  This is a way to discover how much time the
  //  main loop takes.  It creates an array of ten
  //  loop times so that print time doesn't count.
  uint32_t loopTimes[ 10];   //  long integers
  uint8_t botLoop = 0;     //  byte
  void showLoopTimes()
  {
      if( botLoop < 10)    //  save ten loop times
      {
          loopTimes[ botLoop] = millis();
          botLoop++;
      }
      else                   //  then print them
      {
          for( uint8_t x = 0; x < 10; x++)
              printf( "Time: %lu\r\n", loopTimes[ x]);
          botLoop = 0;     //  and reset the counter
      }
  }
#endif

/* -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- */
void setup()
{
    Serial.begin( 115200);  //  this must be set to 9600 for MagViewer
    printf_begin();         //  initialize the 'printf' module
    SPI.begin();            //  initiate the SPI object for the Radio
    Wire.begin();           //  initiate the I2C object for the IMU
    delay(5);               //  wait at least 5ms before continuing

    //  setup output pins and initialize data structures
    bbRadio.setupPlatformRadio();   //  initialize robot radio acknowledge mode
    bbCompass.setupIMU();
    bbCompass.testIMU();

    pinMode( Speaker, OUTPUT);    //  set Speaker pin (D7) to OUTPUT
    bbServo.setup();      // set servo output pin OC4A
    bbServo.test();       //  be careful of obstacles
    bbControl.setup();    //  set joystick input pins and MODE LED output pins
                          //  setup obstacle and alarm pins and button pins
                          //  clear joystick and robot data structures

    setupADCregisters();   //  Custom Analog to Digital is in 'routines.cpp'
    setupTimer3();         //  Setup Timer 3 is also in 'routines.cpp'

    //playReadyMusic();      //  found in 'bb_tone' library
}

// = = = = = = = = MAIN PROGRAM LOOP = = = = = = = = =
// Get joystick values, run controls and retuirn data
void loop()
{
    //  if this is a restart then perform certain housekeeping
    //  sent to joystick: at botLooper = 1, botClock = 6376
    //  1.  back up if possible
    //  2.  turn if necessary
    //  3.  sweep IR sensor
    //  4.  turn again if necessary
    //  4.  halt
    //  5.  play music
    
    //  Every 12.5ms, the Timer3 Interrupt Service Routine sets tc3Flag TRUE.
    //  The last statement of this loop resets the T3 flag to FALSE.
    if( t3Flag)
    {
        //  Update bot clock and loTime
        rDat2.botClock = millis();    // save current bot system time
        rDat1.loTime = rDat2.loTime;  // copy low byte of system time
        ++rDat2.botLoop;              // increment robot loop counter
        digitalWrite( rBluPin, !digitalRead(rBluPin));    //  Toggle blue LED

        //  'checkRobotButtons()', 'fbCHK()' and all
        //   Flag Bit routines are in 'routines.cpp'
        checkRobotButtons();
        if( fbCHK( fbRoBut1)) bbCompass.setupIMU();  // Test and reset IMU.
        if( fbCHK( fbRoBut2)) fbSET( fbProgram);     // Do Program routine:
        if( fbCHK( fbRoBut3)) fbSET( fbPosReset);    // Reset X & Y position values to 0
        if( fbCHK( fbRoBut4)) fbTOG( fbObject);      // Turn ON Object Detect flag
        resetRobotButtons();                         // Clear ALL robot buttons

        bbServo.irRead();    //  Save servo position to rDat1.sPos, and save
                             //  distance to rDat1.range and to servo array.
        bbServo.sweep();     //  Move servo to new position
        //bbServo.scanOnce();     //  Move servo to new position        

        bbCompass.getPosition();      //  Dead Reckoning position,
        bbCompass.getImuValues();     //  Gyro XYZ and Accelerometer XYZ.

        bbControl.control();      //  included motor control 20160710
        toggleFlagBits();         //  in 'alarm.cpp'
        // Receive joystick data two times and acknowledge each receipt
        // with a platform data packet. Then re-fill the TX FIFO buffers.
        bbRadio.platformRadioAck();
        //printRobotData();  // in 'routines.cpp'
        printAllData();      // in 'routines.cpp' - adds 20ms and causes loop to skip
        rDat2.loopTime = (uint8_t)( millis() - rDat2.botClock);        
        t3Flag = false;         // reset the timer 3 flag
    }
} // - - - -  End of PROGRAM LOOP  - - - -
