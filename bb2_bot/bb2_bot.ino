 /*  File Name: bb2_bot.ino
  *  Described: This is for the robot platfrom.
  *             THIS IS THE MASTER/RF TRANSMITTER
  *  Developer: Bud Ryerson
  *  Inception: 23 NOV 2014
  *  Last Work: 07 MAR 19 - made 't3flag' volatile
  *             30 MAR 19 - made Lidar routine library and rearranged range
  *             read to match servo position as transmitted to joystick.
  The robot platform loop contains a 'control' sub-loop
  triggered by a 50ms time to effectively run at 20Hz.

  */

#include <Arduino.h>       //  This is used everywhere
#include <EEPROM.h>
#include <Wire.h>          //  BBO055 NDOF IMU I2C Communication
#include <SPI.h>           //  nRF24 Radio SPI Communication
#include <printf.h>        //  printf is used everywhere

#include <nRF24L01.h>      //  nRF24 Radio Library
#include <RF24.h>
#include <RF24_config.h>

#include <bb_shared.h>     //  global variables
#include <bb_routines.h>   //  and routines
#include <bb_notes.h>      //  musical notes and songs

#include <bb_lidar.h>      //  Calls TFMPlus libraries
bb_lidar bbLidar;          //  for lidar sensing device.

#include <bb_encoder.h>
bb_encoder bbEncoder;      // instantiate the two Encoder ISRs and routines

#include <bb_radio.h>
bb_radio bbRadio;

#include <bb_compass.h>
bb_compass bbCompass;

#include <bb_control.h>  // control calls the servo library, which calls
bb_control bbControl;    // the lidar and megaservo libraries

/* = = = = =  Robot button operations are in 'alarm.cpp'  = = = = = =
 ( Button 1 is nearest to the outer edge of the platfrom )
 Robot Button 1 - fbTOG( fbServo);     Toggle 'Sweep' routine in 'servo.cpp'
 Robot Button 2 - fbSET( fbProgram)    Do 'Program' routine in 'control.cpp'
 Robot Button 3 - fbSET( fbPosReset)   Reset IMU and zero position values as
                                       part of 'getPosition' in 'compass.cpp'
 Robot Button 4 - fbTOG( fbObject)     Enable 'testObjectDetect' in 'alarm.cpp'
                                       called by 'control' in 'control.cpp'
*/

/* -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- */
//  Output Compare Register 3A Interrupt Service Routine
//  OCR3A is set to 3124 (50ms) in 'routines.cpp'
//  !! currently running at 0.032msec - 22APR19 !!
volatile bool t3aFlag = false;       //  Timer 3A flag
ISR( TIMER3_COMPA_vect)    //  Timer 3A ISR triggers every 50ms
{
    t3aFlag = true;         //  set Timer3A Flag
}

//  Output Compare Register 3B Interrupt Service Routine
//  OCR3B is set to 1562 (25ms) in 'routines.cpp'
volatile bool t3bFlag = false;       //  Timer 3A flag
ISR( TIMER3_COMPB_vect)    //  Timer 3A ISR triggers every 50ms
{
    t3bFlag = true;         //  set Timer3B Flag
}
/* -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- */

void setup()
{
    setupADCregisters();   //  Custom Analog to Digital is in 'routines.cpp'
    setupTimer3();         //  Setup Timer 3 is also in 'routines.cpp'
    pinMode( Speaker, OUTPUT);    //  set Speaker pin (D7) to OUTPUT
    playNote( 0);           // play middle C

    Serial.begin( 115200);  //  this must be set to 9600 for MagViewer
    Serial.flush();         //  flush the serial buffer
    printf_begin();         //  initiate the 'printf' module
    printf("Welcome to BudBot2 - 16MAR19\r\n");

    SPI.begin();            //  initiate the SPI object for the Radio
    Wire.begin();           //  initiate the I2C object for the IMU
    delay(5);               //  wait at least 5ms before continuing

    // setup output pins and initialize data structures
    bbEncoder.encoderSetup();   //  setup the Encoders in "bb_encoder.h" file
    bbRadio.setupPlatformRadio();   //  initiate platform radio TX mode
    bbCompass.setupIMU();
    bbCompass.testIMU();            // count down to IMU streaming
    bbCompass.getImuCalStatus();
    bbCompass.printImuCalStatus();  // display IMU status
    playNote( 1);                   // play D above C    

    bbLidar.setup();                // includes servo setup and test
    delay(20);

    bbControl.setup();    //  call Alarm setup
                          //  •  set Object Detect Sense and Enable pins
                          //  •  set Motor Current pins
                          //  •  set Platform Button and LED pins
                          //  call Servo setup and test
                          //  •  set azimuth servo output pin 6
                          //  •  test servo clearance
                          //  call Motor setup
                          //  clear joystick and robot data structures
                          //  setup Velocity and Rotation PID controllers
    playNote( 2);         // play E above middle C0x5A
}

// = = = = = = = = MAIN PROGRAM LOOP = = = = = = = = =
// Get joystick values, run controls and return data
void loop()
{
    //  Every 50.0ms, the Timer3 Interrupt Service Routine sets tc3Flag TRUE.
    //  The last statement of this loop resets the T3 flag to FALSE.
    if( t3aFlag)
    {
        // elapsed time since last T3 interrupt - should always equal 50 milliseconds
        // rDat2.loopTime = (uint8_t)( millis() - rDat2.botClock);
        //  Update bot clock and loTime
        rDat2.botClock = millis();    // Save platform system time in rDat2
        rDat1.loTime = rDat2.loTime;  // Save low byte of system time in rDat1,
                                      // used to check receipt of both rDat's.

        //  -  -  -  -  -  -  -   SENSING  -  -  -  -  -  -  -
        // Get the number of encoder interrupts for the Left Front and Right Front
        // motors since the last T3 and save in rDat1 as "intCount". Calculate the
        // average time per interrupt to get motor speed. For precise timing, this
        // routine should occur as soon as possible after a T3 interrupt.
        bbEncoder.pulseAverage();
        // Update the IMU position and motion values
        bbCompass.getPosition();      // Dead Reckoning position,
        bbCompass.getImuValues();     // Gyro and Accelerometer XYZ values.
        // Get Lidar range and flux data and save to rDat1 and save to rDat1
        //bbLidar.read();

        //  -  -  -  -  -  -   COMMUNICATION  -  -  -  -  -  -  -
        // Transmit the current platform status data (rDat) as two 32 byte data packets
        // and receive one 32 byte acknowledgment (ACK) packet as joystick data (jDat)
        // after each.  ACK from the second packet will overwrite the first jDat.
        bbRadio.platformRadioAck();

        //  -  -  -  -  -   NAVIGATION & CONTROL  -  -  -  -  -  -  -
        // Check platfom buttons,
        // If servo ON, move servo to next position
        // Run motion program, control motors accordingly
        // Set and toggle flagbits according to jDat and platform buttons.
        bbControl.control();

        //  -  -  -  -  -  -  -  TESTING  -  -  -  -  -  -  -
        printRobotData();    // in 'routines.cpp'
        //  printAllData();  // as three long segments of hex characters
                             // adds 20ms and causes the loop to skip
                             // also in 'routines.cpp'

        //  -  -  -  -  -  -   HOUSEKEEPING  -  -  -  -  -  -  -
        t3aFlag = false;         // Reset the Timer 3 flag.
        ++rDat2.botLoop;        // increment robot loop counter
        digitalWrite( rBluPin, !digitalRead(rBluPin));    //  Toggle blue LED
        // measure the time to complete all instructions in loop
        uint32_t currentTime = millis();
        rDat2.loopTime = (uint8_t)( currentTime - rDat2.botClock);
    }

    //  Every 25.0ms, the OC Timer3B Interrupt Service Routine sets t3bFlag TRUE.
    //  The last statement of this loop resets the T3B flag to FALSE.
    if( t3bFlag)
    {
        // Get Lidar range and flux data and save to rDat1 and save to rDat1
        bbLidar.read();
        t3bFlag = false;         // Reset the Timer 3B flag.
    }

} // - - - -  End of PROGRAM LOOP  - - - -
