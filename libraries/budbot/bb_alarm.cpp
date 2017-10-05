/* File Name: bb_alarm.h
 * First Day: lost in the mists of time
 * Developer: Bud Ryerson
 * Last Work: 8JUL16 - added four FC-51 corner LED collision detectors
 *  14FEB17
 * Described: definition of variables and functions supporting "alarm"
 *            There are four kinds of alarms
 *            1. Over current on a motor
 *            2. Corner sensor hit
 *            3. Compound eye too close
 *            4. Loss of radio communication
 */

//  Some pins, such as Motor Current, map to analog pins,
//  such as A8, A9, etc., that do not exist on the ProMini.
//  So this file will compile only for ATmega2560
#ifdef __AVR_ATmega2560__

  #include <Arduino.h>    //  must always include this library
  #include <bb_alarm.h>
  #include <bb_defines.h>
  #include <bb_shared.h>
  #include <bb_routines.h>

  //bb_alarm::bb_alarm(){}
  //bb_alarm::~bb_alarm(){}

  //  Initiate arrays of Left Front, Right Front, Left Rear & Right Rear sensor pins
  const int odPin[ 4]  =  { odPinLF, odPinRF, odPinLR, odPinRR};     //  Obstacle Detect digital input pins
  const int odePin[ 4] =  { odePinLF, odePinRF, odePinLR, odePinRR}; //  Obstacle Detect Enable digital out pins
  const int odFlag[ 4] =  { odFlagLF, odFlagRF, odFlagLR, odFlagRR}; //  Obstacle Detect flagBits in rDat1.flagBits
  const int mcPin[ 4]  =  { mcPinLF, mcPinRF, mcPinLR, mcPinRR};     //  Motor Current analog input pins
  const int rButPin[ 4] = { rButPin1, rButPin2, rButPin3, rButPin4}; //  Robot Button digital input pins
  const int rLEDpin[ 4] = { rBluPin, rRedPin, rYlwPin, rGrnPin};     //  Robot LED digital output pins

  //  Configure input pins for various alarm sensors
  void setupAlarm()
  {
      for( int x = 0; x < 4; ++x)
      {
          pinMode( odPin[ x], INPUT_PULLUP); // set Object Detector pins
          pinMode( odePin[ x], OUTPUT);      // set Object Detector Enable pins
      }
      for( int x = 0; x < 4; ++x) pinMode( mcPin[ x], INPUT);     // Set Motor Current pins
      for( int x = 0; x < 4; ++x) pinMode( rButPin[ x], INPUT);   // Set platform Button pins
      for( int x = 0; x < 4; ++x) pinMode( rLEDpin[ x], OUTPUT);  // Set platform LED pins
  }

  // Light an LED to match rDat1 flagBit state
  void showLED()
  {
      digitalWrite( rRedPin, fbCHK( fbMotor )); //  fbMotor = 1, Motors are in run mode
      digitalWrite( rGrnPin, fbCHK( fbAuto  )); //  fbAuto = 1,  platform is Autonomous
      digitalWrite( rYlwPin, fbCHK( fbServo )); //  fbServo = 1, Servo seek is active
      digitalWrite( rOrgPin, fbCHK( fbObject)); //  fbObject = 1, Object Detect is active
     // digitalWrite( rBluPin, fbCHK( fbRadio )); //  fbRadio = 1, received valid jDat data
  }

  //  Check jDat signal bits to toggle rDat1 flag bits
  //  Check jDat modes to set rDat1 flag bits
  //  Turn on LEDs.
  void toggleFlagBits()
  {
      static uint16_t sigState = 0;       // Initialize once this function's
                                          // signal state storage variable
      if( jDat.sigBits != sigState)       // If any jDat signal bit is new,
      {
          for( int x = 0; x < 16; ++x)    // then loop through every bit
          {
              if( CHK( jDat.sigBits, x))  // and for each signal bit that's set,
              {
                  fbTOG( x);              // toggle the rDat1 flagBit.
              }
          }
          sigState = jDat.sigBits;        // Save new state of jDat signal bits.
      }

      // Manual flagBit is set if joystick is out of dead zone
      fbBUT( fbManual, ( jDat.velocity || jDat.rotation != 0));
      fbBUT( fbRadio, jDat.radio);
      fbBUT( fbSerial, jDat.serial);
      showLED();                          // Light the platform LEDs.
  }

  //  Return TRUE if any Motor Current value is too high.
  bool testMotorCurrent()
  {
      bool tooHigh = false;
      for( uint8_t x = 0; x < 4; x++)  // analog inputs 8 - 11
      {
          rDat2.motCurRay[ x] = lowByte( readADCInput( mcPin[ x]));       //  get low byte value of ADC read
          if( rDat2.motCurRay[ x] > MOTOR_CURRENT_HIGH) tooHigh = true;;  //  set 'Stall' if value too high
      }
      return tooHigh;
  }

  // Test for an Output pulse, then check for proper length.
  // Object flag set by any detector, reset by Joystick motion.
  //   odPin is normally HIGH. Goes LOW when object detected.
  //   LOW (false) = 0, HIGH (true) > 0
  bool testObjectDetect()
  {
      // Enable each Object Detector's internal 38kHz signal,
      for( int x = 0; x < 4; ++x) digitalWrite( odePin[ x], HIGH);
      // and wait for 8 pulses.  Minimum output rise time is 5 pulses.
      microDelay( 210);
      // Set each odFlag Bit according to each Object Detector's output state,
      for( int x = 0; x < 4; ++x) fbBUT( odFlag[ x], !digitalRead( odPin[ x]));
      // and wait another 15 pulses.
      microDelay( 395);
      // If an odFlag bit is HIGH, then set it again according to the current odPin state.
      for( int x = 0; x < 4; ++x) if( fbCHK( odFlag[ x])) fbBUT( ( odFlag[ x]), !digitalRead( odPin[ x]));
      // Disable each Object Detector's internal 38kHz signal.
      for( int x = 0; x < 4; ++x) digitalWrite( odePin[ x], LOW);
      
      // return TRUE if any odFlag bit is HIGH and fbObject is HIGH
      return ( ( rDat1.flagBits & 0x000F0000UL) & fbCHK( fbObject));
      //if( rDat1.flagBits & 0x000F0000UL) fbSET( fbObject); //  Set Obj Flag if any odFlag bit HIGH
      //    else fbCLR( fbObject);                           //  else Clear Obj Flag
  }

  //  = = = = = = = = = Platform Button Routines  = = = = = = = = = 
  //  Check each platform button and operate a separate timer for each.
  //  If button closure detected, wait 10ms for button to settle.
  //  -+- See same code for joystick buttons) -+-
  //  This is the second most brilliant code in the program.

  void resetRobotButtons()
  {
      for ( int x = 0; x < 4; ++x)
      {
          fbCLR( fbRoBut1 + x); // fbRoBut1 given in 'defines.h' as 20
      }
  }
  
  long pButTimer[4];        // separate timer for each platform button
  const int pButDelay = 10;  // nominal time for button contacts to settle
  //  test four platform buttons
  void checkRobotButtons()
  {
      for( int x = 0; x < 4; ++x)
      {
          // 1. if button NOT pressed, reset timer and clear a flagBit
          if( digitalRead( rButPin1 + x) == 0)
          {
              pButTimer[x] = millis() + pButDelay;
              fbCLR( fbRoBut1 + x); // fbRoBut1 given in 'defines.h' as 20
          }
          else
          {
              // 2. If button is pressed, check the timer
              if( pButTimer[x] < millis())
              {
              // 3. if timer is expired, then set the flagBit
                  fbSET( fbRoBut1 + x);
              }
          }
      }
  }
  // - - - - - - - - End of Platform Button Routines  - - - - - - - -

  
#endif  //  End of #ifdef __AVR_ATmega2560__