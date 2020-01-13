/* File Name: bb_alarm.cpp
 * Developer: Bud Ryerson
 * Inception: lost in the mists of time 
 * Last Work:  8JUL16 - added four FC-51 corner LED collision detectors
 *            07MAR19 - increased deboounce time to 20ms
              01NOV19 - modified motor current read to divide by two.  Why?
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

  bb_alarm::bb_alarm(){}   // Constructor
  bb_alarm::~bb_alarm(){}

  // Initiate arrays of Left Front, Right Front, Left Rear & Right Rear sensor pins
  // aa given in 'defines.h'
  const int odPin[ 4]  =  { odPinLF, odPinRF, odPinLR, odPinRR};     //  Obstacle Detect digital input pins
  const int odePin[ 4] =  { odePinLF, odePinRF, odePinLR, odePinRR}; //  Obstacle Detect Enable digital out pins
  const int odFlag[ 4] =  { odFlagLF, odFlagRF, odFlagLR, odFlagRR}; //  Obstacle Detect flagBits in rDat1.flagBits
  const int mcPin[ 4]  =  { mcPinLF, mcPinRF, mcPinLR, mcPinRR};     //  Motor Current analog input pins
  const int rButPin[ 4] = { rButPin1, rButPin2, rButPin3, rButPin4}; //  Robot Button digital input pins
  const int rLEDpin[ 4] = { rBluPin, rRedPin, rYlwPin, rGrnPin};     //  Robot LED digital output pins

  //  Configure input pins for various alarm sensors
  void bb_alarm::setup()
  {
      for( int x = 0; x < 4; ++x)
      {
          pinMode( odPin[ x], INPUT_PULLUP); // set Object Detector pins
          pinMode( odePin[ x], OUTPUT);      // set Object Detector Enable pins
      }
      for( int x = 0; x < 4; ++x) pinMode( mcPin[ x], INPUT);  // Set Motor Current pins
      for( int x = 0; x < 4; ++x) pinMode( rButPin[ x], INPUT);   // Set platform Button pins
      for( int x = 0; x < 4; ++x) pinMode( rLEDpin[ x], OUTPUT);  // Set platform LED pins

      sigState = 0;  // initalize signal bit state for toggleFlagBits()
  }

  // Set LEDs to match rDat1 flagBit status
  void bb_alarm::showLED()
  {
      digitalWrite( rRedPin, fbCHK( fbMotor )); //  fbMotor = 1, Motors are in run mode
      digitalWrite( rGrnPin, fbCHK( fbAuto  )); //  fbAuto = 1,  platform is Autonomous
      digitalWrite( rYlwPin, fbCHK( fbServo )); //  fbServo = 1, Servo seek is active
      digitalWrite( rOrgPin, fbCHK( fbObject)); //  fbObject = 1, Object Detect is active
     // digitalWrite( rBluPin, fbCHK( fbRadio )); //  fbRadio = 1, received valid jDat data
  }

  // Called by main Control routine in 'control.cpp'
  // Set modes according to signals from the joystick.
  //   1. Check jDat signal bits,
  //   2. Set platform modes accordingly. and
  //   3. Illuminate platform LEDs as appropriate.
  
  void bb_alarm::toggleFlagBits()
  {
      //  1. Compare jDat signal bits to saved sigState to see if
      //  any bit has changed, and toggle the corresponding flagbit.
      if( jDat.sigBits != sigState)       //
      {
          for( int x = 0; x < 16; ++x)    // then loop through 16 bits
          {
              if( CHK( jDat.sigBits, x))  // and for each signal bit that's set,
              {
                  fbTOG( x);              // toggle the rDat1 flagBit.
              }
          }
          sigState = jDat.sigBits;        // Save the status of jDat signal bits.
      }

      // 2. Set platform flag bits according to jDat data
      // ( Cannot use rDat values because Auto mode changes those.)
      //fbBUT( fbManual, ( jDat.velocity || jDat.rotation != 0));
      fbBUT( fbRadio, jDat.radioIRQ);
      fbBUT( fbSerial, jDat.serial);

      // 3. Light the corresponding platform LEDs.
      showLED();
  }

  //  Called by first line of main Control routine in 'control.cpp'
  //  Get and store the analog current value for each motor,
  //  Analog to Digital (adc) conversion is in 'routines.cpp'
  //  If no current is too high, restart the counter and return false.
  //  If any current is too high, decrement the counter.
  //  If the counter is expired, return true, else return false.
  //
  int8_t mcCount = 20;   // counts control loops, 20x = 1sec
  bool bb_alarm::testMotorCurrent()
  {
      static bool tooHigh;
      tooHigh = false;
      // Recast the 'uint32_t' pointer as a 'uint8_t' pointer
      // and assign it to point to a 'uint8_t' variable.
      // Then address the pointer like an array.
      //uint8_t * mcRay = ( uint8_t *)rDat2.motCurRay;
      uint16_t adcInput;
      for( uint8_t i = 0; i < 4; i++)  // analog inputs 8 - 11
      {
          //  ADC read returns a 'uint16_t' type variable.
          adcInput = readADCInput( mcPin[ i]);
          if( adcInput < MOTOR_CURRENT_HIGH)        // If current not too high...
          {                                         // (set to 150 in 'defines.h')
              rDat2.mcRay[ i] = ( uint8_t)adcInput; // then save low-order byte mcRay.
           }
          else
          {
              rDat2.mcRay[ i] = MOTOR_CURRENT_HIGH;  // else save high value
              tooHigh = true;                        // set tooHigh value 'true'
          }
      }
      if( tooHigh)          // If ANY current value too high
      { 
        if( mcCount > 0)    // If counter NOT expired
        {
          --mcCount;        // then dec counter
        }
        else return true;   // else return 'true'
      }
      else mcCount = 20;    // else reset counter.
      return false;
  }

  // - - - -  Object Detector  - - - -
  // The Object flag (fbObject) is toggled on/off by Robot Button #4
  // and it is reset by any Joystick motion (fbManual).
  // The particular Object Detect flag (odFlagXX) is set by the corner
  // at which detection occurs.
  // The whole process takes less than a millisecond.
  // Test for an Output pulse and check for proper length.
  //   odPin is normally HIGH. Goes LOW when object detected.
  //   LOW (false) = 0, HIGH (true) > 0
  bool bb_alarm::testObjectDetect()
  {
      // Enable each Object Detector's internal 38kHz signal,
      for( int x = 0; x < 4; ++x) digitalWrite( odePin[ x], HIGH);
      // and delay for 8 pulses.  An Object detector's
      // output signal rise time is a minimum of 5 pulses.
      microDelay( 210);
      // Set each odFlag Bit according to its Object Detector's output state,
      for( int x = 0; x < 4; ++x) fbBUT( odFlag[ x], !digitalRead( odPin[ x]));
      // and wait for another 15 pulses.
      microDelay( 395);
      // Then if the Object Detector signal is still HIGH (or HIGH again)
      // and the odFlag bit had been previously set, then keep it set;
      // otherwise reset it.
      for( int x = 0; x < 4; ++x) if( fbCHK( odFlag[ x])) fbBUT( ( odFlag[ x]), !digitalRead( odPin[ x]));
      // Disable each Object Detector's internal 38kHz signal.
      for( int x = 0; x < 4; ++x) digitalWrite( odePin[ x], LOW);

      // return TRUE if any odFlag bit is HIGH and fbObject is HIGH
      return ( ( rDat1.flagBits & 0x000F0000UL) & fbCHK( fbObject));
      //return ( ( rDat1.senseBits & 0x0F) & fbCHK( fbObject));
      //if( rDat1.flagBits & 0x000F0000UL) fbSET( fbObject); //  Set Obj Flag if any odFlag bit HIGH
      //    else fbCLR( fbObject);                           //  else Clear Obj Flag
  }

  //  = = = = = = = = = Platform Button Routines  = = = = = = = = =
  //  Check platform button and set a separate timer for each one.
  //  If button closure detected, wait 10ms for button to settle.
  //  If still pressed, set 1 sec lock-out time to prevent repeats.
  //  ***  See same code for joystick buttons  ***
  //  This is the second most brilliant code in the program.
  static uint32_t pButTimer[4];         // separate timers for each button
  static const uint8_t pButDelay = 20;  // time for contacts to settle
  
  bool bb_alarm::testPlatformButton( uint8_t pButNmbr)
  {
      if( digitalRead( rButPin[ pButNmbr]) != 0) // If button pressed...
      {
          if( pButTimer[ pButNmbr] < millis())   // AND timer expired...
          {
              pButTimer[ pButNmbr] = millis() + 1000;  // set 1 sec lock-out timer...
              return true;                       // and return TRUE.
          }
      }
      else                                        // If button NOT pressed...
      {
          pButTimer[ pButNmbr] = millis() + pButDelay;  // ...reset timer.
      }
      // If button NOT pressed OR timer NOT expired then return false
      return false;
  }

  //  test four platform buttons
  void bb_alarm::checkPlatformButtons()
  {
      if( testPlatformButton( 0)) fbTOG( fbServo);       // Toggle 'Sweep' routine in 'servo.cpp'
      if( testPlatformButton( 1)) fbSET( fbProgram);     // Do 'Program' routine in 'control.cpp'
      if( testPlatformButton( 2)) fbSET( fbPosReset);    // Part of 'getPosition' in 'compass.cpp'
      if( testPlatformButton( 3)) fbTOG( fbObject);      // Enable flag for 'testObjectDetect' above
  }
  // - - - - - - - - End of Platform Button Routines  - - - - - - - -

  void bb_alarm::printStatus()
  {
      printf( "Status = ");
      if( fbCHK( fbTrack)) printf( "track");
      else if( fbCHK( fbPivot)) printf( "pivot");
      else if( fbCHK( fbEvade)) printf( "evade");
      else if( fbCHK( fbProgram)) printf( "program");
      else if( fbCHK( fbFault)) printf( "fault");
      else if( fbCHK( fbManual)) printf( "manual");
      else if( fbCHK( fbAuto)) printf( "auto");
      printf( "\r\n");
  }
  
#endif  //  End of #ifdef __AVR_ATmega2560__