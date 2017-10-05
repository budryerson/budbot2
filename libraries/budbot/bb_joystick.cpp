/*  File Name: bb_joystick.cpp
 *  Described: Budbot2 joystick routines
 *  Developer: Bud Ryerson
 *  Last Work:
 *  25DEC15 - Stopped the joystick pin assignments and read data routine from
 *            compiling for the Mega2560 because of pin assignment conflicts.
 *  06MAY16 - Moved all velocity and rotation computation to the joystick
 *            and send them to robot. Fixed incorrect usage of "intDiv".
 *  08JUN16 - Moved joystick motion detection to the joystick and added
 *            'isManual' flag to toggleFlagBits() and control() routines.
 *            Deleted boolean isManualed() routine.
 *  20JUL16 - Got rid of setFlags routine.  Almost finished conversion from
 *            boolean flags to pure flasgBits.
 *  16AUG16 - made 'joystick' library exclusive to 'joy.ino'
 *  31DEC16 - changed sense of joystick buttons from SET/CLR to TOG flag bits
 *  00JAN16 - changed back to SET/CLR
 *  28JAN16 -  rewrote getPaneData.  Some flickering at first when multiple modes set.
 *             Now everything seems to be working perfectly.  Need to freeze this.
 *             Also need to add a blue COM light to joystick
 */

//  --  --  --  Do NOT compile for Mega2560  --  --  --  --
//  --  --   or pin numbers will get confused    --  --  --
#ifndef __AVR_ATmega2560__

  #include <Arduino.h>
  #include <bb_defines.h>
  #include <bb_shared.h>
  #include <bb_routines.h>
  #include <bb_joystick.h>

  bb_joystick::bb_joystick(){}  //  "::" is the 'scope resolution operator'
  bb_joystick::~bb_joystick(){}

  uint8_t butPinNumber[3] = { jButPin1, jButPin2, jButPin3};
  void bb_joystick::setup()
  {
      pinMode( joyPin1, INPUT);         //  set ProMini pin# A0 as joystick X-axis
      pinMode( joyPin2, INPUT);         //  set ProMIni pin# A1 as joystick Y-axis

      pinMode( jButPin1, INPUT_PULLUP);  // set ProMini pin# 3 as Motor mode button
      pinMode( jButPin2, INPUT_PULLUP);  // set ProMini pin# 7 as Auto mode button
      pinMode( jButPin3, INPUT_PULLUP);  // set ProMini pin# 8 as Servo mode button

      pinMode( jGrnPin, OUTPUT);     // set ProMini pin# 4 as Green LED pin and Auto mode indicator
      pinMode( jYlwPin, OUTPUT);     // set ProMini pin# 5 as Yellow LED pin and Servo mode indicator
      pinMode( jRedPin, OUTPUT);     // set ProMini pin# 6 as Red LED pin and Motor mode indicator
      pinMode( jBluPin, OUTPUT);     // set ProMini pin# 18 as Pane Communication indicator
  }

  // Light an LED to match rDat1 flagBit state
  void bb_joystick::showLights()
  {
      digitalWrite( jRedPin, fbCHK( fbMotor));  //  fbMotor = 1, Motors are in run mode
      digitalWrite( jGrnPin, fbCHK( fbAuto));   //  fbAuto = 1, robot platform is Autonomous
      digitalWrite( jYlwPin, fbCHK( fbServo));  //  fbServo = 1, Servo seek is active
      digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
  }

  //  = = = = = = = =   read Joystick data routine   = = = = = = =

  //  - - - - - - - - -  Check three joystick buttons  - - - - - - -
  //  Check three joystick button and seat a separate timer for each.
  //  If Button is pressed and held for joyButDelay amount of time
  //  then set the corresponding signal bit TRUE.
  //  Platform uses a signal bit to toggle the corresponding mode state.
  //  *** This is the second most brilliant code in the program. ***
  void bb_joystick::checkJoyButtons()
  {
      static uint32_t jButTimer[ 3];     // separate timer for each button
      static uint32_t jButDelay = 10;    // 10 millisecond settle time for contacts
      jDat.sigBits = 0;                  // reset 'jDat.sigBits' to zero
      for ( int x = 0; x < 3; ++x)
      {
          if( ( !digitalRead( butPinNumber[ x]))    // If button pressed (LOW)
            &&( jButTimer[ x] < millis()))          // and its timer has expired,
          {
              SET( jDat.sigBits, x);                // then set the signal flag bit
          }
          else                                      // otherwise
          {
              CLR( jDat.sigBits, x);                // clear the signal flag bit
              jButTimer[ x] = jButDelay + millis(); // and reset its timer
          }
      }
  }
  //  - - - - - - - -  End of Check three joystick buttons  - - - - - - -

  //  - - - - - - - - -  Read analog joystick data  - - - - - - -
  void bb_joystick::readData()
  {
      //  Get joystick analog values
      jDat.jvX = analogRead( joyPin1);  //  discard first read
      jDat.jvX = analogRead( joyPin1);  //  second read is for real
      delay( 1);                        //  settle A/D converter
      jDat.jvY = analogRead( joyPin2);
      jDat.jvY = analogRead( joyPin2);
      delay( 1);                        //  settle A/D converter

      //  - - -  convert joystick data to robot platform velocity and rotation values  - - - -
      //  'jZmin' and 'jZmax' refer to "(Dead) Zone" values as defined in 'defines.h'.
      if ( jDat.jvY >= jZmin && jDat.jvY <= jZmax) jDat.velocity = 0;   // joystick Y motion gets primary
          else jDat.velocity = map( jDat.jvY, 1, 1018, -255, 255);      // transport velocity value
      if ( jDat.jvX >= jZmin && jDat.jvX <= jZmax) jDat.rotation = 0;   // joystick X motion gets secondary
          else jDat.rotation = map( jDat.jvX, 0, 1022, -255, 255);      // transport rotational modifier

      //  If velocity is non-zero, then limit 'rotation' value to no more
      //  than the difference between 'velocity' and maximum velocity (255).
      if( rDat1.velocity != 0)
      {
          static int spdDiff;
          spdDiff = 255 - abs( jDat.velocity);  // find limit of rotation on velocity
          intConstrain( jDat.rotation, ( spdDiff * -1), spdDiff);
      }

      //checkJoyButtons();
      //  Check for Joystick motion and set MANUAL flag bit
      BUT( jDat.sigBits, ( jDat.velocity || jDat.rotation != 0), fbManual);
  }
  //  - - - - - -    End of Read Joystick Data Routine    - - - - - - -

  //  = = = = = = = =    putPaneData (write) routine    = = = = = = = =
  //  This formats and serial writes the date received from
  //  the robot platform for the Processing display routine
  void bb_joystick::putPaneData()
  {
      //   Serial.flush();        // flush the serial buffer

      // jDat values
      printf( "|JX%4i|JY%4i", jDat.jvX, jDat.jvY);      //  joystick values: 0 - 1023
      printf( "|PV% 04i|PR% 04i", jDat.velocity, jDat.rotation);  //  motion values: -255 to 255

      // rDat values
      printf( "|PH%03u", rDat1.head);      //  magnetic direction of platform: 0 to 359°
      printf( "|PC%03u", rDat1.course);    //  set course when fbAuto initiated: 0 to 359°
      printf( "|PB% 04i", rDat1.bear);     //  difference between head and course: -180 to 180°

      printf( "|ML% 04i|MR% 04i", rDat1.motorSpeedLeft, rDat1.motorSpeedRight);  // L and R motor speed: -255 to 255
      printf( "|EL%03u|ER%03u", rDat1.intCountL, rDat1.intCountR);  //  L&R wheel encoder Interrupt Counters.

      printf( "|SP% 04i", rDat1.srvPos);   // Pan Servo Positions: 0 - 180
      printf( "|IV%03u",  rDat1.avgVal);   // A single raw IR sensor output value
      printf( "|ID%03u",  rDat1.range);    // Average IR value converted to centimeters
      printf( "|NP% 04i", rDat1.nearPos);  // Average IR value converted to centimeters

      printf( "|FB");                      // Flag Bits as a 32bit 'long'
      for( int x = 0; x < 32; ++x)
      {
          if( ( rDat1.flagBits >> x) & 1) printf( "1"); else printf( "0");
      }

      // rDat2 values
      printf( "|LF%03u", rDat2.motCurRay[ 0]);  // Motor Current Left Front
      printf( "|RF%03u", rDat2.motCurRay[ 1]);  // Motor Current Right Front
      printf( "|LR%03u", rDat2.motCurRay[ 2]);  // Motor Current Left Rear
      printf( "|RR%03u", rDat2.motCurRay[ 3]);  // Motor Current Right Rear

      printf( "|PX% 7i", rDat2.drPosX);      // dead reckoning position X
      printf( "|PY% 7i", rDat2.drPosY);      // dead reckoning position Y
      //  IMU values
      printf( "|AX% 7i", rDat2.imAccX);      // IMU X-axis linear acceleration in mm/sec/sec
      printf( "|AY% 7i", rDat2.imAccY);      // IMU Y-axis linear acceleration
      printf( "|SX% 7i", rDat2.imSpdX);      // IMU X-axis speed in mm/sec
      printf( "|SY% 7i", rDat2.imSpdY);      // IMU Y-axis speed in mm/sec

      printf( "|PT%10lu", rDat2.botClock);   //   Platform Clock in milliseconds
      printf( "|LC%10lu", rDat2.botLoop);    // joystick Loop Counter
      printf( "|LT% 4u",  rDat2.loopTime);      // joystick signal string length
  //    printf( "|ID% 4u",  rDat2.mpxID);      // joystick signal string length
  //    printf( "|LC%10lu", rDat2.botLoop);   // platform Loop Counter
  //    printf( "|LT %4u", rDat2.loopTime);    // platform Loop execution time

      printf("\r\n");                        //  end of line
  //    getPaneData();
  }
  //  - - - -  End of getPaneData (read signal string) routine  - - - -

  //  = = = = = = = =    getPaneData (read) routine    = = = = = = = =
  //  Try 15 times to read data from PC and wait 250us
  //  after each try.  Same retry setting as Radio
  String gpdBuffer = "";  // holder for the input string to be chopped up
  const int gpdCountMax = 15; // max read retries
  void bb_joystick::getPaneData()
  {
      int gpdCount = 0;      // zero counter for data read retries
      fbCLR( fbSerial);      // CLR rDat serial
      jDat.serial = false;   // CLR jDat serial
      String strBuffer = "";  // buffer for PC signal string
      while( gpdCount < gpdCountMax)
      {
          if( Serial.available() > 0)
          {
              // Examine the serial port looking for an EOL character
              strBuffer = Serial.readStringUntil('\n');
              // Do not continue unless an EOL character has been received
              if ( strBuffer != "")
              {
                  // test first characters and length
                  if( strBuffer.startsWith( "|") && strBuffer.length() == 17)
                  {
                      jDat.strLen = (uint8_t)strBuffer.length();  // length of signal string
                      for( int x = 0; x < 16; ++x)
                      {
                        // 'jDat.sigBits' set to zero each loop by 'checkJoyButtons()'
                        if( strBuffer[ x + 1] == '1') SET( jDat.sigBits, x);
                      }
                      fbSET( fbSerial);
                      jDat.serial = true;   // CLR jDat serial
                      gpdCount = gpdCountMax; // and kill the loop
                  }
              }
          }
          ++gpdCount;        // advance the count
          microDelay( 250);  // and wait 250 microseconds to try again
      }
      Serial.flush();        // regardless of result, flush the serial buffer
  }
  //  - - - -  End of getPaneData (read) routine    - - - -

  // This prints joystick data
  // It can be called by 'bb2_joy.ino' if needed for diagnostic reasons
  void bb_joystick::printJoyData()
  {
      printf( "Joystick data:  ");
      printf( "X= %4i | Y= %4i | ", jDat.jvX, jDat.jvY);
      printf( "V= %4i | R= %4i | ", jDat.velocity, jDat.rotation);
      printf( "jFlagBits= ", jDat.sigBits);
      for( int x = 0; x < 16; ++x)
      {
          if( ( jDat.sigBits >> x) & 1) printf( "1"); else printf( "0");
      }
      printf( " | ");
      printf( "Motor= %1u ", fbCHK( fbMotor));
      printf( "Auto= %1u ",  fbCHK( fbAuto));
      printf( "Servo= %1u ", fbCHK( fbServo));
      printf( "Joy Loop Count= %5u ", jDat.joyLoop);
  //    printf(" | Robot Data Size= %1u ", rDat1.rDat1Size);
      printf( "\r\n");
  }

#endif  //  - - - - - -  End of Do NOT compile for Mega2560  - - - - - - -