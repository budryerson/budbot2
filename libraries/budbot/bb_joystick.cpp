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

   // Joystick Button attachemnt pin numbers are defined in 'defines.h'
  uint8_t butPinNumber[3] = { jBut1Pin, jBut2Pin, jBut3Pin};
  void bb_joystick::setup()
  {
      pinMode( joyPin1, INPUT);         //  set ProMini pin# A0 as joystick X-axis
      pinMode( joyPin2, INPUT);         //  set ProMIni pin# A1 as joystick Y-axis

      pinMode( jBut1Pin, INPUT_PULLUP);  // set ProMini pin# 3 as Motor mode button
      pinMode( jBut2Pin, INPUT_PULLUP);  // set ProMini pin# 7 as Auto mode button
      pinMode( jBut3Pin, INPUT_PULLUP);  // set ProMini pin# 8 as Servo mode button

      pinMode( jGrnPin, OUTPUT);     // set ProMini pin# 4 - Green LED - Auto mode
      pinMode( jYlwPin, OUTPUT);     // set ProMini pin# 5 - Yellow LED - Servo mode
      pinMode( jRedPin, OUTPUT);     // set ProMini pin# 6 - Red LED - Motor mode
      pinMode( jBluPin, OUTPUT);     // set ProMini pin# 18 - Blue LED - Com mode
  }

  // Light an LED to match rDat1 flagBit state
  void bb_joystick::showLights()
  {
      digitalWrite( jRedPin, fbCHK( fbMotor));  //  fbMotor = 1, Motors are in run mode
      digitalWrite( jGrnPin, fbCHK( fbAuto));   //  fbAuto = 1, robot platform is Autonomous
      digitalWrite( jYlwPin, fbCHK( fbServo));  //  fbServo = 1, Servo seek is active
      //digitalWrite( jBluPin, !digitalRead(jBluPin));   //  Toggle blue LED
  }

  //  = = = =  Read Joystick Data routine and check buttons  = = = =
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
      //  'jZmin' and 'jZmax' refer to "(Dead) Zone" values given in 'defines.h'.
      // joystick Y motion translate to primary transport velocity
      if ( jDat.jvY >= jZmin && jDat.jvY <= jZmax) jDat.velocity = 0;
          else jDat.velocity = map( jDat.jvY, 1, 1018, -255, 255);
      // joystick X motion translates to rotational velocity modifier
      if ( jDat.jvX >= jZmin && jDat.jvX <= jZmax) jDat.rotation = 0;
          else jDat.rotation = map( jDat.jvX, 0, 1022, -255, 255);

      //  If velocity is non-zero, then limit 'rotation' value to not more
      //  than the difference between 'velocity' and maximum velocity (255).
      if( jDat.velocity != 0)
      {
          static int spdDiff;
          spdDiff = 255 - abs( jDat.velocity);  // find limit of rotation on velocity
          jDat.rotation = intConstrain( jDat.rotation, ( spdDiff * -1), spdDiff);
      }
      // Set MANUAL flag if joystick is moved
      if( jDat.velocity || jDat.rotation != 0) fbTOG( fbManual);
      //if( jDat.sync == true) fbTOG( fbRadio);

      //  - - - - - - - - -  Check three joystick buttons  - - - - - - -
      //  Check three joystick button and start a separate timer for each.
      //  If pressed and held for 10 msec, then SET the corresponding signal bit.
      //  Button is then locked-out for 1 second to prevent repeated toggles.
      //
      //  The Platform uses signal bits to toggle corresponding mode states.
      //  Signal bits are cleared in 'routines.cpp' during initialization
      //  and again before 'getPaneData' after both rDat packets are synced
      //
      //  *** This is the second most brilliant code in the program. ***
      static uint32_t jButTimer[ 3];     // separate timer for each button

      for ( int x = 0; x < 3; ++x)  // check each of the three joystick buttons
      {
          if( !digitalRead( butPinNumber[ x]))      // If a button pressed (LOW)...
          {
              if( jButTimer[ x] < millis())         // and its timer has expired...
              {
                  SET( jDat.sigBits, x);            // then set the signal bit and
                  jButTimer[ x] = millis() + 1000;  // and set 1 sec lock-out timer.
              }
          }
          else                                      // If button NOT pressed...
          {
              jButTimer[ x] = millis() + 10;        // set 10 msec de-bounce timer.
          }
      }
  }
  //  - - - - - -    End of Read Joystick Data Routine    - - - - - - -

  //  = = = = = = = =    putPaneData (write) routine    = = = = = = = =
  //  This formats and serial writes the date received from
  //  the robot platform for the Processing display routine
  void bb_joystick::putPaneData()
  {
//      while(Serial.available()) // flush the serial RX buffer
//      {
//          Serial.read();
//      }
      Serial.flush();           // flush the serial TX buffer

      // - - - -  jDat values  - - - -
      printf( "|JX%4i|JY%4i|PV% 04i|PR% 04i",
          jDat.jvX,
          jDat.jvY,        //  joystick values: 0 - 1023
          jDat.velocity,
          jDat.rotation);  //  motion values: -255 to +255

      // - - - -  rDat values  - - - -
      printf( "|PH%03u|PC%03u|PB% 04i",
          rDat1.head,      //  magnetic direction of platform: 000° to 359°
          rDat1.course,    //  set course when fbAuto initiated: 000° to 359°
          rDat1.bear );    //  difference between head and course: -180° to 180°

      printf( "|ML% 04i|MR% 04i|EL%03u|ER%03u",
          rDat1.motorSpeedLeft,   //  Signed integer of Left and Right
          rDat1.motorSpeedRight,  //  motor speed: -255 to +255
          rDat1.intCountL,        //  Unsigned byte of Left and Right wheel
          rDat1.intCountR );      //  encoder Interrupt Counters per T3 loop

      printf( "|X1%03i|LD%03u|LF%05u|X2%03i",
          rDat1.azPos,      // First Azimuth index value
          rDat1.dist,       // Lidar distance value in centimeters
          rDat1.flux,       // Lidar signal return intensity
          rDat1.azPos2 );   // Second Azimuth index value

      // Flag Bits as an 8 char hexadecimal number
      // (converted to boolean string in Processing).
      printf( "|FB%08lX", rDat1.flagBits);

      // - - - -  rDat2 values  - - - -

      // Motor Current array as 8 char hexadecimal integer
      printf( "|MC%08lX", rDat2.mcInt);

      // Dead reckoning position
      printf( "|PX% 7i|PY% 7i",
          rDat2.drPosX,      // dead reckoning position X
          rDat2.drPosY);     // dead reckoning position Y

      //  IMU values
      printf( "|AX% 7i|AY% 7i|SX% 7i|SY% 7i",
          rDat2.imAccX,      // IMU X-axis linear acceleration in mm/sec/sec
          rDat2.imAccY,      // IMU Y-axis linear acceleration
          rDat2.imSpdX,      // IMU X-axis speed in mm/sec
          rDat2.imSpdY);     // IMU Y-axis speed in mm/sec

      printf( "|PT%8lX|LC%8lX|LT% 4u",
          rDat2.botClock,    //  Platform Clock in milliseconds - 8 place hex number
          rDat2.botLoop,     //  Platform Loop Counter - 8 place hex number
          rDat2.loopTime);   //  Platform Loop Time -  4 place unsigned integer

  //    printf( "|ID% 4u",  rDat2.mpxID);      // joystick signal string length
  //    printf( "|LC%10lu", rDat2.botLoop);   // platform Loop Counter
  //    printf( "|LT %4u", rDat2.loopTime);    // platform Loop execution time

      // Serial.write( ( uint8_t*) &rDat1, sizeof( rDat1));   // waiting for the day
      printf("\r\n");                        //  end of line
      // Total of 214 (was 225, 253) characters in a line,
      // (not counting the CR/LF characters at the end).
  //  getPaneData();
  }
  //  - - - -  End of getPaneData (read signal string) routine  - - - -

  //  = = = = = = = =    getPaneData (read) routine    = = = = = = = =
  //  Try 15 times to read data from PC and wait 250us
  //  after each try.  Same retry setting as Radio
  //  The only bits that might get set in Processing are:
  //  fbMotor | fbAuto | fbServo | fbProgram | fbObject | fbPosReset
  static String strBuffer;  // buffer for PC signal string
  //static String gpdBuffer = "";  // holder for the input string to be chopped up
  static byte gpdCount;
  static const byte gpdCountMax = 15; // max read retries
  void bb_joystick::getPaneData()
  {
      gpdCount = 0;      // zero data read retry counter
      strBuffer = "";    // clear buffer for signal characters from PC
      fbCLR( fbSerial);      // CLR rDat serial
      jDat.serial = false;   // CLR jDat serial

      while( gpdCount < gpdCountMax)
      {
          if( Serial.available() > 0)
          {
              // Examine the serial port looking for an EOL character
              strBuffer = Serial.readStringUntil('\n');
              // If an EOL character has been received...
              if ( strBuffer != "")
              {
                  // test for valid first character and proper length.
                  if( strBuffer.startsWith( "|") && strBuffer.length() == 17)
                  {
                      jDat.strLen = (uint8_t)strBuffer.length();  // length of signal string
                      for( int x = 0; x < 16; ++x)
                      {
                        // 'sigBits' reset to 0 on each loop by 'readData()' above
                        if( strBuffer[ x + 1] == '1') SET( jDat.sigBits, x);
                      }
                      fbSET( fbSerial);
                      jDat.serial = true;     // CLR jDat serial
                      gpdCount = gpdCountMax; // and kill the loop
                  }
              }
          }
          ++gpdCount;        // advance the count
          microDelay( 250);  // and wait 250 microseconds to try again
      }
      //Serial.flush();        // regardless of result, flush the serial buffer
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
      printf( "Radio IRQ Count= %5u ", jDat.rIrqCount);
  //    printf(" | Robot Data Size= %1u ", rDat1.rDat1Size);
      printf( "\r\n");
  }

#endif  //  - - - - - -  End of Do NOT compile for Mega2560  - - - - - - -