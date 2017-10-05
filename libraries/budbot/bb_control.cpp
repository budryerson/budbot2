/* File Name: bb_control.cpp
 * Developer: Bud Ryerson
 * First Day: 31DEC15
 * Described:  Robot control code: retreat, autonomous,
 *             joystick and program.
 * Last Work: 27MAR16 - working on the program routine
 *            having some trouble tracking
 *            see line #77
 * 10JUL16 - Moved the Motor Library call as well as calls to
 *           Motor Setup and Motor Control from the main program
 *           to here
 * 27JUL16 - I have got to figure this control thing out somehow.
 */

//  --  --  --  --  Compile only for Mega 2560  --  --  --  --
//  Joystick library is called only by "joytx_4.ino"
#ifdef __AVR_ATmega2560__

  #include <Arduino.h>
  #include <bb_defines.h>
  #include <bb_shared.h>
  #include <bb_routines.h>
  #include <bb_notes.h>       //  musical notes and songs

  #include <bb_alarm.h>
  //bb_alarm bbAlarm;
  #include <bb_motor.h>
  bb_motor bbMotor;
  #include <bb_pid.h>
  bb_pid bbPidRC;    // instantiate PID controller for Rotation Control
  bb_pid bbPidVC;    // instantiate PID controller for Velocity Control

  #include <bb_control.h>
  bb_control::bb_control(){}
  bb_control::~bb_control(){}

  void bb_control::setup()
  {
      //bbAlarm.setup();          // setup sensor and button input pins
      setupAlarm();
      bbMotor.setup();          //  set motor output pins, encoder, PID controls
                                //  and clear motor data structures
      //  bbMotor.motorReset();    //  velocity = 0; difference = 0; motorRun = FALSE

      clearDataStructures();    //  in 'routines.cpp'

      fbCLR( fbAuto);  //  clear Auto Mode

      //  Set parameters for the gross motion PID Controllers
      //                Kp    Ki    Kd  min/max In   min/max Out
      bbPidRC.setupPID( 2.0, 0.003, 1.0, -180,  180, -255, 255);   //  Rotation Control
      bbPidRC.setPoint =  0.0;  // initial setPoint

      //bbPidVC.setupPID( 5.0,  0.5, 1.0,   17,  250, -255, 255);  //  Velocity Control - original
      bbPidVC.setupPID( 10.0,  1.0,  5.0,  20,  250,  -255, 255);  //  Velocity Control 13MAR17
      bbPidVC.setPoint = 30.0;  //  this distance set small to test the Object Detectors
      //bbPidVC.setPoint = theZone;  //  set in 'defines.h' Servo Definitions ~ line 117
  }

  void bb_control::resetMotion()
  {
      fbCLR( fbEvade);
      evadeStep = 0;
      fbCLR( fbProgram);
      programStep = 0;
      fbCLR( fbTrack);
      fbCLR( fbPivot);

      rDat1.velocity = 0;
      rDat1.rotation = 0;
      rDat1.course = rDat1.head;  //  match course to heading
      rDat1.bear = 0;             // set bearing direction to zero

  //    haltTimer = millis() + haltTime;  //  restart halt timer
  }

  //  = = = = = = =  Check Halt Time (whether platform is moving)  = = = = =
  //  Only check while in AUTO or PROGRAM mode
  //  Set TRUE if platform does not move for 'haltTime' milliseconds.
  #define haltTime 500                    //  Define haltTime as 1/2 second
  unsigned long haltTimer = haltTime;     //  Initialize halt timer
  void checkHalted()
  {
      // changed halt time to a total of 2 clicks instead of 0 - 21DEC16
      if( ( rDat1.intCountL + rDat1.intCountR) > 4) haltTimer = millis() + haltTime;
      fbBUT( fbHalt, ( haltTimer < millis()));
  }
  //  - - - - - - - - - -   End of CHECK HALT TIME  - - - - - - - - - -

  //  = = = = = = = = = = = =  Set Velocity  = = = = = = = = = = = =
  //  maintain safe platform distance from an IR reflective object
  void setVelocity()
  {
      //  The PID setup above makes 'setPoint' equal to 'theZone' (75cm)
      //  'inPut' to the PID is the rDat1.range in centimeters and
      //  a value somewhere between 'tooFar' (200) and 'tooNear' (25)
      bbPidVC.inPut = (float)rDat1.range;   //  retype range (distance) as floating point
      bbPidVC.Compute();                    //  tell PID to compute an output
//      int vel = (int)bbPidVC.outPut;  //  get velocity from PID as an integer
      int vel = bbPidVC.outPut;  //  get velocity from PID as an integer
      intConstrain( vel, -255, 255);         //  constrain velocity
      vel *= -1;                            //  reverse velocity  WHY?
      rDat1.velocity = vel;
      //
      #if( PID_CALIBRATE && PID_VELOCITY_CALIBRATE)
        //  Tune this PID controller with
        //  "processing\sketches\PID control\bb2_PID_Velocity.pde"
        bbPidVC.serialRX();
        bbPidVC.serialTX();
      #endif
  }
  //  - - - - - - - - - -  End of SET VELOCITY  - - - - - - - - -

  //  = = = = = = = = = = =  Set Rotation  = = = = = = = = = = =
  //  'Bearing' is the difference between the desired direction of
  //  the platform ('course'), and the actual direction ('heading')
  //  offset by the servo 'near' position, if active.
      void getBear()
      {
          rDat1.bear = rDat1.course - rDat1.head; 
          //if( fbCHK( fbServo) rDat1.bear -= rDat.nearPos;
          //  Constrain 'bearing' within ±180°
          if( rDat1.bear > 180) rDat1.bear -= 360;
              else if( rDat1.bear < -180) rDat1.bear += 360;
      }
  //
  //  Output a rotation value to minimize a bearing value.
  //  Called by 'track()' and 'pivot()' routines.
  void setRotation()
  {
      getBear();   //  Get course correction
      //  "setPoint" is set to zero in "setupPID" above
      //  "inPut" is the "bearing" or course correction value
      //  "outPut" value is from -180 to 180 degrees
      bbPidRC.inPut = (float)rDat1.bear;   //  retype bearing to floating point
      bbPidRC.Compute();        //  Tell PID controller to compute an output
//      int rot = (int)bbPidRC.outPut;     //  save PID output as integer
      int rot = bbPidRC.outPut;     //  save PID output as integer
      intConstrain( rot, -255, 255);      //  constrain rotation - done in PID
      rot *= -1;                         //  reverse rotation  WHY?
      rDat1.rotation = rot;
      //
      #if( PID_CALIBRATE && PID_ROTATION_CALIBRATE)
        //  tune this PID controller with
        //  "processing\sketches\PID control\bb2_PID_Rotation.pde"
        bbPidRC.serialRX();
        bbPidRC.serialTX();
        //  set velocity to zero so that platform rotates in place
        //  rDat1.velocity = 0;
      #endif
  }
  //  - - - - - - - -  End of SET ROTATION  - - - - - - - -

  //  = = = = = = = = = =  TRACK  = = = = = = = = = = =
  //  Called by the 'evade' and 'program' functions
  //  Run robot at given speed for given number of encoder clicks.
  //  Maintain compass heading while running.
  uint32_t trackLength = 0;
  //
      void setTrack( int speed, uint16_t length)  //
      {
          fbCLR( fbHalt);   //  isHalted = false;
          rDat1.velocity = speed;
          rDat1.rotation = 0;
          trackLength = eDatR.epc + (uint32_t)length;  // trackLength is given in encoder clicks
          fbSET( fbTrack);  //  isTrack = true;
      }
  //  Right now the platform is satisfied if it goes the correct distance or farther.
  //  Ideally, the platform should be more precise and reverse if it goes too far.
  void track()
  {
      setRotation();      //  setRotation calls getBear();
      if( eDatR.epc >= trackLength) fbCLR( fbTrack);  // clear Track flag bit
  }
  //  - - - - - - - -  End of TRACK  - - - - - - - -

  //  = = = = = = = =  PIVOT  = = = = = = = =
  //  Find the shorter angular distance between heading
  //  and course with getBear(). Pivot flag remains
  //  TRUE until platform is halted for 1/2 second.
      void setPivot( int angle)
      {
          fbCLR( fbHalt);  // isHalted = false;
          rDat1.velocity = 0;
          rDat1.course = angle;
          fbSET( fbPivot);  // isPivot = true;
      }
  //
  void pivot()
  {
      setRotation();      //  setRoatation calls getBear();
      if( rDat1.bear == 0) fbCLR( fbPivot); //  clear Pivot flag bit
  }
  //  - - - - - - - - - -  End of PIVOT   - - - - - - - -

  //  = = = = = = = = = = =  Sweep Scan  = = = = = = = = =
  //  If distance is too small:
  //  1. stop,
  //  2. do sweep scan
  //  3. turn in a random direction
  //  4. proceed

  //  = = = = = = = = = = =  Evade  = = = = = = = = = =
  //  If object is detected:
  //  1. stop,
  //  2. turn 25° toward object,
  //  3. move 200 clicks in reverse, away from object
  //  4. turn in a random direction
  //  5. proceed
  //
  uint8_t odFb;
  int newCourse;

      void bb_control::setEvade()
      {
          //  resetObjectDetect();
          //  sweep the scanner one time
          odFb = ( rDat1.flagBits >> 16) & 0x0F;              // single byte copy of eight OD Flag Bits
          newCourse = rDat1.course + 180 + random( -90, 90);  // set new course after evade;
          if( newCourse > 359) newCourse -= 360;              // constrain to less than 360
          evadeStep = 0;    //  zero the Evade step counter
          fbSET( fbEvade);  //  set the Evade flag
      }
  //
  void bb_control::evade()
  {
      ++evadeStep;
      switch( evadeStep)
      {
          case 1:    //  turn 45° toward object detected
              //odFb = (rDat1.flagBits >> 16) & 0x0F;  // get Object Detect Flag Bits
              if( ( odFb != 3) && (odFb != 12))  // not both front and not both back detectors
              {
                  if( odFb & 0x09)               // either left front or right rear detector
                  {
                      setPivot(  rDat1.course - 25);   // turn 25° to the left
                  }
                  else if( odFb & 0x06)          // either right front or left rear detector
                  {
                      setPivot(  rDat1.course + 25);   // turn 25° to the right
                  }
              }
          break;
          case 2:
              if( odFb & 0x03)           // if obstacle in front
              {
                  setTrack( -200, 200);  // go backward 200 clicks
              }
              else if( odFb & 0x0C)      // if obstacle in rear
              {
                  setTrack( 200, 200);   // go forward 200 clicks
              }
          break;
          case 3:  // pivot away from object toward a random, new direction
              //newCourse = rDat1.course + 180 + random( -90, 90);  // set new course after evade
              //if( newCourse > 359) newCourse -= 360;                  // constrain to less than 360
              setPivot( newCourse);
          break;
          case 4:
              fbCLR( fbEvade);           // clear evade flag
              evadeStep = 0;             // reset evade program counter
          break;
          default:
              controlReset();
          break;
      }
  }
  //  - - - - - - - - -  End of EVADE   - - - - - - - - -

  //  = = = = = = = = = = =  PROGRAM 1 = = = = = = = = =
  //  When button 2 flag is set, perform the sequence
  //  of steps.  Then reset counter and drop the flag.
      void bb_control::setProgram()
      {
          fbSET( fbAuto);    // isAuto = true;
          fbSET( fbMotor);   // isMotor  = true
          fbSET( fbProgram); // isProgram = true;    *Why?*          
          fbCLR( fbServo);   // stop the servo
          fbCLR( fbHalt);    // clear halt condition
          programStep = 0;
      }
  
  void bb_control::program()
  {
      ++programStep;             //  advance the program counter
      switch( programStep)
      {
        case  1:
          fbSET( fbMotor);   // isMotor  = true
          fbCLR( fbServo);   // isServo  = FALSE
          fbSET( fbAuto);    // isAuto = true;
          fbCLR( fbHalt);    // isHalted = FALSE
          fbSET( fbProgram); // isProgram = true;    *Why?*
        break;
        case  2: setPivot( 0); break;             // turn North
        case  3: setTrack( -200, 1000); break;    // back 1000
        case  4: setPivot( 90); break;            // turn East
        case  5: setTrack( 200, 1000); break;     // ahead 1000
        case  6: setPivot( 0); break;             // turn North
        case  7: setTrack( 200, 2000); break;     // ahead 2000
        case  8: setTrack( -200, 1000); break;    // back 1000
        case  9: setPivot( 270); break;           // turn West
        case 10: setTrack( 200, 1000); break;     // ahead 1000
        case 11: setPivot( 0); break;             // turn North
        case 12:
          fbCLR( fbAuto);      // isAuto = FALSE
          fbCLR( fbHalt);      // isHalted = FALSE
          fbCLR( fbProgram);   // isProgram = FALSE
          programStep = 0;
        break;
        default: controlReset(); break;
      }
  }
  //  - - - - - - - -   End of PROGRAM 1   - - - - - - - -

  //  = = = = = = = = = =  Autonomous Control  = = = = = = = = = =
  //  Platform maneuvers to find itself halted and centered
  //  in 'theZone' (400).
  //  1) Move forward between 'tooFar' and 'theZone'.
  //  2) Reverse between 'tooNear' and 'theZone'.
  //  3) 'Fault' if 'tooNear'.
  //  4) 'Halt' in 'theZone' (400).
  //  5) If 'Halt', then 'pivot' until servo centered.
  //  ------------------------------------------------------------
  void bb_control::autoControl()
  {
      //  The point of 'firstAuto' is to detect the first time through Auto Mode
      //  This sets up some housekeeping, and detects 'dropping out' of Auto Mode
      if( firstAuto)
      {
        #if( !PID_ROTATION_CALIBRATE)   //  and if not calibrating
          rDat1.course = rDat1.head;    //  set the course
        #endif
          fbCLR( fbHalt);                   //  clear Halt flag
          haltTimer = millis() + haltTime;  //  reset Halt timer
          bbPidRC.setControlMode( 1);   // set rotation PID controller to AUTO
          bbPidVC.setControlMode( 1);   // set velocity PID controller to AUTO
          firstAuto = false;            //  prime the first pass trigger
      }

      checkHalted();        //  set Halt flag bit if no motion for 1/2 second
      if( fbCHK( fbHalt))
      {
        #if( !PID_CALIBRATE)    // ----  DO NOT  EVADE IF TESTING  ----
        setEvade();             //  If Halted, perform Evade routine
        #endif                  // ------------------------------------
      }

      setVelocity();
      setRotation();  // this calls the getBear() routine
  }

  //  - - - - - - -   control Reset   - - - - - - - -
  //  Clear all the flagBits except for Buttons & OD Flags ( 0x00FF)
  //  and Manual, Reset, Object, Serial, Radio, Servo & Motor ( 0xE40D).
  //  Reset Object Detect by robot button #4, or by PC program.
  //  Why do we NOT reset OD flags?
  //
  //                   Stall Fault   Program Bear Track Pivot Halt Evade        Auto 
  //  Manual Reset Object       Serial                                Radio Servo   Motor
  //    1110 0100 0000 1101 - flag bit mask
  //      E4        0D      - hex value

  // bool firstAuto = false;
  void bb_control::controlReset()
  {
      firstAuto = true;                //  set TRUE when fbAuto flag falls
      rDat1.flagBits &= 0x00FFE40DUL;  //  clear flag bits
      resetMotion();  // clear all motion values ( see above )
  }

  void bb_control::control()
  {
      //  I have got to figure this control thing out somehow.
      //  I will begin by separating signal, sense and alarm functions from platform control

      //  After reading jDat from the radio,
      //  set motor, auto & servo mode from jDat flag bits and turn on the LEDs.
      //toggleFlagBits();    //  'toggleFlagBits()' includes 'showLED()' in 'alarm.cpp'
      
      //  The Platform is either in Manual or in Auto Mode
      //  Every automatic behavior, such as Program or Evade, has to come under Auto mode

      // = = = = = = = = = = = = =  Signal, Sense and Alarm   = = = = = = = = = = = = =

      //  Sense motor currents and if any too high, set 'fbStall' - in 'alarm.cpp'
      if( testMotorCurrent()) fbSET( fbStall);

      //  Sense corner object detectors in 'alarm.cpp' and
      //  if 'fbOject' is set and not calibrating then set Evade Mode
      if( testObjectDetect())
      {
          // Compile only if NOT calibrating velocity
          #if( !PID_VELOCITY_CALIBRATE)
          setEvade(); 
          #endif
      }

      if( fbCHK( fbManual) ||                 //  If joystick is jogged, or
        ( !fbCHK( fbAuto) && !firstAuto) ||   //  if 'dropping out' of Auto Mode, or
          fbCHK( fbStall) ||                  //  if there's a motor over-current condition, or
          fbCHK( fbFault))                    //  if there's any other fault condition...
      {
          controlReset();         //  ...then stop the platform and reset everything.
      }

      if( fbCHK( fbTrack)) track();            //  #1 - track as programmed
      else if( fbCHK( fbPivot)) pivot();       //  #2 - pivot as programmed
      else if( fbCHK( fbEvade)) evade();       //  #3 - perform evasion step and advance counter
      else if( fbCHK( fbProgram)) program();   //  #4 - perform program step and advance counter
      else if( fbCHK( fbAuto)) autoControl();  //  #5 - run free in auto mode
      else                                     //  #6 - if nothing else,
      {
          rDat1.velocity = jDat.velocity;      //  then perform manual joystick control
          rDat1.rotation = jDat.rotation;
      }

      bbPidRC.setControlMode( fbCHK( fbAuto));  // reset rotation PID controller
      bbPidVC.setControlMode( fbCHK( fbAuto));  // reset velocity PID controller
      bbMotor.control();    //  used to be in the T3 sub-loop, now here
  }

  // = = = = = = = = =   Diagnostic print   = = = = = = = = = = = = =
  // if( fbCHK( fbProgram)) printf(" Program is 'ON' here.\r\n");    |
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#endif