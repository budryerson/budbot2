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
 * 25APR19 - servo controls removed to Lidar section
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
  bb_alarm bbAlarm;

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
      bbAlarm.setup();  // set motor current sense pins
                             // set platform button pins
                             // set platform LED pins
      bbMotor.motorSetup();  //  set motor speed and direction pins
                             //  clear motor data structures
                             //  initialize motor PID controls
      //  bbMotor.motorReset();    //  velocity = 0; difference = 0; motorRun = FALSE

      clearDataStructures();    //  in 'routines.cpp'
      rDat1.mpxID = RDAT1_ID;   //  set to 0x0096 or 0b10010110 - 'defines.h'
      rDat2.mpxID = RDAT2_ID;   //  set to 0x0069 or 0b01101001 - 'defines.h'
 //     fbCLR( fbAuto);  //  clear Auto Mode
      evadeStep = 0;
      programStep = 0;

      // Set parameters for the gross motion PID Controllers
      // Velocity control parameters:
      //                  Kp    Ki   Kd   min/max In  min/max Out
      //bbPidVC.setupPID( 5.0,  0.5, 1.0,  17,  250,  -255, 255);  //  original
      bbPidVC.setupPID( 10.0,  1.0,  5.0,  20,  250,  -255, 255);  //  13MAR17
      //bbPidVC.setPoint = theZone;  //  set in 'defines.h' Servo Definitions ~ line 117
      bbPidVC.setPoint = 30.0;  //  this distance set small to test the Object Detectors
      // Rotation control parameters:
      //                Kp    Ki   Kd   min/max In  min/max Out
      bbPidRC.setupPID( 2.0, 0.003, 1.0, -180,  180, -255, 255);
      bbPidRC.setPoint =  0.0;  // initial setPoint
  }

  void bb_control::resetMotion()
  {
      fbCLR( fbEvade);
      evadeStep = 0;
      fbCLR( fbProgram);
      programStep = 0;
      fbCLR( fbTrack);
      fbCLR( fbPivot);

      rDat3.velocity = 0;
      rDat3.rotation = 0;
      rDat1.course = rDat1.head;  //  match course to heading
      rDat1.bear = 0;             // set bearing direction to zero

  //    haltTimer = millis() + haltTime;  //  restart halt timer
  }

  //  = = = = = = =  Check Halt Time (whether platform is moving)  = = = = =
  //  Only check while in AUTO or PROGRAM mode
  //  Set TRUE if platform does not move for 'HALT_INTERVAL' milliseconds.

  //uint32_t haltTimer = HALT_TIME;    //  Initialize halt timer
  void checkHalted()
  {
      // If either wheel moves more than 2 clicks...
//      if( rDat1.intCountL > 2 || rDat1.intCountL > 2)
      if( rDat3.velocity > 2 || rDat3.rotation > 2)
      {
          rDat3.haltTimer = millis() + HALT_INTERVAL;  // ...reset halt timer.
      }
      // set or clear HALT flag according to whether timer has expired
      fbBUT( fbHalt, ( rDat3.haltTimer < millis()));
  }
  //  - - - - - - - - - -   End of CHECK HALT TIME  - - - - - - - - - -

  //  = = = = = = = =  Check Sweep Time   = = = = = = = =
  //  Only while in AUTO or PROGRAM mode, check whether it
  //  is time to stop everything and perform target sweep.
  #define SWEEPTIME 14000                 //  Define SWEEPTIME as 7 second
  unsigned long sweepTimer = SWEEPTIME;  //  Initialize sweep timer
  void checkSweepTimer()
  {
      if( sweepTimer < millis())
      {
//        fbCLR( fbMotor);
//        fbSET( fbServo);
//        sweepServo();
        fbSET( fbServo);
        fbSET( fbSeek);
        sweepTimer = millis() + SWEEPTIME;  // reset sweep timer.
//        fbSET( fbMotor);
      }
  }
  //  - - - - - - - - -   End of CHECK SWEEP TIME  - - - - - - - - - -


  //  = = = = = = = = = = = =  Set Velocity  = = = = = = = = = = = =
  //  maintain safe platform distance from Lidar scanned object
  void setVelocity()
  {
      //  The PID setup above makes 'setPoint' equal to 'theZone' (75cm)
      //  'inPut' to the PID is the distance in centimeters and
      //  a value somewhere between 'tooFar' (200) and 'tooNear' (25)
      bbPidVC.inPut = (float)rDat1.dist;    //  re-type distance as floating point
      bbPidVC.Compute();                    //  tell PID to compute an output
   // - - - - All of this is done in 'bb_pid.cpp'  - - - -
   //   int vel = (int)bbPidVC.outPut;        // get velocity from PID as an integer
   //   int vel = bbPidVC.outPut;             // get velocity from PID as an integer
   //   vel = intConstrain( vel, -255, 255);  // constrain velocity
   //   vel *= -1;                            // reverse velocity  WHY?
      rDat3.velocity = bbPidVC.outPut * -1;
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
          //if( fbCHK( fbServo) rDat1.bear -= rDat3.nearDex;
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
      bbPidRC.inPut = (float)rDat1.bear;   //  retype 'bear' as floating point
      bbPidRC.Compute();        //  Tell PID controller to compute an output
   // - - - - All of this is done in 'bb_pid.cpp'  - - - -
   //   int rot = (int)bbPidRC.outPut;        // save PID output as integer
   //   int rot = bbPidRC.outPut;             // save PID output as integer
   //   rot = intConstrain( rot, -255, 255);  // constrain rotation - done in PID
   //   rot *= -1;                            // reverse rotation  WHY?
      rDat3.rotation = bbPidRC.outPut * -1;
      //
      #if( PID_CALIBRATE && PID_ROTATION_CALIBRATE)
        //  tune this PID controller with
        //  "processing\sketches\PID control\bb2_PID_Rotation.pde"
        bbPidRC.serialRX();
        bbPidRC.serialTX();
        //  set velocity to zero so that platform rotates in place
        //  rDat3.velocity = 0;
      #endif
  }
  //  - - - - - - - -  End of SET ROTATION  - - - - - - - -

  //  = = = = = = = = = =  TRACK  = = = = = = = = = = =
  //  Called by the 'evade' and 'program' functions
  //  Run robot at given speed for given number of encoder clicks.
  //  Maintain compass heading while running.
  uint32_t trackLength = 0;  // trackLength is actually final click value
  uint32_t saveClickR, saveClickL;
  //
      void setTrack( int speed, uint16_t length)  //
      {
          fbCLR( fbHalt);   //  isHalted = false;
          rDat3.velocity = speed;
          rDat3.rotation = 0;
          // trackLength is given in encoder clicks
          trackLength = eDatR.epc + (uint32_t)length;
          saveClickR = eDatR.epc + length;
          saveClickL = eDatL.epc + length;
          fbSET( fbTrack);  //  isTrack = true;
      }
  //  Right now the platform is satisfied if it goes the correct distance or farther.
  //  Ideally, the platform should be more precise and reverse if it goes too far.
  void track()
  {
      setRotation();      //  setRotation calls getBear();
      if( ( eDatR.epc >= saveClickR) &&
          ( eDatL.epc >= saveClickL)) fbCLR( fbTrack);  // clear Track flag bit
  }
  //  - - - - - - - -  End of TRACK  - - - - - - - -

  //  = = = = = = = =  PIVOT  = = = = = = = =
  //  Find the shorter angular distance between heading
  //  and course with getBear(). Pivot flag remains
  //  TRUE until platform is halted for 1/2 second.
      void setPivot( int angle)
      {
          fbCLR( fbHalt);  // isHalted = false;
          rDat3.velocity = 0;
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
  static uint8_t odFb;   // object detect flag bit
  int newCourse;

      void bb_control::setEvade()
      {
          //  resetObjectDetect();
          odFb = (uint8_t)( rDat1.flagBits >> 16) & 0x0F;     // make copy of four OD Flag Bits          
          // stop the platform
          rDat3.velocity = 0;    // zero motion commands
          rDat3.rotation = 0;
          // sweep the scanner one time
          fbSET( fbServo);       // sweep the servo right and left
          fbSET( fbSeek);        // this is reset at end of scan
          evadeStep = 0;    //  zero the Evade step counter
          fbSET( fbEvade);  //  set the Evade flag
      }
  //
  void bb_control::evade()
  {
      fbCLR( fbEvade); 

/*      ++evadeStep;
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
          break;  */
          
  /*      case 1:
              if( odFb & 0x03)           // if obstacle Front Left or Front Right
              {
                  setTrack( -200, 200);  // go backward 200 clicks
              }
              else if( odFb & 0x0C)      // if obstacle RearLeft or RearRight
              {
                  setTrack( 200, 200);   // go forward 200 clicks
              }
          break;

          case 2:  // pivot away from object toward a random, new direction
              //newCourse = rDat1.course + 180 + random( -90, 90);  // set new course after evade
              //if( newCourse > 359) newCourse -= 360;                  // constrain to less than 360
              setPivot( newCourse);
          break;
          case 3:
              fbCLR( fbEvade);           // clear evade flag
              evadeStep = 0;             // reset evade program counter
          break;
          default:
              controlReset();
          break;
      }
  */
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
  #define SCAN_TIME 5000       // 5 sec.
  uint32_t scanTimer;          // run scan routine every 5 seconds
  #define WAIT_TIME 1000       // 1 sec.  
  uint32_t waitTimer;          // delay after zero motion
  void bb_control::autoControl()
  {
      //  The point of 'firstAuto' is to detect the first time through Auto Mode
      //  This sets up some housekeeping, and detects 'dropping out' of Auto Mode
      if( firstAuto)
      {
        #if( !PID_ROTATION_CALIBRATE)   //  and if not calibrating
          rDat1.course = rDat1.head;    //  set the course
        #endif
          fbCLR( fbHalt);               //  clear Halt flag
          fbCLR( fbServo);
          fbCLR( fbSeek);
          bbPidRC.setControlMode( 1);   // set rotation PID controller to AUTO
          bbPidVC.setControlMode( 1);   // set velocity PID controller to AUTO
          //scanTimer = millis() + SCAN_TIME;
          rDat3.seekTimer = millis() + SEEK_INTERVAL;
          rDat3.haltTimer = millis() + HALT_INTERVAL;
          firstAuto = false;            //  prime the first pass trigger
      }
      
      if( fbCHK( fbServo))                 // If SERVO is TRUE...
      {
          if( rDat3.seekTimer < millis())  // when 'seekTimer' exzpires...
          {
              fbSET( fbSeek);              // ...set SEEK flag to TRUE.
          }
      }

      // If either HALT or SEEK is TRUE, then...
      if( fbCHK( fbHalt) || fbCHK( fbSeek))
      {
          rDat3.velocity = 0;   // ...set motion commands to zero
          rDat3.rotation = 0;
      }
      else                   // Otherwise...
      {
          setVelocity();     // set PID motion according to distance
          setRotation();     // calling the 'getBear' routine.
          checkHalted();     // check if platfrom is moving
      }

/*    if( fbCHK( fbHalt))
      {
        #if( !PID_CALIBRATE)    // ----  DO NOT  EVADE IF TESTING  ----
        setEvade();             //  If Halted, perform Evade routine
        #endif                  // ------------------------------------
      }
*/
  }

  //  - - - - - - -   control Reset   - - - - - - - -
  //  Clear all the flagBits except for Buttons & OD Flags ( 0x00FF)
  //  and Manual, Reset, Object, Serial, Radio & Motor ( 0xE409).
  //  Reset Object Detect by robot button #4, or by PC program.
  //  Why do we NOT reset OD flags?
  //
  //                   Seek  Fault   Program Bear Track Pivot Halt Evade  Servo Auto
  //  Manual Reset Object       Serial                                Radio        Motor
  //    1110 0100 0000 1001 - flag bit mask
  //      E4        09      - hex value
  void bb_control::controlReset()
  {
      firstAuto = true;                //  set TRUE when fbAuto flag falls
      rDat1.flagBits &= 0x00FFE409UL;  //  clear flag bits
      resetMotion();  // clear all motion values ( see above )
  }

  void bb_control::control()
  {
      //  20171227 - I have got to figure this control thing out somehow.
      //  I begin by separating signal, sense and alarm functions from platform control

      //  The Platform is either in Manual or in Auto Mode
      //  Every automatic behavior, such as Program or Evade, has to occur under Auto mode
      bbAlarm.checkPlatformButtons();

      //  Scanning servo occurs in either mode
  //  checkSweepTimer();
   //   bbServo.sweep();    //  Get Lidar distance data and save as
                          //  unsigned 16 bit integer 'rDat1.dist'.
                          //  Save servo position as signed 16 bit integer to 'rDat1.sPos'
                          //  and into private servo array for target acquisition..
                          //  If 'fbScan' then move servo to new position.

      // = = = = = = = = = = = = =  Signal, Sense and Alarm   = = = = = = = = = = = = =

      //  - - - -   Alarm functions are always on  - - - -
      //  Get motor current values and set 'fbFault' if any too high ('alarm.cpp')
      //  *** Prevent reset of fbFault by fbManual for 3 seconds - not implemented yet ***
      //if( bbAlarm.testMotorCurrent()) fbSET( fbFault);

      //  Sense corner object detectors ('alarm.cpp') and
      //  if 'fbOject' is set and not calibrating then set Evade Mode
      // Compile only if NOT calibrating velocity
      #if( !PID_VELOCITY_CALIBRATE)
      //if( bbAlarm.testObjectDetect())  setEvade();
      #endif

      if( fbCHK( fbManual) ||                 //  If joystick is jogged, or
        ( !fbCHK( fbAuto) && !firstAuto) ||   //  if 'dropping out' of Auto Mode, or
          fbCHK( fbFault))                    //  if motor over-current condition...
      {
          // ...then stop the platform and reset the following flags:
          // Seek, Fault, Program, Bear, Track, Pivot, Halt, Evade, Servo & Auto
          //bbAlarm.printStatus();
          controlReset();
      }

      if( fbCHK( fbTrack)) track();            //  #1 - track as programmed
      else if( fbCHK( fbPivot)) pivot();       //  #2 - pivot as programmed
      else if( fbCHK( fbEvade)) evade();       //  #3 - perform evasion step and advance counter
      else if( fbCHK( fbProgram)) program();   //  #4 - perform program step and advance counter
      else if( fbCHK( fbAuto)) autoControl();  //  #5 - run free in auto mode
      else                                     //  #6 - if nothing else,
      {
          //  Perform manual joystick control
          rDat3.velocity = jDat.velocity;   //  These values are calculated
          rDat3.rotation = jDat.rotation;   //  in 'joystick.cpp'
      }

      //  why run the motor control if platform is in 'seek' mode?
      //  'setControlMode' if 'newAuto' or first time in auto mode
      //  'fbCHK()' returns a boolean value: 0 = false, 1 = true
      bbPidRC.setControlMode( fbCHK( fbAuto));  // reset rotation PID controller
      bbPidVC.setControlMode( fbCHK( fbAuto));  // reset velocity PID controller
      bbMotor.control();    //  used to be in the T3 sub-loop, now here

      //  After having read jDat from the radio on the previous loop,
      //  set 'motor', 'auto' & 'servo' mode from jDat flag bits
      //  and illuminate the corresponding LEDs.
      bbAlarm.toggleFlagBits();         //  This is in 'alarm.cpp'
  }

  // = = = = = = = = =   Diagnostic print   = = = = = = = = = = = = =
  // if( fbCHK( fbProgram)) printf(" Program is 'ON' here.\r\n");    |
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#endif