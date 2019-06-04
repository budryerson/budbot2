/* File Name: bb_servo.cpp
 * Developer: Bud Ryerson
 * Described: One of the first things I ever wrote.
     This controls setup, testing and control of the servo panning motion
     of the IR sensor as well as interpretation of data from the sensor.
 * Inception: Probably inherited from budbot1
 * 26DEC15 - converted servo from "bit bang" to
             Atmega's internal PWM generator
 * 07JAN16 - changed servo direction to deviation value
             simplified servo pulse code
 * 24SEP16 - Folded the "eye" library into the "servo" library
             in order to manipulate the position of sensor values
             that appear offset depending on the scan direction.
 * 10DEC16 - reworked sweep routine, eliminated servo reset
 * 20FEB17 - modified servo to work more precisely and not to
             pulse when Servo is OFF and the servo is centered
 *  8MAR17 - Installed a separate 5V supply for IR sensor
 * 30MAR17 - Replaced burned out analog HiTEc HS322 servo motor with
             digital Tower Pro MG996R.
             Installed another separate 5V supply for servo motor.
             Got rid of IR sensor ring buffer.
             Now sample IR sensor values 10X for each servo reposition.
 * 26FEB19 - Replaced entire IR sensor code with TFMini-Plus Lidar code.
             Added MegaServo library to replace the custom servo timing code.
 * 07MAR19 - Screw the MegaServo library. Went back to hard code for servo PWM.
             Added interrupt on/off when setting OC registers
 * 28MAR19 - took lidar out of servo and created separate routine library
 * 11APR19 - Finally got rid of 'servoOffset'. Mystery solved!
 * 25APR19 - Servo routines moved to and now called by Lidar routines
 * Should someday include these features:
        Scan mode,
        encounter mode,
        pursuit mode
 */

//  = = = = = = = =  Compile this file ONLY for the MEGA2560  = = = = = = = =
#ifdef __AVR_ATmega2560__

    #include <Arduino.h>
    #include <bb_defines.h>
    #include <bb_shared.h>
    #include <bb_routines.h>
    #include <bb_notes.h>

    #include <bb_servo.h>
    bb_servo::bb_servo(){}
    bb_servo::~bb_servo(){}

    // servos are attached to pins 6 & 7 and rotate from 0-180 degrees by 1 degree increments.
    int azPin = 6;     // output pin of OC4A is azimuth (az) pin
    int elPin = 7;     // output pin of OC4B is elevation (el) pin

    // Definitions for the servo library
    // The robot patform coordinate system ranges from -90 (left) to +90 (right).
    // The servo coordinate system ranges from 0 to 180 degrees with 90 being straight ahead
    #define SERVO_MID     90    // zero degrees - level and straight ahead position

    #define SERVO_MIN    -90    // full left (counter-clockwise from above) and down
    #define SERVO_MAX     90    // full right (clockwise from above) and up

    #define SWEEP_MIN    -60    // sweep direction limits
    #define SWEEP_MAX     60    // cannot be more than 70

    #define tooNear       10    // distance in centimeters
    #define tooFar       250
    #define theZone       50

    // deadband values prevent the robot over reacting to small movements of the object
    #define dxDeadBand   100    // average rDat1.logDist change without robot body reacting
    #define pxDeadBand   300    // average pan change without robot body reacting

//    #define    SERVO_CENTER     1520   // center (neutral) servo position in µsec
//    #define    SERVO_ONE_DEG       9   // one degree of servo position in µsec
//    #define    SERVO_TWO_DEG      19   // two degrees of position in µsec

    int elPos = SERVO_MID;  // initialize elevation position

    //uint16_t distRay[ 181];  // array of Lidar distance values
                             // for each servo index value

    int azInc = 1;    // azimuth increment value
    int elInc = 1;    // elevation increment value
    int scanQuad = 1; // scan quadrant 1 -4

    //  - - - - - - - -  Servo Setup  - - - - - - - - - - - -
    //  Configure register values and output pin for servo
    void bb_servo::setup()
    {
        setupTimer4();
        delay(20);                 // Wasit for timer to initalize

        pinMode( azPin, OUTPUT);   // set Pin 6 (green) as azimuth servo output
        pinMode( elPin, OUTPUT);   // set Pin 7 (yellow) as elevation servo output

        servoTest();               //  swivel servo back and forth
        resetServo();              //  reset servo level/front and turn OFF

        delay(20);                 // And wait a moment
    }
    //  - - - - - -  End of Servo Setup  - - - - - - - - - -

    //  Configure register values and output pin for servo
    void bb_servo::resetServo()
    {
        memset( &srvDat.distRay, 0, sizeof( srvDat.distRay));    // clear the Servo Position Array
        srvDat.azDex = 90;           // set azimuth idex to center
        srvDat.azStep = -1;          // initialize sweep direction to the left
        rDat1.azPos = srvDat.azDex;  // save index to rDat1 for transmittal
        servoSet();                  // strobe the servo
        //fbCLR( fbServo);    // Servo = OFF
        fbCLR( fbSeek);       // Seek  = OFF
        fbCLR( fbHalt);       // is this necessary?
        rDat3.seekTimer = millis() + SEEK_INTERVAL;
        rDat3.haltTimer = millis() + HALT_INTERVAL;
        firstScan = true;     // for 'scanOnce()' routine
    }

    //  = = = = = = = = = =  Servo Set  = = = = = = = = = = = = = =
    //  Set azimuth and elevation servo positions in T4 timer ticks
    //  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
    void bb_servo::servoSet()
    {
        // convert azimuth index value to Timer4 ticks
        int azTick = floatToInt( srvDat.azDex * 1.22) + 55;
        // constrain range of ticks to 55 - 275 or ±90°
        azTick = intConstrain( azTick, 55, 275);
        // do the same for elevation servo position
        //int elTick = floatToInt( ( rDat1.elPos + 90) * 1.22) + 55;
        //elTick = intConstrain( elTick, 55, 275);    // limit position within ±90°

        // Set Output Compare registers for Timer 4A & 4B
        cli();     // suspend all interrupts while setting registers
          OCR4A = azTick;    // set according to azimuth servo position
          OCR4B = 180;       // set to counter ticks for 90°
        sei();     // re-enable all interrupts
    }

    //  = = = = = = = = = =  Pan servo back and forth  = = = = = = = = = =
    //  If Servo is OFF, pulse only until centered and then cease pulsing.
    //#define SEEK_TIME 5000        //  stop and seek target every 5 secs
    //uint32_t seekTimer;
   //srvDat.azStep = -1;           //  Initialize sweep direction to the left
    void bb_servo::sweep()
    {
        if( fbCHK( fbServo))       // If SERVO is TRUE do one of two things
        {
            if( fbCHK( fbAuto))        // If in AUTO mode...
            {
                if( fbCHK( fbSeek))    // and if SEEK is TRUE...
                {
                    scanOnce();        // immediately move to left postion and settle
                }                      // sweep once right, once left and return to center
            }
            else
            {
                if( srvDat.azDex <= 30)    // If left limit...
                {
                    srvDat.azDex = 30;     // constrain the position...
                    srvDat.azStep = 1;     // and set direction right.
                    getTargets();
                }
                else if( srvDat.azDex >= 150) // If at right limit...
                {
                    srvDat.azDex = 150;       // constrain the position...
                    srvDat.azStep = -1;       // and set direction left.
                    getTargets();
                }
                srvDat.azDex += srvDat.azStep;  // Increment the position...
                servoSet();                     // and pulse the servos.
            }
        }
        else                       //  If Servo is OFF,
        {                          //
            if( srvDat.azDex != 90)       //  pulse servo until centered
            {                      //
                if ( srvDat.azDex > 90) --srvDat.azDex;
                else if ( srvDat.azDex < 90 ) ++srvDat.azDex;
                servoSet();
            }
            else
            {
              resetServo();
            }
        }
    }

    //  = = = = = = = =  Sweep servo back and forth one time  = = = = = = = =
    //  At set intervals, stop and scan once back and forth.
    //  This may be more useful than continuous panning since landmarks can
    //  be sensed, stored, and possibly identified as location beacons
    #define SCAN_ONCE_DELAY   400UL     //  Half second in milliseconds
    uint32_t sodTimer;                  //  Scan Once Delay Timer
    void bb_servo::scanOnce()
    {
        if( firstScan)                  // initialized 'true' in setup
        {
            srvDat.azStep = 1;          //  Initialize sweep direction to the right
            srvDat.azDex = 30;          //  set servo position to far left
            // set timer to current time plus delay
            sodTimer = millis() + SCAN_ONCE_DELAY;
            firstScan = false;          // reset firstScan
        }
        if( sodTimer < millis())       // When millis() catches up to timer
        {
            if( srvDat.azDex <= 150)   // If not at right limit
            {
                servoSet();            //  pulse the servos
                ++srvDat.azDex;
            }
            else
            {
                getTargets();        // getr near and far targets
                resetServo();        // center servo and clear flags.                
            }
        }
    }

    // This will scan through the Lidar distance value array
    // amd identify the nearest and furthest values to guide
    // the AUTO-directed motion of the platform.
    // • SEEK mode will steer toward the nearest target.
    // • FLEE will steer toward the furthest target.
    int nDex, fDex;  // target indices
    void bb_servo::getTargets()
    {
        rDat3.nearDex = 90;  // nearest point index
        rDat3.farDex  = 90;  // furthest point index
        // scan distance value arrray from west to east
        for( int i = 30; i <= 150; ++i)
        {
          // find the nearest point
          if( srvDat.distRay[ i] < srvDat.distRay[ rDat3.nearDex]) rDat3.nearDex = i;
          // find the furthest point
          if( srvDat.distRay[ i] > srvDat.distRay[ rDat3.farDex]) rDat3.farDex = i;
        }
        // rDat3.nearDex = nDex;
        // rDat1.course = rDat3.nearDex - 90;
    }


/*    //  = = = = = = =  Check Scan Timer  = = = = =
    //  Stop everything and perform scan.
    //  Only check while in AUTO or PROGRAM mode
    #define SCANTIME 5000                   //  Define SCANTIME as 5 seconds
    unsigned long scanTimer = SCANTIME;     //  Initialize scan timer
    void bb_servo::checkScanTimer()
    {
      if( scanTimer < millis())             // if scan timer expired
      {
          servoTest();
          scanTimer = millis() + SCANTIME;  // reset scan timer.
      }
    }
*/

    //  = = = = = = = = = =  Servo Test  = = = = = = = = = =
    //  Swivel the neck from -90 to +90 degrees and return to center
    void bb_servo::servoTest()
    {
        srvDat.azDex = 90;  //  set azimuth index to center
        srvDat.azStep = -1;           //  set position step leftwards
        for( int x = 0; x < 360; ++x)
        {
            //  if either limit encountered, flip the position increment sign
            if( srvDat.azDex <= 0 || srvDat.azDex >= 180)
            {
              playNote( 2);
              srvDat.azStep *= -1;   // reverse the sign
            }
            srvDat.azDex += srvDat.azStep;        //  increment position
            servoSet();
            milliDelay( 10);
        }
        while( srvDat.azDex != 90)  // pulse servo until centered
        {
            if ( srvDat.azDex > 90) --srvDat.azDex;
            else if ( srvDat.azDex < 90 ) ++srvDat.azDex;
            servoSet();
            milliDelay( 10);
        }
        playNote( 2);
        fbCLR( fbServo);       //  set Servo Mode OFF
        //resetServo();  // turn Servo OFF and return to center
    }

    void bb_servo::printServoData()
    {
        printf( "Pan Position: % 3i |", rDat1.azPos);     //  A = Azimuth
        printf( "Distance: %u |", rDat1.dist);
      //  printf( "Pan Pulse: % 3i", map( rDat1.azPos,  SERVO_MIN, SERVO_MAX, 70, 290));
        printf("\r\n");
    }

 /*   void bb_servo::tilt()
    {
      if( elPos <= lowStop || elPos >= highStop)   // if elevation too high or low
      {
        elPos = highStop;     // set elevation to highStop
        if( elInc != 0)       // if not on secondary scan
        {
          elInc = 0;          // set flag to secondary scan
          elPos += 10;        // offset highStop position
        }
        else elInc = 1;       // set flag to primary scan
      }
      // set servo position between -20° and +20° (1070 - 1920 usec)
      elServo.writeMicroseconds( (int)elPos);
      elPos -= SERVO_TWO_DEG;
    }
*/

#endif  //  #if defined(__AVR_ATmega2560__)