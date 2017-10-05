/* File Name: bb_servo.cpp
 * Developer: Bud Ryerson
 * Described: One of the first things I ever wrote.
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
 * 30MAR17 - Replaced burned out analog HiTEc HS322 servo motor with digital Tower Pro MG996R
             Installed another separate 5V supply for servo motor.
             Got rid of IR sensor ring buffer.
             Now sample IR sensor values 10X for each servo reposition.
 * Should include these features: Scan mode, encounter mode, pursuit mode
 */

//  = = = = = = = =  Compile this file ONLY for the MEGA2560  = = = = = = = =
#ifdef __AVR_ATmega2560__

#include <Arduino.h>
#include <bb_defines.h>
#include <bb_shared.h>
#include <bb_routines.h>

#include <bb_servo.h>
bb_servo::bb_servo(){}
bb_servo::~bb_servo(){}

int srvPosOff;               //  servo position plus offset
uint16_t srvRay[ 181];       //  array of IR values
uint16_t servoOffset;        //  offset value for servo position used in servoPulse()
//uint16_t minPos = 90;      //  servo position of minimum range value

//  Configure register values and output pin for servo
void bb_servo::setup()
{
    //  = = = = = = = =  IR Scanner Positioning  = = = = = = = =
    //  A 16MHz clock is divided by a 'prescalar' value of 64 to
    //  create a 250kHz clock.
    //  A 10 bit register counts up to 1024 and down again to
    //  create a 122.25Hz frequency, which has a 8.2ms wavelength.
    //  The output pin switches state at every OC4A value compare
    //  thus creating a variable width pulse between 0.9 - 2.1ms.
    //
    //  stop all interrupts
    cli();
      TCCR4A = 0b10100011; // _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);
      //  Bits 7-6: Compare Output Mode for Channel A1
      //    Clear OC4A on compare match;
      //    Set compare to occur at BOTTOM;
      //    Set output to low level (non-inverting mode).
      //  Bits 5-4: Compare Output Mode for Channel B1
      //    Set the same as for Channel A1.
      //  Bits 3-2: Compare Output Mode for Channel C1
      //    Set to zero
      //  Bits 2-0: Waveform Generation Mode
      //    Phase Correct, 10 bit (1024 count)
      TCCR4B = 0b00000011; // TCCR4B = _BV(CS41) | _BV(CS40);
      //    16M clock / 64 prescalar / 2048 (1024 up & down count)
      OCR4A = 180;
      //  A 1.5 millisecond pulse on Digital Pin 6
      //  should result in a 90° servo position
      OCR4B = 180;
      //  Do the same on Digital Pin 7
    sei();
    //  allow interrupts again

    pinMode( servoPin, OUTPUT);  // set Pin 6 (blue) as servo output
    pinMode( irPin, INPUT);      // IR Sensor analog in pin A01
                                 // Do not use INPUT_PULLUP
    resetServo();
}

//  Configure register values and output pin for servo
void bb_servo::resetServo()
{
    memset( &srvRay, 250, sizeof( srvRay));    // clear the Servo Position Array
    rDat1.srvPos = 0;   //  set position to center
    servoOffset = 0;    //  set offset to zero
    servoPulse();       //  strobe the servo
    firstScan = true;
    fbCLR( fbServo);    //  set Servo Mode to OFF
}

//  = = = = = = = = = =  IR Read  = = = = = = = = = =
//  Get values from IR sensor, correct and average.
//  Number of reads to average is in 'define.h' as IR_SAMPLE_SIZE
//  Roughly convert the average to a distance in centimeters and
//  return as an unsigned 16 bit integer called 'range'
uint32_t irTotal = 0;              //  total of all IR samples
void bb_servo::irRead()
{
    irTotal = 0;                       //  clear total of all IR samples
    for( int x = 0; x < IR_SAMPLE_SIZE; ++x)
    {
        //  The IR sensor signal adds a half volt pulse about every
        //  millisecond that lasts less than 200 microseconds.
        //  The following cleverness tries to avoid reading that pulse.
        uint16_t irVal1, irVal2;              //  Declare two local variables.
        irVal1 = readADCInput( irPin);        //  Get one IR sensor value,
        microDelay( 200);                     //  wait longer than the bump duration,
        irVal2 = readADCInput( irPin);        //  and get another IR value.
        if( irVal2 < irVal1) irVal1 = irVal2; //  Keep the smaller of the two,
        irTotal += irVal1;                    //  and add it to the total.
    }
    //  divide IR total by IR sample size to get average IR value
    rDat1.avgVal = (uint16_t)(longDiv( irTotal, IR_SAMPLE_SIZE));
    //  Do rough conversion of IR sensor values to physical distance in centimeters
    //  from manufacturer's data sheet. Valid between about 8 and 80 centimeters
    int mapVal = map( rDat1.avgVal, 82, 532, 100, 1100);
    intConstrain( mapVal, 80, 1200);
    rDat1.range = (uint16_t)(intDiv( 20000, mapVal));  // save as unsigned integer
    srvRay[ rDat1.srvPos + 90] = rDat1.range;    //  populate servo array with range values
}

//  = = = = = = = = = =  Servo Pulse  = = = = = = = = = =
//  Offset, constrain and map servo position to a determined set of values for
//  "Output Compare 4A"  to create a proper pulse width to drive the servo motor.
//  ** Considering the number of times I have had to replace the servo motor,
//  ** I need an easier way to match the OCR4A range to the servo angle range.
//  NOTE: Reducing OC4A value will move the servo to the left.
void bb_servo::servoPulse()
{
    srvPosOff = rDat1.srvPos - servoOffset;             // offset servo position
    intConstrain( srvPosOff, SERVO_MIN, SERVO_MAX);       // limit position to ±90°
    OCR4A = map( srvPosOff, SERVO_MIN, SERVO_MAX, 279, 48);
    // printServoData();
}

//  = = = = = = = = = =  Pan servo back and forth  = = = = = = = = = =
//  If Servo is OFF, pulse only until centered and then cease pulsing.
//  Oddly, the servo reports different angles whether panning right or left.
//  Also, different brands of servos behave differently.
//  The servo offset value 'servoOffset' is added to compensate for that behavior.
int sweepStep = -1;                   //  Initialize sweep direction to the left
void bb_servo::sweep()
{
    if( fbCHK( fbServo))              //  If Servo is ON,
    {
        rDat1.srvPos += sweepStep;    //  then pan servo along horizontal.
        if( rDat1.srvPos < SWEEP_MIN)  //  If too far left,
        {
            sweepStep = 1;            //  set pan direction right.
            servoOffset = -4;         //  servo offset, not to exceed ±70
            rDat1.srvPos = SWEEP_MIN + 1;
            getTarget();
        }
        else if( rDat1.srvPos > SWEEP_MAX) //  If too far right,
        {
            sweepStep = -1;           //  set pan direction left.
            servoOffset = 4;          //  offset not to exceed ±70
            rDat1.srvPos = SWEEP_MAX - 1;
            getTarget();
        }
        servoPulse();                  //  then pulse the servo
    }
    else                         //  If Servo is OFF,
    {                            //
        if( rDat1.srvPos != 0)   //  pulse servo until centered
        {                        //
            if ( rDat1.srvPos > 0) --rDat1.srvPos;
            else if ( rDat1.srvPos < 0 ) ++rDat1.srvPos;
            servoPulse();        //
        }                        //
        else servoOffset = 0;    //  zero offset when centered
                                 //  no pulse when centered
        // Test print for stuttering servo
        //   printf("Servo Dir: % 04i", rDat1.srvPos);
        //   printf("\r\n");
    }
}

//  = = = = = = = =  Sweep servo back and forth one time  = = = = = = = =
//  Periodically, robot will stop and scan one time.
//  This may be more useful than continuous panning since landmarks can
//  be sensed, stored, and possibly identified as location beacons
void bb_servo::scanOnce()
{
    if( fbCHK( fbServo))              //  If Servo is ON,
    {
        if( firstScan)
        {
            rDat1.srvPos = SWEEP_MIN;   //  then pan servo along horizontal.
            servoOffset = -4;           //  servo offset, not to exceed ±70
            servoPulse();
            firstScan = false;
        }
        else
        {
            if( rDat1.srvPos < SWEEP_MAX)
            {
                ++rDat1.srvPos;
                servoPulse();
            }
            else
            {
                getTarget();
                rDat1.srvPos = SERVO_MID;
                servoOffset = 0;           //  servo offset, not to exceed ±70
                servoPulse();
                firstScan = true;
                fbCLR( fbServo);
            }
        }
    }
    else                         //  If Servo is OFF,
    {                            //
        if( rDat1.srvPos != 0)   //  pulse servo until centered
        {                        //
            if ( rDat1.srvPos > 0) --rDat1.srvPos;
            else if ( rDat1.srvPos < 0 ) ++rDat1.srvPos;
            servoPulse();        //
        }                        //
        else servoOffset = 0;    //  zero offset when centered
    }
}

//  = = = = = = = = = =  Servo Test  = = = = = = = = = =
//  Swivel the neck from -90 to +90 degrees and return to center
void bb_servo::test()
{
    rDat1.srvPos = 0;  //  set servo position to center
    servoOffset = 0;
    int pStep = -1;           //  set position step leftwards
    for( int x = 0; x < 360; ++x)
    {
        //  if either limit encountered, flip the position increment sign
        if( rDat1.srvPos <= SERVO_MIN || rDat1.srvPos >= SERVO_MAX) pStep *= -1;
        rDat1.srvPos += pStep;        //  increment position
        servoPulse();
        milliDelay( 10);
    }
    while( rDat1.srvPos != 0)  // pulse servo until centered
    {
        if ( rDat1.srvPos > 0) --rDat1.srvPos;
        else if ( rDat1.srvPos < 0 ) ++rDat1.srvPos;
        servoPulse();
        milliDelay( 10);
    }
    fbCLR( fbServo);       //  set Servo Mode OFF
    //resetServo();  // turn Servo OFF and return to center
}

// This will identify and categorize IR shadows as "targets"
// to determine the auto-directed motion of the platform.
// There will be no more than three targets: shad0, shad1, shad2
//   shad0 - the closest target to the heading of the platform
//   shad1 - the nearest westward target
//   shad2 - the nearest eastward target
// Shadow Structure:
//   distance: less than 250
//   left edge position
//   right edge position
//   size
//   direction
//  For some reason, maybe extra draw from the servo motor,
//  we need to ignore readings that are close to the limit.
//  They seem to go low.  A reason for separate power supply?
void bb_servo::getTarget()
{
    int nPos1 = 30;   // far west
    int nPos2 = 150;  // far east
    int nPos3 = 90;   // center
    // find the westernmost, nearest point
    for( int i = 30; i <= 150; ++i)
      if( srvRay[ i] < srvRay[ nPos1]) nPos1 = i;
    // find the easternmost, nearest point
    for( int i = 150; i >= 30; --i)
      if( srvRay[ i] < srvRay[ nPos2]) nPos2 = i;
    nPos3 = nPos1 - nPos2;             // find width of shadow
    nPos3 = intDiv( nPos3, 2);         // find middle position
    nPos3 = nPos2 + nPos3 - SERVO_MID;  // convert to bearing ±90°
    intConstrain( nPos3, SWEEP_MIN, SWEEP_MAX);      // constrain to ±60°
    rDat1.nearPos = nPos3;             // save as 'near' position
}

void bb_servo::printServoData()
{
    printf( "Pan Position: % 3i |", rDat1.srvPos);     //  A = Azimuth
    printf( "Range: %u |", rDat1.range);
    printf( "Pan Pulse: % 3i", map( rDat1.srvPos,  SERVO_MIN, SERVO_MAX, 70 + servoOffset, 290 + servoOffset));
    printf("\r\n");
}

#endif  //  #if defined(__AVR_ATmega2560__)