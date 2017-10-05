/*  File Name: bb_shared.h
 *  Described: Common, global, shared variables for all parts of budbot2
 *  Developer: Bud Ryerson
 *  Inception: 23 NOV 2014
 *  08MAR16 - added program structure
 *  04JUN16 - added isManual flag bit to rDat1
 *            Moved inCount from rDat1 to rDat2
 *  10FEB17 - removed all digital compass code
 */

#ifndef BB_SHARED_H     //  'include guard' macro to prevent errors
#define BB_SHARED_H     //  at compilation time from multiple calls

#include <Arduino.h>    //  It is always important to include this!
#include <bb_defines.h>

//  Declare these only for the robot to save memory
#ifdef __AVR_ATmega2560__

// This will identify and categorize IR shadows as "targets"
// for the auto directed motion of the platform.
  struct shadowStruct {
    int pos1;   // west edge position in degrees: 0 - 180
    int pos2;   // east edge position in degrees: 0 - 180
    int low;    // lowest value
    int high;   // highest value
    int wide;   // difference between pos1 and pos2
    int bear;   // heading minus weighted average position between pos1 and pos2
  }
  extern shad0, shad1, shad2;

/*  // PID Controller data structure
 struct pidStruct {
      float  setPoint     //  1- 4
      float  inPut        //  5- 8
      float  outPut       //  9-12
      float  kp;     // 13-16
      float  ki;     // 17-20
      float  kd;     // 21-24

      float  lastInPut;   // 25-28
      float  sampleTime;  // 29-32
      float  ITerm;       // 33-36
      float  pidOutMin;   // 37-40
      float  pidOutMax;   // 41-44
      float  inPutMin;    // 45-48
      float  inPutMax;    // 49-52

      uint8_t  controlMode;       // 53
      uint8_t  controlDirection;  // 54
      bool     inAuto;            // 55
      uint8_t  kDatSize;  // 56 - size of this structure
                          // should be 56 bytes
  };
  extern pDatMR, pDatML, pDatRC, pDatVC;
*/

  // = = = = = =  float3_s Structure  = = = = = = =
  //  A three dimensional structure of float values
  //  to be used by the Digital Magnetic Compass
  //  (dcStruct) and the IMU (imuStruct)
  struct float3_s {
    union {
      float vRay[ 3]; //  float values as an array
      struct {
        float vX;   //  and as individual float values
        float vY;
        float vZ;
      };
      struct {      //  or as named orientation values
        float Head;
        float Roll;
        float Pitch;
      };
    };
  };
  // - - - -  End of floast3_s Data Structure  - - - - -

  // = = = = = =   Quaternion Data Structure = = = = = =
  //  A four dimensional structure of integer values
  //  to be used by the IMU structure (imuStruct)
  struct integer4_s {
    union {
      int16_t vRay[ 4]; //  integer values as an array
      struct {
        int16_t vW;     //  and as individual values
        int16_t vX;
        int16_t vY;
        int16_t vZ;
      };
    };
  };
  // - - - -  End of Quaternion Data Structure  - - - - -

  //  data structure for 9DoF IMU Motion Shield data
  struct imuStruct {
      float3_s Acc;     // Accelerometer data            12bytes
      float3_s Gyro;    // Gyroscope data                12bytes
      float3_s Mag;     // Magnetometer data             12bytes
      float3_s LinAcc;  // Linear Accelerometer data     12bytes
      float3_s GravAcc; // Gravity Acceleration data     12bytes
      float3_s Euler;   // Euler angle data              12bytes
      integer4_s Quat;      // Quaternion data            8bytes
      uint8_t  AccCalStat;  // Accelerometer Calibration Status: 0 - 3
      uint8_t  GyroCalStat; // Gyroscope Calibration Status: 0 - 3
      uint8_t  MagCalStat;  // Magnetic Compass Calibration Status: 0 - 3
      uint8_t  SysCalStat;  // System Calibration Ststus: 0 - 3
  };
  extern imuStruct imuDat;   //  84bytes total

  //  Servo Structure Declaration for Pan and Tilt Data
  struct srvStruct {
      int sPos;     //  position angle how is this set
      int sDir;     //  direction
      int sPin;     //  pin number
      int sMin;     //  maximum angle
      int sMax;     //  minimum angle
      int sOff;     //  mechanical center offset
      uint16_t sVal[ 90];  // IR sensor values storage
  };
  extern srvStruct srvDat;

  //  Encoder Structure Declaration
  //  for Right and Left Encoder Data
  struct encStruct {
      volatile unsigned long epti;      //  encoder pulse immediate time
      volatile unsigned long epts;      //  encoder pulse saved time
      volatile unsigned long epv;       //  encoder pulse average
      volatile unsigned long epc;       //  count of all encoder pulses since program start
      volatile unsigned long eps;       //  saved pulse count since last T3 interrupt
  //    volatile unsigned long ringTotal; //  total of encoder pulse array
  //	  volatile unsigned long epa[8];    //  encoder pulse array
  //	  volatile byte epx;                //  encoder pulse array index
      volatile bool epf;                //  encoder pulse flag
  };
  extern encStruct eDatR, eDatL;

  //  Motor Structure Declaration
  //  for Right and Left Motor Data
  struct motStruct {
      unsigned long epv;  // Encoder pulse width average
      float epCalc;       // Encoder pulse width calculated
      unsigned long eps;  // Encoder pulse count saved
      int  mSpd;          //  Motor Speed
      int  mSpdTarget;    //  Motor Speed Target
      int  mSpdOffset;    //  Motor Speed Offset
      bool mDir;          //  Motor Direction: true = FORWARD, false = REVERSE
  //    char mChr;          //  Motor Side designation character
  };
  extern motStruct motDatR, motDatL;

#endif  // - - - End of exclusive AVR_ATmega2560 declarations - - -
      //  The reset are declared for both joystick and robot platforms

// = = = =   There are Three Robot Data structures   = = = = =
//  Robot Data structures carry robot data back to the
//  Joystick Transmitter in two parts, rDat1 and rDat2.
//  The platform employs a 32 bytes transmit (TX) buffer
//  for multiplexing, and the joystick uses a 32 byte
//  receive (RX) buffer to demultiplex the transmission.
struct roboStruct1
{
    uint8_t loTime;      // 1 copy of low order byte of bot clock
    uint8_t blank1;      // 2 not used
    uint8_t intCountR;   // 3 T3 period (50ms) wheel encoder interrupt counts
    uint8_t intCountL;   // 4 Saved by bb_motor. 'bb_encoder.pulseAverage();'

    union {
      uint32_t flagBits;  //  long integer
      /* 0 = Motor is ON
         1 = Auto (autonomous navigation) is ON
         2 = Servo is ON
         3 = Radio is ON and linked
         4 = Evade, trying to escape from trapped position
         5 = Halt, no wheel encoder clicks during 40 auto control cycles (2 seconds)
         6 = Pivot, spinning in place to a new heading
         7 = Track, traveling on a set course
         8 = Bear, whenever Heading differs from Course
         9 = Program, running a sequence of steps
        10 = Serial is ON and linked
        11 = Fault
        12 = Stall, when any motor current is over limit
        13 = Object Detection is ON
        14 = Reset Position
        15 = Manual, when joystick is moved out of dead zone
        16, 17, 18, 19 = Obstacle Detect sensors Left & Right Front, Left & Right Rear
        = = =  Robot buttons are declared in Main Program 'bb2_bot.ino'  = = =
        20 = Robot Button 1 - bbCompass.setupIMU() - test and reset IMU
        21 = Robot Button 2 - fbSET( fbProgram) - do Program routine
        22 = Robot Button 3 - fbSET( fbPosReset) - reset X & Y position values to 0
        23 = Robot Button 4 - fbTOG( fbObject) - turn ON/OFF Object Detect flag
      */
      struct {
        uint16_t stateBits;    //  machine state flag bits
        uint8_t  senseBits;    //  object detect and robot button bits
        uint8_t  timerBits;    //  timers and joystick bits
      };
    };

    int velocity;        //  9-10  These two values are read by the
    int rotation;        // 11-12  motor control loop every 50ms.
    int motorSpeedLeft;  // 13-14  Set in bb_motor::control
    int motorSpeedRight; // 15-16      range: -255 to 255
    int  head;          // 17-18 magnetic compass heading: 0 - 359°
    int  course;        // 19-20 course set when auto enabled: 0 - 359°
    int  bear;          // 21-22 difference between course and heading: -180 to +180°

    int  srvPos;        // 23-24 servo pan position: 0 - 180
    int  nearPos;       // 25-26 direction to shortest range value
    uint16_t avgVal;    // 27-28 average IR value from 10 samples
    uint16_t range;     // 29-30 calculated range in centimeters

    uint8_t  mpxID;     // 32 multiplex ID byte = 0x96, 150, 0b10010110
    uint8_t  blank;     // 31
};
extern roboStruct1 rDat1;

struct roboStruct2
{
    union {
      uint32_t botClock;    //  1-4 milliseconds since power on
      struct {
        uint8_t loTime;       //  1 low order byte of bot clock
        uint8_t hiTime[3];    //  2-4 remainder of unsigned long integer
      };
    };
    uint32_t botLoop;       //  5- 8 loops since power on
                            //       (exceeds 64k in 54 minutes)
    int      drPosX;        //  9-10 dead reckoning position X
    int      drPosY;        // 11-12 dead reckoning position Y
    int      drSpdX;        // 13-14 dead reckoning speed X
    int      drSpdY;        // 15-16 dead reckoning speed Y

    int      imAccX;        // 17-18 IMU X-axis linear acceleration
    int      imAccY;        // 19-20 IMU Y-axis linear acceleration
    int      imSpdX;        // 21-22 IMU speed X
    int      imSpdY;        // 23-24 IMU speed Y

    uint8_t  motCurRay[ 4]; // 25-28 Motor current array     4 bytes

    uint8_t  radioTime;     // 29 time in milliseconds to complete radio link
    uint8_t  loopTime;      // 30 time in milliseconds to execute the loop
    uint8_t  mpxID;         // 31 multiplex ID byte = 0x69, 105, 0b01101001
    uint8_t  blank;         // 32
};
extern roboStruct2 rDat2;
//
// - - - - - -    End of Robot Data Declarations   - - - - - - - - //

// ==========  There is one joystick data structures  ============ //
//  Joystick Structure Declaration to carry various joystick value
//  and flag data forward to the Robot Receiver. Joystick routine
//  also computes velocity and rotation values.
//  -------------------------------------------------------------- //
struct joyStruct    // Joystick data structure
{
    union {
      uint32_t joyClock; // 1-4  joystick milliseconds since power on
      struct {
        uint8_t loTime;    //  low order byte of joystick system time
        uint8_t hiTime[3]; //  high order bytes of joystick system time
      };
    };
    uint32_t joyLoop;    // 5-8  number of joystick loops since power up
    uint16_t sigBits;    // 9-10 Unsigned 16 bit value for Joystick signals.
                         // Joystick Buttons are first three bits
    uint16_t tFlags;     // 11-12 Unsigned 16 bit value for timing flags.
    uint16_t jvX;        // 13-14 joystick X: 0-1023 unsigned integer from analog pin A0
    uint16_t jvY;        // 15-16 joystick Y: 0-1023 unsigned integer from analog pin A1
    int velocity;        // 17-18 joystick Velocity: integer -255 to 255
    int rotation;        // 19-20 joystick Rotation: integer -255 to 255, depending on velocity
    
    uint8_t blankRay[4]; // 21-24 - not used currently

    bool radio;          // 25 signal to return rDat1
    bool radio2;         // 26 radio link to platform is ON and good
    bool serial;         // 27 serial link to PC is ON and good
    bool sync;           // 28

    uint8_t strLen;      // 29 signal flags string length
    uint8_t loopTime;    // 30
    uint8_t mpxID;       // 31 multiplex ID byte = 0xCD; (1100 1101)
    uint8_t blank;       // 32
};
extern joyStruct jDat;  // declare the joystick data variable
//
// - - - - - -   End of Joystick Data Declaration  - - - - - - - //

struct radioBufferStruct {
    uint8_t  loTime;     // 1 low order byte of system clock
    uint8_t  blankRay[29];
    uint8_t  mpxID;      // 31 structure ID byte
    uint8_t  blank;      // 32 always blank
};
extern radioBufferStruct rBuf1, rBuf2;

// global flag variables
extern bool centerDrift;    //  nudge servo toward straight and level

//extern int evadeStep;       //  for 'control.cpp'
//extern int programStep;
//extern int saveCourse;    //  used by the evade maneuver in 'control.cpp'
extern int scale;           //  something for the servo

#endif