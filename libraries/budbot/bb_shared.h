/*  File Name: bb_shared.h
 *  Described: Common, global, shared variables for all parts of budbot2
 *  Developer: Bud Ryerson
 *  Inception: 23 NOV 2014
 *  08MAR16 - added program structure
 *  04JUN16 - added isManual flag bit to rDat1
 *            Moved inCount from rDat1 to rDat2
 *  10FEB17 - removed all digital compass code
 *  20MAR19 - added rDat3 to take vel and rot values and free
 *            up space in rDat1 to pass Lidar flux values
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

      int   drPosX;    // Dead reckoning position in millimeters
      int   drPosY;
      int   drSpdX;    // Dead reckoning speed in mm/sec
      int   drSpdY;
  };
  extern imuStruct imuDat;   //  84 bytes total

  //  Servo Structure Declaration for Pan and Tilt Data
  struct srvStruct {
      int azDex;     //  azimuth position index value
      int azDir;     //  azimuth sweep direction
      int azStep;
      int azPin;     //  azimuth servo pin number
      int azMin;     //  azimuth minimum angle
      int azMax;     //  aziumuth maximum angle
      uint16_t distRay[ 181];  // array of Lidar distance values
                               // for every azimuth index value
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
        //  The rest are declared for both joystick and robot platforms

// = = = =   There are Three Robot Data structures   = = = = =
//  Robot Data structures carry 64 bytes of data to the Joystick
//  in two parts: rDat1 and rDat2. Each a separate radio operation
//  The platform employs a 32 bytes transmit (TX) buffer
//  for multiplexing, and the joystick uses a 32 byte
//  receive (RX) buffer to demultiplex the transmission.
struct roboStruct1
{
    uint8_t loTime;      // 1 copy of low order byte of bot clock
                         // Another 'loTime' at head of R2 Identifies the two 32 byte
                         // packets as parts of the same 64 byte packet transmission
    uint8_t blank1;      // 2 not used
    uint8_t intCountR;   // 3 Actual motor speed in wheel encoder interrupts per T3
    uint8_t intCountL;   // 4 (50ms) interval. Counted in 'encoder'. Saved in 'motor'.

    union {
      uint32_t flagBits;  // 5-8 long integer
      /* 0 = Motor is ON
         1 = Auto (autonomous navigation) is ON
         2 = Servo is ON
         3 = Radio is ON and linked
         4 = Serial is ON and linked
         5 = Evade, trying to escape from trapped position
         6 = Halt, no wheel encoder clicks during 40 auto control cycles (2 seconds)
         7 = Pivot, spinning in place to a new heading
         8 = Track, traveling on a set course
         9 = Bear, whenever Heading differs from Course
        10 = Program, running a sequence of steps
        11 = Fault
        12 = Seek, sweep servo one time to acquire target
        13 = Object Detection is ON
        14 = Reset Position
        15 = Manual, when joystick is moved out of dead zone
        16, 17, 18, 19 = Obstacle Detect sensors Left & Right Front, Left & Right Rear
        = = =  Robot button operation is in 'alarm.cpp'  = = =
        20 = Robot Button 1 - fbTOG( fbServo);    // Toggle 'Sweep' routine in 'servo.cpp'
        21 = Robot Button 2 - fbSET( fbProgram)   // Do 'Program' routine in 'control.cpp'
        22 = Robot Button 3 - fbSET( fbPosReset)  // Reset IMU and zeroe position values as
                                                  // part of 'getPosition' in 'compass.cpp'
        23 = Robot Button 4 - fbTOG( fbObject)    // Enable 'testObjectDetect' in 'alarm.cpp'
                                                  // called by 'control' in 'control.cpp'
      */
      // Divide the flagBits into separate categories
      struct {                 // 5-8
        uint16_t stateBits;    //  machine state flag bits
        uint8_t  senseBits;    //  object detect and robot button bits
        uint8_t  timerBits;    //  timers and joystick bits
      };
    };

    int  azPos;         //  9-10 azimuth servo position 1: -90° to +90°
    uint16_t dist;      // 11-12 Lidar reported distance in centimeters    
    uint16_t flux;      // 13-14 Lidar reported intensity    
    int  azPos2;        // 15-16 second azimuth servo position
    uint16_t dist2;     // 17-18 second Lidar reported distance
    uint16_t flux2;     // 19-20 second Lidar reported intensity

    int motorSpeedLeft;  // 21-22  Commanded speed as calculated in bb_motor::control
    int motorSpeedRight; // 23-24  by the PID controller. Range: -255 to 255
    int  head;           // 25-26  magnetic compass heading: 0 - 359°
    int  course;         // 27-28  course set when auto enabled: 0 - 359°
    int  bear;           // 29-30  difference between course and heading: -180 to +180°
    uint16_t  mpxID;     // 31-32  8 bit multiplex ID byte followed by zeros 
};
extern roboStruct1 rDat1;

struct roboStruct2
{
    union {                 //  Platform system time
      uint32_t botClock;    //  1-4 milliseconds since power on
      struct {
        uint8_t loTime;     //  1 low order byte of the clock
        uint8_t hiTime[3];  //  2-4 the rest of the number
      };
    };
    uint32_t botLoop;       //  5- 8 loops since power on,
                            //       will exceed 64k in 54 min.
                            
    //  From the 'compass.cpp' routines
    int      drPosX;        //  9-10 dead reckoning position X in millimeters
    int      drPosY;        // 11-12 dead reckoning position Y
    int      drSpdX;        // 13-14 dead reckoning speed X in mm/sec
    int      drSpdY;        // 15-16 dead reckoning speed Y in mm/sec

    int      imAccX;        // 17-18 IMU X-axis linear acceleration in cm/s/s
    int      imAccY;        // 19-20 IMU Y-axis linear acceleration
    int      imSpdX;        // 21-22 IMU speed X in meters/second
    int      imSpdY;        // 23-24 IMU speed Y

    union {                 // To send 4 motor currents as single word
      uint8_t  mcRay[ 4];   // 25-28 Motor current as an array of unsigned bytes
      uint32_t mcInt;       // 25-28 Same array as 32 bit unsigned integer
    };

    uint8_t  radioTime;     // 29 milliseconds to complete radio link
    uint8_t  loopTime;      // 30 milliseconds to execute each loop
    uint16_t  mpxID;        // 31-32 multiplex ID byte followed by zero byte
};
extern roboStruct2 rDat2;

// System-wide data that does not get transmitted to the controller
struct roboStruct3
{
    int velocity;       // 0-1  vel and rot values are received from jDat
    int rotation;       // 2-3  and read by motor control every 50ms.
    int nearDex;        // 4-5 lowest distance array index value
    int farDex;         // 6-7 highest distance array index value
    int tmpC;           // 8-9 Lidar temperature in degrees Celsius
    uint16_t distRay[ 181];  // array of Lidar distance values for each servo position
    uint16_t fluxRay[ 181];  // array of Lidar strength values for each servo position
	  //uint32_t gpTimer;     // general purpose timer
    uint32_t seekTimer;   // 5 sec timer between SEEK intervals
    uint32_t haltTimer;   // 5 sec timer between SEEK intervals
};
extern roboStruct3 rDat3;
//
// - - - - - -    End of Robot Data Declarations   - - - - - - - - //

// ==========  There is one joystick data structures  ============ //
//  Joystick Structure Declaration to carry various joystick value
//  and flag data forward to the Robot Receiver. Joystick routine
//  also computes velocity and rotation values.
//  -------------------------------------------------------------- //
struct joyStruct {   // Joystick data structure
    union {
      uint32_t joyClock;   // 1-4  joystick milliseconds since power on
      struct {
        uint8_t loTime;    //  low order byte of joystick system time
        uint8_t hiTime[3]; //  remaining bytes of joystick system time
      };
    };
    uint32_t joyLoop;    // 5-8  number of joystick loops since power up
    uint16_t sigBits;    // 9-10 Unsigned 16 bit value for Joystick signals.
    /* 0 = Motor is ON   // Joystick Buttons are first three bits
       1 = Auto (autonomous navigation) is ON
       2 = Servo is ON
       3 = Radio is ON and linked
       4 = Serial is ON and linked
       5 = Evade, trying to escape from trapped position
       6 = Halt, no wheel encoder clicks during 40 auto control cycles (2 seconds)
       7 = Pivot, spinning in place to a new heading
       8 = Track, traveling on a set course
       9 = Bear, whenever Heading differs from Course
      10 = Program, running a sequence of steps
      11 = Fault
      12 = Seek, sweep servo one time to acquire target
      13 = Object Detection is ON
      14 = Reset Position
      15 = Manual, when joystick is moved out of dead zone                         
    */
    uint16_t tFlags;     // 11-12 Unsigned 16 bit value for timing flags.
    uint16_t jvX;        // 13-14 joystick X: 0-1023 unsigned integer from analog pin A0
    uint16_t jvY;        // 15-16 joystick Y: 0-1023 unsigned integer from analog pin A1
    int velocity;        // 17-18 joystick Velocity: integer -255 to 255
    int rotation;        // 19-20 joystick Rotation: integer -255 to 255, depending on velocity

    uint16_t rIrqCount;   // 21-22 radio IRQ counter
    uint16_t loopTime;    // 23-24

    bool radioIRQ;       // 25 radio IRQ has occurred
    bool radio1;         // 26 rDat1 is received
    bool radio2;         // 27 rDat2 is received
    bool sync;           // 28 entire rDat packet is received

    uint8_t  strLen;     // 29 signal flags string length
    bool     serial;     // 30 serial link to PC is ON and good
    uint16_t  mpxID;     // 31-32 multiplex ID byte followed by zero byte
};
extern joyStruct jDat;  // declare the joystick data variable
//
// - - - - - -   End of Joystick Data Declaration  - - - - - - - //

struct radioBufferStruct {
    uint8_t  loTime;        //  1 low order byte of system clock
    uint8_t  blankRay[29];  //  2-30 not assigned
    uint16_t mpxID;         // 31-32 multiplex ID byte
};
extern radioBufferStruct rBuff;

#endif