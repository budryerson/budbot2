/*  File Name: bb_defines.h
 *  Developer: BudRyerson
 *  Inception: 14NOV14
 *  Described: Definitions and macros to be inserted at compile time
 *  Last work:
 *    04JUL15 - radio available timeout value
 *    05JUL15 - changed servo range from -90-90 to 0-180
 *    03AUG15 - added LED mode pins for the Mega
 *    04OCT15 - switch encoder output to 'quadrature' doubling number of pulses
 *    10FEB17 - added blue LED to joystick and changed Calibrate flagBit to Serial
 */

//  ALT 227 = π  (Pi)
//  ALT 241 = ±  (plus/minus)
//  ALT 167 = º  (degrees)
//  ALT 248 = º  (degrees)

//  If the expression you write (after the #if) has a nonzero value,
//  then the line group immediately following the #if directive
//  is retained in the translation unit.  (0) = False

// ==========    Compile time conditional definitions    ==============

// - - - - - - -  SWITCHES  - - - - - - - - -
  // Compile time conditional definitions, such as:
  //  For any PID calibration PID_CALIBRATE must be set to (1) to enable
  //  PID serial data and to suppress any extraneous serial printing
  #define PID_CALIBRATE (0) //  enable code of PID serial link to Processing display
  #define PID_MOTOR_RIGHT_CALIBRATE (0) //  enable the RX/TX code for each particular Controller
  #define PID_MOTOR_LEFT_CALIBRATE  (0) //  PID Calibrate Motor Left
  #define PID_ROTATION_CALIBRATE  (0)   //  be sure to turn off Velocity
  #define PID_VELOCITY_CALIBRATE  (0)   //  be sure to turn off Halt
  #define PID_PAN_CALIBRATE       (0)
  #define PID_TILT_CALIBRATE      (0)
  #define IMU_DEBUG (0)     //  enable print IMU Info
  #define RADIO_DEBUG (0)   //  enable print Radio Info
  //#define EYE_CALIBRATE (0) //  enable IReye calibration
// - - - - - - -    End of SWITCHES  - - - - - - - - -


// - - - - - - -  MACROS - - - - - - - - -
  // Some useful macros for bit manipulation
  #define SET( x, y) ( x = x | ( 1 << y))      // set y bit of x
  #define CLR( x, y) ( x = x & ( ~( 1 << y)))  // reset y bit of x
  #define CHK( x, y) ( x & ( 1 << y))          // test y bit of x
  #define TOG( x, y) ( x = x ^ ( 1 << y))      // flip y bit of x
  // Set bit Z of X if Y is HIGH, or clear bit Z if Y is LOW
  #define BUT( x, y, z) ( x ^= ( -y ^ x) & ( 1 << z))

  //#define DBLCHK( x, y) ( ( x >> y) & 1)  // test y bit of long x
  #define LOWORD(a) ((uint16_t)(a))  //  lo order word of 32 bit double word
  #define HIWORD(a) ((uint16_t)(((uint32_t)(a) >> 16) & 0xFFFF))
  #define LOBYTE(a) ((uint8_t)(a))   //  low order byte of 32 bit double word

  // returns the 32 bit long integer of a floating point number
  #define round(x) ( (x)>=0 ? (long)((x)+0.5):(long)((x)-0.5))

  //  OFFSETOF if not a standard function in Arduino
  //  This macro returns a pointer to a member of a structure
  // #define OFFSETOF(type, field)    ((unsigned long) &(((type *) 0)->field))
// - - - - - - -  End of MACROS  - - - - - - - - -

// - - - - - - -  CONSTANTS  - - - - - - - - -
  // Reset rDat3.seekTimer to SEEK at 7 second intervals
  // when SERVO is TRUE
  #define SEEK_INTERVAL  7000UL
  // Reset rDat3.haltTimer to HALT after 800 milliseconds
  // of minimal motion commands when AUTO is TRUE
  #define HALT_INTERVAL   800
  // 32 bit unsigned integer value of 1 used by FlagBit functions
  #define longOne 0x00000001UL
  //  Dashed line with carriage return/linefeed for printing.
  # define DASHLINE "------------------------------------\r\n"

//  ==============    Data Structure Definitions   ==============
  //  data structure 16 bit mpxID byte followed by zero byte
  # define JDAT_ID  0x00CD  //  jDat.mpxID  = bit pattern: 1100 1101
  # define RDAT1_ID 0x0096  //  rDat1.mpxID = bit pattern: 1001 0110
  # define RDAT2_ID 0x0069  //  rDat2.mpxID = bit pattern: 0110 1001

  //  position number of flagBits
  # define fbMotor    0
  # define fbAuto     1
  # define fbServo    2
  # define fbRadio    3
  # define fbSerial   4
  # define fbEvade    5
  # define fbHalt     6
  # define fbPivot    7
  # define fbTrack    8
  # define fbBear     9
  # define fbProgram  10
  # define fbFault    11
  # define fbSeek     12
  # define fbObject   13
  # define fbPosReset 14
  # define fbManual 15
  # define odFlagLF 16  // object detector flag Left Front
  # define odFlagRF 17  // object detector flag Right Front
  # define odFlagLR 18  // object detector flag Left Rear
  # define odFlagRR 19  // object detector flag Right Rear
  # define fbRoBut1 20
  # define fbRoBut2 21
  # define fbRoBut3 22
  # define fbRoBut4 23
  # define fbRoTim1 24  // three second halt timer?
  # define fbRoTim2 25
  # define fbJoBut1 26
  # define fbJoBut2 27
  # define fbJoBut3 28
  # define fbJoTim1 29
  # define fbJoTim2 30
  # define fbJoTim3 31

//  ==============    Compass Definitions   ==============
//  Define some values of Pi for budbot
//#define bb_Pi    3.1415926536    //  Pi
//#define bb_2Pi   6.2831853072    //  2 x Pi
//#define bb_180ByPi 57.295779513  //  180 / Pi

// Digital Compass Scale data stored at the bottom of EEPROM memory
//#define EEPROM_SCALE_ADDR    (0)
// DC Offset data stored 12 byte (or 3 float) addresses higher
//#define EEPROM_OFFSET_ADDR   (EEPROM_SCALE_ADDR + sizeof( float3_s))
//  IR Eye Calibration data stored 12 addresses above that
//#define EEPROM_EYECAL_ADDR   (EEPROM_OFFSET_ADDR + sizeof( float3_s))
//#define EEPROM_NEXT_ADDR     (EEPROM_EYECAL_ADDR + 8; // size of an array of four, 2 byte integers

//  ==============    Radio Definitions    ==============
#define rxRadioTimeout  200  // millisecond wait for radio available
#define radioFailMax     10  // maximum number of consecutive radio failures

//  ==============    Joystick Definitions    ==============
//  joystick (dead) zone = ±20 joystick units centered on 512
#define jZone    20   //  one half of joystick zone
#define jZmid   512   //  joystick zone center
#define jZmin   ( jZmid - jZone)   //  dead zone lower bound
#define jZmax   ( jZmid + jZone)   //  dead zone upper bound

#define crnrDstMin  200   //  Alarm average from corner sensor
#define MOTOR_CURRENT_HIGH  150   //  Current too high/motor stuck

//  ==============    Encoder Definitions    ==============
#define encWidMin 1838      //  Dual Encoder Width Minimum at 8,500RPM
                            //  shortest pulse (max motor speed) = 1.838 milliseconds
#define encWidMax 470528    //  Dual Encoder Width Maximum (quadrature encoding)
                            //  longest pulse (motor not turning) = 0.470528 seconds
#define clkDist     1.83    //  approximate distance in mm platform travels per encoder click
#define motorSpdMax  255
#define motorSpdMin    0


//  ==============  PIN NUMBER DEFINITIONS  ============================
  //  - - - - - - - -  Robot Motor pin Definitions  - - - - - - - - - -
  //  Motor Controller Board connectors are numbered 0, 1, 2, 3
  //  Do not use digital pins 6,7 & 8 for PWM because the OC4 timer is
  //  used for a custom servo control PWM routine in 'bb_servo.cpp'
  //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //  Motor 0 = Right Front Motor (RFM)
  #define RFMSpin   9   // RFM Speed PWM - digital out D09 - orange
  #define RFMDpin  40   // RFM Direction - digital out D40 - black
  #define mcPinRF  A8   // RF Current pin - analog in  A08 - orange
  //  Motor 1 = Left Front Motor (LFM)
  #define LFMSpin  10   // LFM Speed PWM - digital out D10 - yellow
  #define LFMDpin  41   // LFM Direction - digital out D41 - white
  #define mcPinLF  A9   // LF Current pin - analog in  A09 - yellow
  //  Motor 2 = Right Rear
  #define RRMSpin  11   // RRM Speed PWM - digital out D11 - green
  #define RRMDpin  42   // RRM Direction - digital out D42 - gray
  #define mcPinRR A10   // RR Current pin - analog in  A10 - green
  //  Motor 3 = Left Rear
  #define LRMSpin  12   // LRM Speed PWM - digital out D12 - blue
  #define LRMDpin  43   // LRM Direction - digi tal out D43 - violet
  #define mcPinLR A11   // LR Current pin - analog in  A11 - blue

  //  Robot speaker audio output pin
  #define Speaker 13     // Speaker      - digital out D07

  //  Four corner KY-32 IR Obstacle Avoidance Detectors
  #define odPinLF  26   //  Left Rear  - digital in D26
  #define odePinLF 27   //  Enable
  #define odPinRF  28   //  Right Front - digital in D27
  #define odePinRF 29
  #define odPinLR  30   //  Left Front - digital in D28
  #define odePinLR 31
  #define odPinRR  32   //  Right Rear  - digital in D29
  #define odePinRR 33   //  Enable

  #define RCLpin   48   //  pin number of LED indicator of Radio Control Link

//  -  Some pin definitions are different between Joystick and Robot
#ifdef __AVR_ATmega2560__
  //  - - - - -  Mega2560 (Robot) Mode LED pin definitions  - - - - -
  // pins 22-25 are connected by a four conductor cable to the large breadboard
  #define rBluPin 22   // Robot Blue LED pin   - digi out D22 - white
  #define rRedPin 23   // Robot Red LED pin    - digi out D23 - black
  #define rYlwPin 24   // Robot Yellow LED pin - digi out D24 - brown
  #define rGrnPin 25   // Robot Green LED pin  - digi out D25 - red
  #define rOrgPin 44   // Robot Orange LED pin - digi out D44 - not used

  #define rButPin1 34  //  Robot Button 1 - white
  #define rButPin2 35  //  Robot Button 2 - black
  #define rButPin3 36  //  Robot Button 3 - yellow
  #define rButPin4 37  //  Robot Button 4 - green
  #define rButPin5 38  //  Robot Button 5 - blue (not used)
  #define rButPin6 39  //  Robot Button 6 - (not used)

  // - - - - -  Mega2560 hardware interrupt pin definitions  - - - - -
  #define INT0pin   2   // interrupt 0 = digital pin  2 - green wire to LFMotor
  #define INT1pin   3   // interrupt 1 = digital pin  3 - blue wire to RFMotor
  #define INT2pin  21   // interrupt 2 = digital pin 21
  #define INT3pin  20   // interrupt 3 = digital pin 20
  #define INT4pin  19   // interrupt 4 = digital pin 19
  #define INT5pin  18   // interrupt 5 = digital pin 18
#else
  //  - - -  ProMini (Joystick) Mode LED pin definitions  - - -
  //#define RMLpin  4    // Robot Mode LED - Green  - pin 4
  //#define SMLpin  5    // Servo Mode LED - Yellow - pin 5
  //#define MMLpin  6    // Motor Mode LED -  Red  -  pin 6
  #define jGrnPin  4   // Joystick Green LED Auto
  #define jYlwPin  5   // Joystick Yellow LED Servo
  #define jRedPin  6   // Joystick Red LED Motor
  #define jBluPin 18   // Joystick Blue LED Pane Comm

  //  - - - -  ProMini (Joystick) IO pin definitions  - - - -
  #define joyPin1 A0   // Joystick Pan  - analog in 0 - orange
  #define joyPin2 A1   // Joystick Tilt - analog in 1 - blue
  #define jRadInt   2  // Joystick Radio Interrupt
  #define jBut1Pin  3  // Joystick Button 1 pin - red
  #define jBut2Pin  7  // Joystick Button 2 pin - violet
  #define jBut3Pin  8  // Joystick Button 3 pin - green
#endif

/*  SPI pin numbers are unique to certain Arduino types.
    MOSI & MISO are in reverse order for Mega and ProMini

    RF24  Color   ProMini   Mega
    --------------------------------------
    IRQ   Orange     2       -
    CE    Gray       9      49
    CSN   Violet    10      53
    MOSI  Green     11      51
    MISO  Yellow    12      50
    SCK   Blue      13      52

    CE (Chip Enable) & CSN (Chip Select) pins must be defined
    in software specifically for the Mega and for the ProMini
 */
 
// radio pins
#ifdef __AVR_ATmega2560__
  #define CEpin  49  // gray
  #define CSNpin 53  // violet
#else
  #define CEpin   9
  #define CSNpin 10
#endif
