// File Name: bbPane05.pde
// Creator:   Bud Ryerson
// Inception: 20JAN2017
// LastTouch: 31DEC2019
// Described:
//   Read serial platform data from joystick at 115200 baud.
//   Parse data into integer format.
//   Draw various screens and display data
//   Accept mouse click input over button pads
//   Write serial button data back to joystick

import processing.serial.*;
import java.nio.*;
import java.util.Arrays;

Serial myPort;         // declare a serial port

PFont  faceMSR14, faceMSR20, faceMSR28; // Monospace821BT-Roman
PFont  faceBPB18, faceBPB56;            // BlenderPro-Book

int serialNumber;
static final int serialRate = 115200;
String serialName;
boolean portError = false;  //  error opening designated serial port
boolean firstDraw = true;   //  first timwe through Darw loop

void setup()
{
    size( 1640, 640);   // Size of the display area
    frameRate( 60);     // default frame rate
    // load pre-created fonts from the 'data' directory
    faceMSR14 = loadFont( "Monospace821BT-Roman-14.vlw");
    faceBPB18 = loadFont( "BlenderPro-Book-18.vlw");
    faceMSR20 = loadFont( "Monospace821BT-Roman-20.vlw");
    faceMSR28 = loadFont( "Monospace821BT-Roman-28.vlw");
    faceBPB56 = loadFont( "BlenderPro-Book-56.vlw");

    // CENTER (default) =  xpos, ypos, width, height
    ellipseMode( CENTER);

    // set the output flag char array to all '0's
    // reset to '0' after every putData cycle
    for( int x = 0; x < 16; ++x) outFlags[ x] = '0';

    setupRadar();

    // = = = = = =  setup serial port  = = = = = = =
    // return 2 less than the number of serial ports avaiable
//  20191231
//  This is kinda nuts and should be changed.
//  Happy New Year!
    serialNumber = Serial.list().length - 2;
    if( serialNumber >= 0)
    {
      // List all the available serial ports
      println( "List of available ports: ", Serial.list());

      // get the name of the port
      serialName = Serial.list()[ serialNumber];
      println( "Selected Port Number: ", serialNumber);
      println( "Selected Port Name: ", serialName);

      try
      {
        // Open COM Port 25, which is presently #2 on the list
        myPort = new Serial( this, serialName, 115200);
      }
      catch (RuntimeException e)
      {
        portError = true;
      }
      if( !portError) myPort.clear();
    }
    else
    {
      portError = true;
    }
    if( portError) println( "An error occurred. Port may not be available.");

    firstDraw = true;   //  first time through Draw loop
}

void draw()
{
    if( firstDraw)              // set display location of sketch window
    {
        surface.setLocation(240, 150);
        firstDraw = false;
    }
    background( ccBgd);
    drawButtonBar( 20, 20);     // box indicators for flags
    getData();                  // get Platfrom dtat from joystick
    if( putFlag) putData();     // only send signal string after good data read

    getPosition(); // *** here temporarily from getData()  *****

    drawMAPscreen( 1020, 70);   // map area has large border and must draw first
    drawJOYscreen( 20, 70);     // joystick position and values
    drawRotationScreen( 20, 365);
    drawCOMscreen( 20, 425);
    // drawCopyright( 20, 590);
    // drawRdat1( 20, 590);
    //drawSignalString( 20, 590);    // in 'drawJOY'
    drawCurrent( 20, 590);
    // testByte2Short( 20, 590);     // in 'drawJOY'
    drawMotScreen( 320, 70);    // motion thermometer instruments
    drawNAVscreen( 520, 70);    // overwrite MAP borders
}

/*
  if( contact)
  {
  }
  else  //  loop while looking for handshake from joystick
  {
      calling();   //  this is in the getData routines
      print( "Calling...");
      print( callCounter);
      print( " ");
  }
*/

// = = = = = =  Class Definitions to support rDat structures  = = = = = =
// Even though there are multiple objects, we still only need one class.
// No matter how many cookies we make, only one cookie cutter is needed.
// size of both rDats should be 32 bytes
public class rDat1
{
    //  type byte  = signed 8 bit number
    byte loTime;      // 1 copy of low order byte of bot clock
                         // Another 'loTime' at head of R2 Identifies the two 32 byte
                         // packets as parts of the same 64 byte packet transmission
    byte blank1;      // 2 not used
    byte intCountR;   // 3 T3 period (50ms) wheel encoder interrupt counts
    byte intCountL;   // 4 Saved by bb_motor. 'bb_encoder.pulseAverage();'

    //  type int   = signed 32 bit number
    int  flagBits;    //  5-8 integer consisting of 32 (4x8) flag bits
      /* 0 = Motor ON(1)/OFF(0)
         1 = Auto (autonomous navigation) ON(1)/OFF(0)
         2 = Servo ON(1)/OFF(0)
         3 = Radio is ON and linked (1) or OFF(0)
         4 = Evade, trying to escape from trapped position
         5 = Halt, no wheel encoder clicks during 40 auto control cycles (2 seconds)
         6 = Pivot, spinning in place to a new heading
         7 = Track, traveling on a set course
         8 = Bear, whenever Heading differs from Course
         9 = Program, running a sequence of steps
        10 = Serial is ON(1) and linked or OFF(0)
        11 = Fault
        12 = Stall, when any motor current is over limit
        13 = Object Detection ON(1)/OFF(0)
        14 = Reset Position
        15 = Manual, when joystick is moved out of dead zone
        = = =  Obstacle Detect sensors  = = =
        16, 17, 18, 19 =  Left & Right Front, Left & Right Rear
        = = =  Robot platform mechanical buttons   = = =
        20 = Robot Button 1 - bbCompass.setupIMU() - test and reset IMU
        21 = Robot Button 2 - fbSET( fbProgram) - do Program routine
        22 = Robot Button 3 - fbSET( fbPosReset) - reset X & Y position values to 0
        23 = Robot Button 4 - fbTOG( fbObject) - turn ON/OFF Object Detect flag
      */

    //  type short = sigend 16 bit number
    short azPos;     //  9-10 azimuth servo position: -90 to +90 degrees
    short nearPos;   // 11-12 direction to nearest object: -90 to +90
    short dist;      // 13-14 Lidar distance value in centimeters
    short flux;      // 15-16 Lidar signal return intensity
    short tmpF;      // 17-18 Lidar temperature in degrees Farenheit

    short blank2;     // 19-20  not used
    short motorSpeedLeft;  // 21-22  Set in bb_motor::control
    short motorSpeedRight; // 23-24      dist: -255 to 255
    short head;            // 25-26 magnetic compass heading: 0 - 359°
    short course;          // 27-28 course set when auto enabled: 0 - 359°
    short bear;            // 29-30 difference between course and heading: -180 to +180°

    byte  mpxID;     // 31 multiplex ID byte to identify the packet
                     // Robot data packet 1, R1 = 0x96, 150, 0b10010110
    byte  zeroID;    // 32 part of ID, must always be zero
}

class rDat2
{
    //  type int   = signed 32 bit number
    int botClock;   //  1-4 milliseconds since power on
    int botLoop;    //  5- 8 loops since power on
                     //  (exceeds 64k in 54 minutes)
    //  type short = sigend 16 bit number
    short  drPosX;     //  9-10 dead reckoning position X
    short  drPosY;     // 11-12 dead reckoning position Y
    short  drSpdX;     // 13-14 dead reckoning speed X
    short  drSpdY;     // 15-16 dead reckoning speed Y

    short  imAccX;     // 17-18 IMU X-axis linear acceleration
    short  imAccY;     // 19-20 IMU Y-axis linear acceleration
    short  imSpdX;     // 21-22 IMU speed X
    short  imSpdY;     // 23-24 IMU speed Y

    //  type byte  = signed 8 bit number
    byte[]  motCurRay = new byte[4]; // 25-28 Four byte Motor current array

    byte  radioTime;  // 29 time in milliseconds to complete radio link
    byte  loopTime;   // 30 time in milliseconds to execute the loop
    byte  mpxID;      // 31 multiplex ID byte = 0x69, 105, 0b01101001
    byte  zeroID;     // 32 must always be zero
}

/*  - - - - - - - -  - - - - - - - - - - - - - - - - */
