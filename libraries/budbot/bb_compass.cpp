/* File Name: bb_compass.cpp
 * First Day: 30NOV2015
 * Developer: Bud Ryerson
 * Described: This is code for the Adafruit BNO055 9Axis IMU shield
 *            It will use the two wire I2C protocol to communicate.
 * Last Work: to work with the HMC58X3 libraries
       5DEC15 - use calibration and getOffset routines to determine bias
                and offset respectively.
       7DEC15 - I don't know what I did, but circles are good and the
                compass is working perfectly mounted on the box.  Let's put it
                on the robot and see what happens.
      11DEC15 - Buttons, LEDs and screen prompts interface the selection
                and EEPROM storage of Scale and Offset data.
      18JUN16 - Added a 9DOF IMU motion shield to the Mega2560
      29OCT16 - switched 'heading' to IMU Euler angle.  Sorry to see all this work go,
                but IMU seems to work pretty well. Maybe code can be re-purposed
                to save IMU calibration settings.  Maybe not.
      11DEC16 - moved I2C communication setup to the main platform program
 *    10FEB17 - removed all digital compass code
 *    28JUN17 - corrected and rewrote the dead reckoning formulas
                i.e. should have used 'heading' instead of 'course'
 */

//  This file is defined to compile only for the Mega2560
#ifdef __AVR_ATmega2560__

#include <Arduino.h>         //  It's like a bellybutton
#include <bb_NAxisMotion.h>  //  9DOF IMU Shield Library
#include <EEPROM.h>          //  EEPROM read/write routines

#include <bb_defines.h>
#include <bb_shared.h>    //  this is for Robot Data
#include <bb_routines.h>  //  this is for printFloat()
#include <bb_compass.h>   //  header file for this code

//bb_hmc5883 magn;          //  Instantiate the HMC5883 sensor
bb_NAxisMotion bbImu;     //  Instantiate an Object for the sensor

bb_compass::bb_compass(){}
bb_compass::~bb_compass(){}

//  Set PID Debug in defines
#ifdef PID_CALIBRATE            //  If PID_CALIBRATE set in "defines.h"
    #define NOT_PID_CAL_PRINT   //  do not compile
#else                                  //  Otherwise...
    #define NOT_PID_CAL_PRINT printf   //  compile as 'printf'.
#endif

// ==============================   IMU Initialization   =============================
void bb_compass::setupIMU()
{
    bbImu.initSensor();           // The I2C Address can be changed with this function's parameter.
    bbImu.remapAxisPosition();    // Correct for IMU platform orientation:
                                  // swap X/Y Axis and set X negative
    bbImu.setOperationMode( OPERATION_MODE_NDOF);   // Can be configured to other modes as desired
    bbImu.setUpdateMode( MANUAL);	// Default is AUTO. Setting to MANUAL requires fewer reads
                                  // to the sensor, but requires calling the relevant update
                                  // function prior to calling the read function.
    bbImu.updateAccelConfig();
    bbImu.disableAnyMotion();
    bbImu.disableSlowNoMotion();
}

void bb_compass::testIMU()
{
//    #if( IMU_DEBUG)          //  If in IMU Debug mode
      bbImu.updateAccelConfig();

      printf("IMU accelerometer default settings...\r\n");
      printf("Range: %i\r\n", bbImu.readAccelRange());           // byte: 0 - 3
      printf("Bandwidth: %i\r\n", bbImu.readAccelBandwidth());   // byte 0 - 7
      printf("Power Mode: %i\r\n", bbImu.readAccelPowerMode());  // byte: 0 -5

      printf("IMU streaming data in ");	// Countdown
      printf("3...");
      delay(1000);	       // Wait for a second
      printf("2...");
      delay(1000);         // Wait for a second
      printf("1...\r\n");
      delay(1000);         // Wait for a second
//    #endif
}
// - - - - - - - - - - - - -   End IMU Initialization  - - - - - - - - - - - - - - - -

// Update the BNO055 Calibration Status
// Return an unsigned byte for each of the four calibration registers
// The value will be between 0 (uncalibrated) and 3 (fully calibrated).
// When running in NDOF mode, any data where the system calibration
// value is '0' should generally be ignored.
void bb_compass::getImuCalStatus()
{
    bbImu.updateCalibStatus();     
    imuDat.AccCalStat  = bbImu.readAccelCalibStatus();
    imuDat.GyroCalStat = bbImu.readGyroCalibStatus();
    imuDat.MagCalStat  = bbImu.readMagCalibStatus();
    imuDat.SysCalStat  = bbImu.readSystemCalibStatus();
  //  printImuCalStatus();
}

void bb_compass::printImuCalStatus()
{
    printf("IMU calibration status...\r\n");
    printf("Acc:%1u ", imuDat.AccCalStat);     // Accelerometer Calibration Status (0 - 3)
    printf("Gyro:%1u ", imuDat.GyroCalStat);   // Magnetometer Calibration Status (0 - 3)
    printf("Mag:%1u ", imuDat.MagCalStat);     // Gyroscope Calibration Status (0 - 3)
	  printf("System:%1u", imuDat.SysCalStat);   // System Calibration Status (0 - 3)
    printf("\r\n");
}

// In Manual mode, IMU data requires update before read.
// In NDOF mode, when the system calibration status is '0'
// any data should generally be ignored. 
void bb_compass::getImuValues()
{
    //getImuCalStatus();
/*
    bbImu.updateAccel();                 // Update Accelerometer XYZ data
    imuDat.Acc.vX = bbImu.readAccelX();  // Return float of pure accelerometer
    imuDat.Acc.vY = bbImu.readAccelY();  // data (motion + gravity) in m/s2
    imuDat.Acc.vZ = bbImu.readAccelZ();

    bbImu.updateGyro();                  // Update Gyroscope XYZ data
    imuDat.Gyro.vX = bbImu.readGyroX();  // Return float of gyro data in deg/s
    imuDat.Gyro.vY = bbImu.readGyroY();
    imuDat.Gyro.vZ = bbImu.readGyroZ();

    bbImu.updateMag();                   // Update Magnetic Compass XYZ data
    imuDat.Mag.vX = bbImu.readMagX();    // Return float of magnetometer data in µT
    imuDat.Mag.vY = bbImu.readMagY();
    imuDat.Mag.vZ = bbImu.readMagZ();

    bbImu.updateQuat();                  // Update Quaternion WXYZ data.
    imuDat.Quat.vW = bbImu.readQuatW();  // Return int16_t of Quaternion data
    imuDat.Quat.vX = bbImu.readQuatX();  // times 1000 for 3 decimal place accuracy.
    imuDat.Quat.vY = bbImu.readQuatY();
    imuDat.Quat.vZ = bbImu.readQuatZ();
*/
    bbImu.updateEuler();                          // Update Euler Angle data.
    imuDat.Euler.Head = bbImu.readEulerHeading(); // Return float of Euler Angle data
    imuDat.Euler.Roll = bbImu.readEulerRoll();    // in degrees from 0 to ±180, where + is
    imuDat.Euler.Pitch = bbImu.readEulerPitch();  // head/right, roll/clockwise and pitch/up.

    bbImu.updateLinearAccel();                    // Update Linear Acceleration data.
    imuDat.LinAcc.vX = bbImu.readLinearAccelX();  // Return float of accelerometer data
    imuDat.LinAcc.vY = bbImu.readLinearAccelY();  // without the gravity vector in m/s2.
    imuDat.LinAcc.vZ = bbImu.readLinearAccelZ();
/*
    bbImu.updateGravAccel();                      // Update Gravity Acceleration data.
    imuDat.GravAcc.vX = bbImu.readGravAccelX();   // Return float of accelerometer data
    imuDat.GravAcc.vY = bbImu.readGravAccelY();   // with only the gravity vector in m/s2.
    imuDat.GravAcc.vZ = bbImu.readGravAccelZ();
*/
    //  Save integer value of Euler Heading as 'rDat1.head'
    rDat1.head  =  floatToInt( imuDat.Euler.Head);  // integer of Euler Angle heading
    if( rDat1.head > 359) rDat1.head = 0;           // display 360 degrees as 0 degrees
    
    //  If acceleration exceeds a threshold (0.04 meter/sec/sec)...
    if( abs(imuDat.LinAcc.vX) > 0.04)
    {
      //  multiply by 100 to get centimeters/sec/sec and...
      rDat2.imAccX = floatToInt( imuDat.LinAcc.vX * 100); // cm/s/s
      //  multiply by 10 for meters and by 0.05 second loop time.
      //  Or because 10 x 0.05 = .5, simply divide by 2 to get meters/second
      rDat2.imSpdX += intDiv( rDat2.imAccX, 2);   // meters/second
    }
    else rDat2.imAccX = 0;

    if( abs(imuDat.LinAcc.vY) > 0.04)
    {
      rDat2.imAccY = floatToInt( imuDat.LinAcc.vY * 100);
      rDat2.imSpdY += intDiv( rDat2.imAccY, 2);
    }
    else rDat2.imAccY = 0;

    //  printImuValues();  // as needed for testing
}

//  Called by above 'getImuValues()' routine.
//  Print floating points to 3 decimal place is from 'routines.cpp'
//  Do not compile if doing PID calibration testing
void bb_compass::printImuValues()
{
  #if( !PID_CALIBRATE)  // --------  DO NOT COMPILE IF ADJUSTING PID  ---------

/*    printf( "Euler Angle:");
    printf( " %4i°H", (int)( imuDat.Euler.Head));
    printf( " %4i°R", (int)( imuDat.Euler.Roll));
    printf( " %4i°P", (int)( imuDat.Euler.Pitch));
    printf(" | ");
*/
    printf( "Linear Acc:");
    printFloat( imuDat.LinAcc.vX, 3);      // Linear Acceleration X-Axis data
    printf( "X: ");
    printFloat( imuDat.LinAcc.vY, 3);       // Linear Acceleration Y-Axis data
    printf( "Y: ");
    printFloat( imuDat.LinAcc.vZ, 3);      // Linear Acceleration Z-Axis data
    printf( "Z m/s2");
    printf(" | ");

/*  printf( "Grav Acc:");
    printFloat( imuDat.GravAcc.vX, 3);      // Gravity Acceleration X-Axis data
    printf( "X ");
    printFloat( imuDat.GravAcc.vY, 3);      // Gravity Acceleration Y-Axis data
    printf( "Y ");
    printFloat( imuDat.GravAcc.vZ, 3);      // Gravity Acceleration Z-Axis data
    printf( "Z m/s2 ");
    printf(" | ");
*/
    // Accelerometer Calibration Status returns byte: 0 - 3
    printf("ACS: %i ", imuDat.AccCalStat);

/*  printf( "Course: ");
    printf( " %4i° | ", rDat1.course);
    printf( "MotSpdLft: ");
    printf( " %4i | ", rDat1.motorSpeedLeft);
    printf( "IntCntLft: ");
    printf( " %3u | ", rDat1.intCountL);
//    printf( "drDist: ");
//    printf( " %3i | ", rDat2.drDist);
    printf( " | drPosX: ");
    printf( " %3i | ", imuDat.drPosX);
    printf( " | drPosY: ");
    printf( " %3i | ", imuDat.drPosY);
*/
    printf( " | imSpdX: ");                   // IMU speed in meters/second
    printf( " %3i | ", rDat2.imSpdX);
    printf( " | imSpdY: ");
    printf( " %3i | ", rDat2.imSpdY);

    printf( "\r\n");
    #endif    // ---------  End of DO NOT COMPILE IF ADJUSTING PID   -------
}

void bb_compass::printDeadReckValues()
{
    printf( "Compass: ");
    printf( "enc% 7luL % 7luR", eDatL.epc, eDatR.epc);        //  total Left encoder click count
    printf(" | ");
    printf("dist");
    printFloat( drDist, 2);   // dead reckoning distance in millimeters
    printf(" | ");
    printf( "head% 4i", rDat1.head);  // heading in degrees 0 - 359
    printf(" | ");
    printf( "delta% 5iX % 5iY ", deltaPosX, deltaPosY);      // change in position in mm
    printf(" | ");
    printf( "pos% 5iX % 5iY", imuDat.drPosX, imuDat.drPosY);   // dead reckoning position in mm
    printf( "\r\n");
}

/*  Moved Dead Reckoning navigation math to the PC - 16MAR2019

//  =======  Find new  X & Y Position by Dead Reckoning ======
//  Find new distance using odometry from wheel encoders data
//  Plot XY position using 'heading' angle
//  30JUN16 - Bud Ryerson
//  29JUN17 - update and rework
//  16MAR19 - Why do navigation on platform?
*/
void bb_compass::getPosition()
{
    static int clkL;       // wheel encoder clicks
    static int clkR;
    clkL = rDat1.intCountL; // wheel encoder clicks
    clkR = rDat1.intCountR;
    // If either wheel is stopped then robot is spinning
    // and distance is zero
    if( ( clkL == 0) || ( clkR == 0))
    {
        drDist = 0;
        deltaPosX = 0;
        deltaPosY = 0;
    }
    else     // Otherwise ...
    {
      //  reverse the sign if either wheel is going backwards
      if( rDat1.motorSpeedLeft < 0) clkL *= -1;
      if( rDat1.motorSpeedRight < 0) clkR *= -1;

      //  Add the clicks, divide by 2 to get an average, then multiply
      //  by 0.57 to get distance in millimeters for each 50ms loop
      //  Formula: 166.66 encoder state changes per wheel rotation
      //  divided by 190mm (7.5inches) traveled per wheel rotation
      //  equals 1.14mm of travel per each encoder click.
      //
      //drDist = ( clkL + clkR) / 3.46; // old calculation - why?
      drDist = ( clkL + clkR) * 0.285;
      // direction platform is traveling (heading) in radians
      radHead = imuDat.Euler.Head / 57.2957795;  // 180/Pi
      // Use trig to roughly get the X and Y components of the dead
      // reckoning distance traveled from the angle of the heading.
      //   sin( angle) * hypotenuse = opposite
      //   cos( angle) * hypotenuse = adjacent
      deltaPosX = floatToInt( sin( radHead) * drDist);  // in millimeters?
      deltaPosY = floatToInt( cos( radHead) * drDist);
    }

    if( fbCHK( fbPosReset))  // Robot repositioned to origin point
    {
        setupIMU();          // reinitialize IMU module
        imuDat.drPosX = 0;    // clear dead reckoning
        imuDat.drPosY = 0;    // position
        imuDat.drSpdX = 0;    // and speed
        imuDat.drSpdY = 0;
        rDat2.imAccX = 0;    // clear IMU
        rDat2.imAccY = 0;    // acceleration
        rDat2.imSpdX = 0;    // and speed
        rDat2.imSpdY = 0;
        fbCLR( fbPosReset);  // clear Position Reset flag
    }
    else
    {
      // Find new dead reckoning (dr) distance from point
      // of origin ( 0, 0) in millimeters plus or minus.
      // Map of room is -2250 millimeters (West) to 2250 mm (East),
      // and -3350 millimeters (South) to 3350 mm (North).
      imuDat.drPosX += deltaPosX;
      imuDat.drPosY += deltaPosY;

      // And multiply by 20 to get plus or minus speed in mm/sec
      imuDat.drSpdX = deltaPosX * 20;    //  mm/sec
      imuDat.drSpdY = deltaPosY * 20;    //  mm/sec
    }

    // IMU Data
    // ( Dead Reckoning position and IMU position should equal each other )
    // ------------------------------------------------------------------------
    // | d = ( v * t) + ( ( a * pow( t, 2) / 2);                              |
    // | d = distance, v = initial velocity, t = time, a = acceleration       |
    // | f = v + ( a * t)                                                     |
    // | f = final velocity, v = initial velocity, a = acceleration, t = time |
    // ------------------------------------------------------------------------
    // Is this a good idea?  It does not account for skidding
    if( drDist == 0)         // if robot is spinning or not moving
    {
        rDat2.imAccX = 0;    // clear the acceleration
        rDat2.imAccY = 0;
        rDat2.imSpdX = 0;    // clear the speed
        rDat2.imSpdY = 0;
    }
    else
    {
        //  If acceleration exceeds a threshold (0.04 meter/sec/sec),
        //  then increment speed.
        if( abs(imuDat.LinAcc.vX) < 0.04) rDat2.imAccX = 0;
        else
        {
          //  multiply by 100 to get centimeters/sec/sec and
          rDat2.imAccX = floatToInt( imuDat.LinAcc.vX * 100); // cm/s/s
          //  multiply by 10 for meters and by 0.05 second loop time
          //  Or because 10 x 0.05 = .5, simply divide by 2 to get meters/second
          rDat2.imSpdX += intDiv( rDat2.imAccX, 2);   // meters/second
        }
        if( abs(imuDat.LinAcc.vY) < 0.04) rDat2.imAccY = 0;
        else
        {
          rDat2.imAccY = floatToInt( imuDat.LinAcc.vY * 100);
          rDat2.imSpdY += intDiv( rDat2.imAccY, 2);
        }
    }
    //  Save integer value of Euler Heading as 'rDat1.head'
    rDat1.head  =  floatToInt( imuDat.Euler.Head);  // integer of Euler Angle heading
    if( rDat1.head > 359) rDat1.head = 0;           // display 360 degrees as 0 degrees

    //printDeadReckValues();
}
// - - - - -   End of Navigation calculations   - - - -  //

//  - - - - - - - - -  End of DEAD RECKONING  - - - - - - - - -

#endif  // End of #ifdef __AVR_ATmega2560__

