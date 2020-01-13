/* File Name: bb_lidar.cpp
 * Developer: Bud Ryerson
 * Described: Setup and control of Lidar sensing device
              as well as interpretation of data from the sensor.
 * Inception: 28MAR19
 *            11APR19 - Cleared all but 9 bytes from serial2 input buffer
 *            before doing Lidar read.  USed to see 8 degree postion offset.
 *            Now data is more timely.
 *            25APR19 - Now includes all servo controls
 *            28OCT19 - Now calls the TFMini-Plus I2C library
 */

#ifdef __AVR_ATmega2560__

    #include <Arduino.h>
    #include <bb_defines.h>
    #include <bb_shared.h>
    #include <bb_routines.h>
    #include <bb_notes.h>

    #include <bb_servo.h>
    bb_servo bbServo;

    #include <bb_lidar.h>
    bb_lidar::bb_lidar(){}
    bb_lidar::~bb_lidar(){}
 
    #include <TFMPI2C.h>     //  Add TFMini-Plus-I2C Library
		                         //  TFMP_DEFAULT_ADDRESS   0x10
    TFMPI2C bbTfmp;          //  Create a TFMini-Plus object

    //  - - - - - -   Lidar Setup   - - - - - - - - - -
    void bb_lidar::setup()
    {
        // Setup TFMini-Plus Lidar sensor
        printf("TFMini-Plus I2C library in use.\r\n");
				
        printf( "Lidar device system reset ");
        if( bbTfmp.sendCommand( SYSTEM_RESET, 0)) printf( "pass.");
            else printf( "fail.");
        printf( ". Status = %2u\r\n", bbTfmp.status);
				
        printf( "Lidar frame rate ");
        if( bbTfmp.sendCommand( SET_FRAME_RATE, 100)) printf( "set to %2uHz", 100);
            else printf( "command failed");
        printf( ". Status = %2u\r\n", bbTfmp.status);

        printf( "Lidar device settings ");
        if( bbTfmp.sendCommand( SAVE_SETTINGS, 0)) printf( "saved");
            else printf( "not saved");
        printf( ". Status = %2u\r\n", bbTfmp.status);

        delay(20);              // And wait a moment
        
        bbServo.setup();  // set Timer4 to function as PWM
                          // set az and el servo output pins
                          // sweep servos for clearance of obstacles
                          // reset servos to center position
    }
    //  - - - - - -   End of Lidar Setup   - - - - - - - - -

    /*  - - - - -   TFMini-Plus Lidar sensor Read  - - - -  */
    //  If no read then set range to zero.
    static uint16_t tfDist, tfFlux, tfTemp;
    void bb_lidar::read()
    {
      // copy pos, dist and flux values from azPos1 to azPos2
      //memmove( rDat1.azPos2, rDat1.azPos, 48);
      // If the status is good (READY), save the data in rDat1
      for( uint8_t i = 0; i < 3; ++i)
      {
        if( bbTfmp.getData( tfDist, tfFlux, tfTemp))
        {
          rDat1.dist = tfDist;
          rDat1.flux = tfFlux;
          rDat3.tmpC = tfTemp; 	// rDat3 is in 'shared.h'
										            // decoded to °C in library
          break;
        }
        else
        { // or else report the status in "distance"
          rDat1.dist = bbTfmp.status;
        }
      }
      // Save the current distance to the azimuth
      // position array in 'srvDat'
      srvDat.distRay[ srvDat.azDex] = rDat1.dist;
      // Save the azimuth index value to rDat1 for transmittal
      rDat1.azPos = srvDat.azDex;
      // If SERVO is TRUE then reposition the servo
      bbServo.sweep();
    }
    /*  - - - - - -   End of Lidar sensor Read  - - - - - -  */


#endif  //  #if defined(__AVR_ATmega2560__)