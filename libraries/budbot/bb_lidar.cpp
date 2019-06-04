/* File Name: bb_lidar.cpp
 * Developer: Bud Ryerson
 * Described: Setup and control of Lidar sensing device
              as well as interpretation of data from the sensor.
 * Inception: 28MAR19
 *            11APR19 - Cleared all but 9 bytes from serial2 input buffer
 *            before doing Lidar read.  USed to see 8 degree postion offset.
 *            Now data is more timely.
 *            25APR19 - Now includes all servo controls
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
 
    #include <TFMPlus.h>     //  Add the TFMini Plus Library
    TFMPlus bbTfmp;          //  Create a TFMini-Plus object

    //  - - - - - -   Lidar Setup   - - - - - - - - - -
    void bb_lidar::setup()
    {
        //  setup TFMini-Plus Lidar sensor
        Serial2.begin( 115200);  // Initialize device serial port.
        delay(20);               // Give port time to initalize

        // Initialize lidar device object and pass serial port.
        // Display message whether Lidar device is available
        printf("Lidar device serial port \"Serial2\" ");
        if( bbTfmp.begin( &Serial2)) printf( "initalized.");
          else printf( "not available.");
        printf( "\r\n");

        //factoryReset();
        frameRate( 100);
        //saveSettings();
        delay(20);              // And wait a moment
        
        bbServo.setup();  // set Timer4 to function as PWM
                          // set az and el servo output pins
                          // sweep servos for clearance of obstacles
                          // reset servos to center position
    }
    //  - - - - - -   End of Lidar Setup   - - - - - - - - -

    /*  - - -   Several useful Lidar commands    - - - -   */
    void bb_lidar::factoryReset()
    {
        printf( "Lidar device system reset ");
        if( bbTfmp.sendCommand( RESTORE_FACTORY_SETTINGS, 0))
        {
            printf( "pass.");
        }
        else printf( "fail.");
        printf( ". Status = %2u", bbTfmp.status);
        printf( "\r\n");
    }

    void bb_lidar::frameRate( uint16_t rate)
    {
        printf( "Lidar device frame rate ");
        if( bbTfmp.sendCommand( SET_FRAME_RATE, rate))
        {
            printf( "set to %2uHz", rate);
        }
        else printf( "command failed");
        printf( ". Status = %2u", bbTfmp.status);
        printf("\r\n");
    }

    void bb_lidar::saveSettings()
    {
        printf( "Lidar device settings ");
        if( bbTfmp.sendCommand( SAVE_SETTINGS, 0))
        {
            printf( "saved");
        }
        else printf( "not saved");
        printf( ". Status = %2u", bbTfmp.status);
        printf( "\r\n");
    }
    /*  - - -   End of useful Lidar commands   - - - -   */


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
          // Decode tfTemp value to °F
          rDat3.tmpF = uint16_t( tfTemp * 0.225) - 493;
          break;
        }
        else
        {
          // or else report the status in "distance"
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