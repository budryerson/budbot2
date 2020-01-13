/* File Name: bb_radio.cpp
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Described: Acknowledge Radio Routines
 *  - - - - - - - - - - - - - - - - - - - - - - -
 *  Implement variables and routines for an nRF24L01+ device
 *  using a single pipe for TX and RX with the Radiohead library.
 *  The robot platform is the primary transmitter (PTX) and sends
 *  telemetry data.  It receives command data back as an ACK packet
 *  from the joystick, which acts as the primary receiver (PRX).
 *  - - - - - - - - - - - - - - - - - - - - - - -
 *  Last Work: 04JUL2015 - radio available timeout
 *    13SEP2015 - procedures for using acknowledge packet
 *    01NOV2015 - created printRadioInfo routine
 *    11DEC16 - moved SPI initialization to the two main programs
 *    16JAN17 - trying a "fastWrite" technique to transfer 2 packets
 *              at once rather than 1 packet twice.
 *    8FEB17 - after thorough review, abandoned "fastWrite()" because
 *             it did not do what I wanted, namely 'acknowledgment'
 *    24FEB17 - tried out new hi power radios, power supply problems, I think
 *              new radios draw 10x the power, set TX power to minimum
 *    25JUN17 - Reversed the radio com paradigm. Now robot platform is primary
 *              transmitter (PTX) and joystick is primary receiver (PRX)
 *    18JUL17 - added Wait Timers to both sides to avoid hang
 *              while waiting for second packet
 *    05DEC19 - Added high power module to platform.
 *              Set TX to lowest power (0).  Seems to be working okay.
 *    25DEC19 - Added high power module to joystick.
 *              Set TX to lowest power (0).  Did not work.
 * Maybe high power module requires more current?
 * Switched back to regular module, left power at 'minimum'.
 * Works okay now.
 */

#include <Arduino.h>
#include <nRF24L01.h>     // These files are in the 'RF24' Library
#include <RF24.h>         // in the 'D:\IDE\arduino\libraries' folder.
#include <RF24_config.h>
//  The SPI library is initialized in each of the two
//  main programs 'bb2_bot.ino' and 'bb2_joy.ino'.
#include <bb_defines.h>
#include <bb_shared.h>
#include <bb_routines.h>
#include <bb_radio.h>

bb_radio::bb_radio(){}  //  "::" is the 'scope resolution operator'
bb_radio::~bb_radio(){}

const uint64_t pipe0 = 0xE8E8F0F0E1LL;  // only need one pipe
const uint8_t plSize = 32;  //  maximum payload size is 32 bytes

RF24 radio( CEpin, CSNpin); //  Board specific SPI pin numbers
                            //  as defined in 'bb_defines.h'

#define JOYSTICK 0    //  numeric values to identify 'myDevice'
#define PLATFORM 1

//#if( RADIO_DEBUG)    //  RADIO_DEBUG is set in 'defines.h'
  void printRadioInfo( int myDevice, RF24* myRadio)
  {
      //  Print the name of the Device chosen
      if( myDevice == JOYSTICK) printf( "This is the JOYSTICK radio. ");
          else printf( "This is the PLATFORM radio. ");
      //  Print the 'Chip Enable' (CE) & 'Chip Select' (CSN)
      //  pin numbers to make sure they are correct.
      printf( "CEpin= %u, CSNpin= %u\r\n", CEpin, CSNpin);
      // Print the configuration of the nRF24L01+ module
      (*myRadio).printDetails();
  }
//#endif

//  = = = = = = = =  Joystick Radio Routines  = = = = = = = = =
//  - - - - - - -  Do NOT compile for Mega2560  - - - - - - - -
#ifndef __AVR_ATmega2560__

  void bb_radio::setupJoystickRadio()
  {
      radio.begin();
      radio.maskIRQ( 1, 1, 0);   // Mask all interrupts except 'payload received'
      radio.setAutoAck( true);   // Enable auto-acknowledge packets (default).
      radio.enableAckPayload();  // Enable custom payloads on acknowledge packets.
      radio.enableDynamicPayloads();  // Enable dynamic payloads on ALL pipes.
      radio.setRetries( 4, 5);    // Set delay between retries in 250us increments.
                                  // Range: 0 to 15 max; 0 = 250us; 15 = 4000us.
                                  // Set number of retries before giving up.
                                  // Range: 0 to 15 max
      radio.setPALevel( 0);       // Set radio Power Amp level to minimum.
      //radio.setPALevel( 3);       // Set Power Amplifier output to one of four levels
                                  // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX,
                                  // corresponding to NRF24L01 output levels:
                                  // -18dBm, -12dBm,-6dBM, and 0dBm
                                  // If the value is more than 3, set level to MAX
                                  // (The NRF24L01+ has a 10db power amp)
      radio.openReadingPipe( 1, pipe0);

      // Fill the TX FIFO buffers with jDat structure to send as ACK
      //radio.writeAckPayload( 1, &jDat, sizeof( jDat));
      //radio.writeAckPayload( 1, &jDat, sizeof( jDat));

      radio.startListening();      // Start listening

      #if( RADIO_DEBUG)
        printRadioInfo( JOYSTICK, &radio);  // Print configuration info of the nRF24 module
      #endif
  }

  //  Radio ready, so set jDat as ACK and read radio into RX buffer.
  //  If rDat1 ID NOT correct, reset radio flag and start over;
  //  otherwise set radio1 flag and wait for next radio RX.
  //  If rDat2 ID correct, set radio2 flag true and check sync
  //  Radio buffer structures (rBuf) are declared in 'shared.h'
  //
  void bb_radio::joystickRadioAck()
  {
      //static uint32_t jrTimer;         // joystick radio timer
      //jrTimer = millis();              // save current time

      jDat.radio1 = false;             // reset rDat1 packet radio flag
      jDat.radio2 = false;             // reset rDat2 packet radio flag
      radio.writeAckPayload( 1, &jDat, sizeof( jDat));  // fill ACK buffer
      radio.read( &rBuff, sizeof( rBuff));    // Fill radio read buffer
                                              // and flush ACK FIFO
      if( rBuff.mpxID == RDAT1_ID)     // if rDat1 received
      {
          jDat.radio1 = true;                        // set rDat1 radio flag
          memcpy( &rDat1, &rBuff, sizeof( rBuff));   // copy buffer to rDat1
          radio.writeAckPayload( 1, &jDat, sizeof( jDat));  // fill ACK buffer
          while( !radio.available());                // wait for next radio packet
          radio.read( &rBuff, sizeof( rBuff));    // Fill radio read buffer
                                                  // and flush ACK FIFO
          if( rBuff.mpxID == RDAT2_ID)    // if rDat2 received
          {
              jDat.radio2 = true;                        // set rDat2 radio flag
              memcpy( &rDat2, &rBuff, sizeof( rBuff));   // copy buffer to rDat2
          }
          // set sync flag to whether two rDat packets are received in correct order
          jDat.sync = ( ( rDat1.loTime == rDat2.loTime) && jDat.radio1 && jDat.radio2);
      }
      else
      {
          // If first data from radio is not rDat1, then radio is out of sync.
          // Flush the ACK buffer and return to call conditional in Main Loop.
          radio.flush_tx();
      }
      //jDat.loopTime = (uint16_t)( millis() - jrTimer); // measure radio loop time
      //jDat.radioIRQ = false;  //  Reseting the radio flag for another IRQ is the last thing to do.
  }

#endif
// - - - - - -  End of Joystick Radio Routines  - - - - - -

//  = = = = = = =  Platform Radio Routines  = = = = = = = =
//  - - - -  Compile ONLY for Mega 2560 platform  - - - - -
#ifdef __AVR_ATmega2560__

  void bb_radio::setupPlatformRadio()
  {
      radio.begin();
      radio.setAutoAck( true);   // Enable auto-acknowledge packets ( default).
      radio.enableAckPayload();  // Enable custom payloads on acknowledge packets.
      radio.enableDynamicPayloads();  // Enable dynamic payloads on ALL pipes.
      radio.setRetries( 4, 5);   // Set delay between retries from 0 to 15 max
                                 // in 250us increments. 0 = 250us; 15 = 4000us.
                                 // Set number of retries from 0 to 15 max before giving up.
      radio.setPALevel( 0);      // Set radio TX power to minimum: -18dBm plus
                                 // New 20dB power amp yields +2dBm total TX output
      //radio.setPALevel( 3);      // Set radio TX Power Amp Level to maximum: +8dBm
      radio.openWritingPipe( pipe0);

//      #if( RADIO_DEBUG)
//        printRadioInfo( PLATFORM, &radio);  // Print configuration info of the nRF24 module
//      #endif
  }

  //  Send two rDat packets to the Joystick from the Platform
  //  and return the jDat packet as acknowledgments each time.
  //  Do not alter current jDat values unless 'read' is valid.
  void bb_radio::platformRadioAck()
  {
      // code to measure the amount of time for radio routine to execute
      static uint32_t prTimer;    // platform radio timer
      prTimer = millis();

      // Pass through if Joystick radio not working
      if( radio.write( &rDat1, sizeof( rDat1)))   // Test whether first TX/ACK completes, then...
      {
          fbSET( fbRadio);                        // set the rDat radio flag (true), and...
          if( radio.isAckPayloadAvailable())      // if there is something in the RX buffer...
          {
              radio.read( &rBuff, sizeof( jDat)); // then fill rBuf with ACK from RX buffer...
              if( rBuff.mpxID == JDAT_ID)                  // and if data is valid...
                  memcpy( &jDat, &rBuff, sizeof( rBuff));  // then copy rBuf to jDat.
          }
          radio.write( &rDat2, sizeof( rDat2));   // Now, assume that radio link is okay and
          if( radio.isAckPayloadAvailable())      // that second TX/ACK will also complete, so...
          {
              radio.read( &rBuff, sizeof( jDat));  // again fill rBuf from RX FIFO buffer...
              if( rBuff.mpxID == JDAT_ID)                  // and if data is valid...
                  memcpy( &jDat, &rBuff, sizeof( rBuff));  // then copy rBuf to jDat.
          }
      }
      else fbCLR( fbRadio);            // clear Radio flag (false)

      rDat2.radioTime = (uint8_t)( millis() - prTimer);  // time to complete TX routine
  }

#endif
// - - - - - -  End of Platform Radio Routines  - - - - - -