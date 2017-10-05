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
 */

#include <Arduino.h>
#include <nRF24L01.h>     // These are in the RF24 Library in
#include <RF24.h>         // the Aruino IDE Libraries folder
#include <RF24_config.h>
//  SPI library initialized in the two main programs
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

#if( RADIO_DEBUG)    //  RADIO_DEBUG is set in 'defines.h'
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
#endif

//  = = = = = = = =  Joystick Radio Routines  = = = = = = = = =
//  - - - - - - -  Do NOT compile for Mega2560  - - - - - - - -
#ifndef __AVR_ATmega2560__

  void bb_radio::setupJoystickRadio()
  {
      radio.begin();
      radio.setAutoAck( true);   // Enable auto-acknowledge packets (default).
      radio.enableAckPayload();  // Enable custom payloads on acknowledge packets.
      radio.enableDynamicPayloads();  // Enable dynamic payloads on ALL pipes.
      radio.setRetries( 4, 5);    // Set delay between retries in 250us increments.
                                  // Range: 0 to 15 max; 0 = 250us; 15 = 4000us.
                                  // Set number of retries before giving up.
                                  // Range: 0 to 15 max
      radio.setPALevel( 0);       // Set nRF24L01 TX power to minimum: -18dBm
      //  radio.setPALevel( 3);       // Set transmitter power to maximum:   0dBm
      radio.openReadingPipe( 1, pipe0);

      // Fill the TX FIFO buffers with jDat structure to send as ACK
      radio.writeAckPayload( 1, &jDat, sizeof( jDat));
      radio.writeAckPayload( 1, &jDat, sizeof( jDat));

      radio.startListening();      // Start listening

      #if( RADIO_DEBUG)
        printRadioInfo( JOYSTICK, &radio);  // Print configuration info of the nRF24 module
      #endif
  }

  //  If radio ready, read RX buffer into rDat1, send jDat as ACK.
  //  Then wait up to 0.5 second for radio ready again for rDat2.
  //  If rDat1 ID correct, set radio flag true, otherwise
  //  swap data structures and set radio flag false.
  //  Radio buffer structures (rBuf) are declared in 'shared.h'
  //
  void bb_radio::joystickRadioAck()
  {
      static uint32_t raTimer;    // radio available timer
      jDat.radio = false;
      // If Radio has received first packet and sent FIFO buffer as ACK.
      if( radio.available())
      {
          memset( &rBuf1, 0, sizeof( rBuf1));     // Clear the radio buffers
          memset( &rBuf2, 0, sizeof( rBuf2));
          radio.read( &rBuf1, sizeof( rBuf1));    // Read received data

          raTimer = millis() + 500;        // Set timer to 0.5 seconds
          while( !radio.available()        // If radio RX takes too long,
            && ( millis() < raTimer));     // break out of 'while' loop
          radio.read( &rBuf2, sizeof( rBuf2));    // Read received data
          // If the low order system time bytes are the same
          if( rBuf1.loTime == rBuf2.loTime)
          {
              jDat.radio = true;
              jDat.sync = true;
              if( rBuf1.mpxID == rDat1ID)
              {
                   memcpy( &rDat1, &rBuf1, sizeof( rBuf1));
                   memcpy( &rDat2, &rBuf2, sizeof( rBuf2));
              }
              else   /// Swap the buffers
              {
                   memcpy( &rDat2, &rBuf1, sizeof( rBuf1));
                   memcpy( &rDat1, &rBuf2, sizeof( rBuf2));
              }
          }
      }
      // Flush and refill first two radio TX FIFO buffers
      uint8_t flush = radio.flush_tx();
      radio.writeAckPayload( 1, &jDat, sizeof( jDat));
      radio.writeAckPayload( 1, &jDat, sizeof( jDat));
  }

/*    radio.startWrite( &jDat, sizeof( jDat), 0);        // write jDat packet with ACK
      radio.whatHappened( jtx_ok, jtx_fail, jrx_ready);  // test result
      if( jtx_ok)
      {
        jDat.radio = true;
        joyRadioRead();
        // Use write instead of startWrite to wait for built-in retry feature
        if( radio.write( &jDat, sizeof( jDat)))
        {
          jDat.radio2 = true;
          joyRadioRead();
          jDat.sync = ( rDat1.loTime == rDat2.bcLo);
        }
      }  */

/*    // Subroutine to Read an ACK packet into RX buffer, then sort
      // buffer into proper rDat structures according to mpxID.
      void joyRadioRead()
      {
          static uint8_t jBuf[ 32];              // One-time declaration of 32 byte RX buffer
          radio.read( &jBuf, sizeof( jBuf));     // read ACK packet back from radio write
          if( jBuf[31] == rDat1ID) memcpy( &rDat1, &jBuf, sizeof( jBuf));
            else if( jBuf[31] == rDat2ID) memcpy( &rDat2, &jBuf, sizeof( jBuf));
      }  */

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
      //radio.setPALevel( 0);      // Set radio TX power to minimum: -18dBm plus
                                 // New 20dB power amp yields +2dBm total TX output
      radio.setPALevel( 1);      // Set radio TX Power Amp Level to +8dBm
      radio.openWritingPipe( pipe0);

//      #if( RADIO_DEBUG)
//        printRadioInfo( PLATFORM, &radio);  // Print configuration info of the nRF24 module
//      #endif
  }

  //  Send two rDat data packets from the Platform to the Joystick
  //  and return two jDat data packets as acknowledgments.
  //  Do not alter current jDat structure unless RX is successful.
  void bb_radio::platformRadioAck()
  {
      // code to measure the amount of time for radio routine to execute
      //   static uint32_t tempTime;
      //   tempTime = millis();

      static uint32_t raTimer;    // radio available timer
      // Pass through if Joystick radio not working
      if( radio.write( &rDat1, sizeof( rDat1)))
      {
          memset( &rBuf1, 0, sizeof( rBuf1));     // Clear the radio buffers
          memset( &rBuf2, 0, sizeof( rBuf2));
          radio.read( &rBuf1, sizeof( rBuf1));    // Read received ACK data

          raTimer = millis() + 500;               // Set timer to 0.5 seconds
          while( !radio.write( &rDat2, sizeof( rDat2))   // If 'write' takes too long,
            && ( millis() < raTimer));                   // break out of 'while' loop
          radio.read( &rBuf2, sizeof( rBuf2));           // Read received data          

          // Check for valid second packet of joystick data
          if( ( rBuf2.mpxID == jDatID) && ( rBuf2.blank == 0))
          {
              fbSET( fbRadio);                        // set the rDat radio flag,
              memcpy( &jDat, &rBuf2, sizeof( rBuf2)); // save rBuf2 as jDat
          }
          else fbCLR( fbRadio);      // clear Radio flag (false)
      }
      //  rDat2.radioTime = (uint8_t)( millis() - tempTime);
  }

#endif
// - - - - - -  End of Platform Radio Routines  - - - - - -