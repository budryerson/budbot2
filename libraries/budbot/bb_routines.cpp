/* File Name: bb_routines.cpp
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Described: C++ source file implementation of declared general
 *            and universal budbot2 functions and variables.
 * Last Work: 15SEP16 - changed the reference voltage for the
 *            ADC read routine from 2.5v to VCC.
 */

#include <Arduino.h>

#include <bb_shared.h>    // Declaration of shared budbot2 variables and structures
#include <bb_routines.h>  // Declaration of general and universal budbot2 routines

//  "microDelay()" routine replaces all "delayMicroseconds()"s
//  with something that does not use interrupts
void microDelay( int ud)
{
    unsigned long microTimer;
    microTimer = micros() + ud + 1;  // "+1" ensures delay will be AT LEAST given value
    while( micros() < microTimer);   // wait for micros timer to catch up
}

//  "milliDelay()" routine replaces all "delay()"s
//  with something that does not use interrupts
void milliDelay( int md)
{
    unsigned long milliTimer;
    milliTimer = millis() + md + 1;  // "+1" ensures delay will be AT LEAST given value
    while( millis() < milliTimer);   // wait for millis timer to catch up
}

//  Convert floating point number to long integer with rounding.
long floatToLong( float value)
{
  if( value >= 0) value += 0.5; else value -= 0.5;
  return (long)value;
}

//  Convert floating point number to integer with rounding.
int floatToInt( float value)
{
  if( value >= 0) value += 0.5; else value -= 0.5;
  return (int)value;
}

//  Constrain an integer to a given range of integers
void intConstrain( int &value, int low, int high)
{
    if( value < low) value = low;
    else if ( value > high) value = high;
}

// integer division with rounding
// n = numerator; d = denominator
int intDiv( int n, int d)
{
    if( ( n < 0) == ( d < 0))    // If n and d are the same sign ...
    {
        if( n < 0)           // If n (and d) are negative ...
        {
            n = -n;
            d = -d;
        }
        // Unsigned division rounds down.  Adding d-1 to n effects a round up.
        return ( ( (unsigned)n) + ( (unsigned)d) - 1) / ( (unsigned)d);
    }
    else return ( n / d);
}

// long integer division with rounding
// n = numerator; d = denominator
long longDiv( long n, long d)
{
    if( ( n < 0) == ( d < 0))  // If n and d are the same sign ...
    {
        if( n < 0)             // If n (and d) are both negative ...
        {
            n = -n;
            d = -d;
        }
        // Unsigned division rounds down.  Adding d-1 to n effects a round up.
        return (long)( (unsigned long)( n + d - 1) / ( unsigned long)d);
    }
    else return ( n / d);
 }

// prints val with up to six decimal places given by precision
// example: printFloat( 3.1415, 2) prints 3.14 (two decimal places)
void printFloat( float val, byte precision)
{
    if( val < 0.0)
    {
        // print leading spaces if negative
        if( val > -100.0) Serial.print(' ');
        if( val > -10.0) Serial.print(' ');
        Serial.print('-');
        val = -val;
    }
    else
    {
        // print leading spaces if not negative
        if( val < 100.0) Serial.print(' ');
        if( val < 10.0) Serial.print(' ');
        Serial.print(' ');
    }

    Serial.print( int( val));  // print the int part
    if( precision > 0)
    {
        Serial.print(".");     // print the decimal point
        unsigned long frac;
        unsigned long mult = 1;
        byte padding = precision - 1;     // for precision = 2, padding = 1
        while( precision--) mult *= 10;   // for precision = 2, mult = 100

        if( val >= 0) frac = ( val - int( val)) * mult;
            else frac = ( int(val) - val ) * mult;
        unsigned long frac1 = frac;
        while( frac1 /= 10 ) padding--;   //  frac1 = frac1 / 10
        while(  padding--) Serial.print("0");
        Serial.print( frac, DEC);
    }
}

// = = = = = = =  FLAG BIT MANIPULATION Routines  = = = = = = = =
//  Routines to Set, Check, Toggle, Clear and Button, which
//  sets the bit according to a given boolean value
//  'longOne' is set in defines.h as 0x00000001UL
// #define longOne 0x00000001UL
void fbSET( int index)
{
  rDat1.flagBits |= ( longOne << index);
}

bool fbCHK( int index)
{
  return  rDat1.flagBits & ( longOne << index);
}

void fbTOG( int index)
{
    rDat1.flagBits ^= ( longOne << index);
}

void fbCLR( int index)
{
  rDat1.flagBits &= ~( longOne << index);
  // ~ is the one's complement (or bitwise not) operator
}

void fbBUT( int index, bool value) // CLR if value is 0 (false) or SET if <> 0 (true)
{
  if( value != 0) rDat1.flagBits |= ( longOne << index);  // bbSET
      else rDat1.flagBits &= ~( longOne << index);        // bbCLR
}
// - - - - - - - - -  End FLAG BIT MANIPULATION  - - - - - - - - -

// = = = = = = = =  Clear All Data Structures  = = = = = = = = = =
// Add a coded IU byte at end of structure to help ensure proper transfer
// ID values are set in 'defines.h'
void clearDataStructures()
{
    memset( &jDat,  0, sizeof( jDat));   //  clear joystick data structure
    //  jDat values .serial, .radio and .mpxID are all set to 0 or 'false'
    jDat.mpxID = jDatID;                 //  set ID bit pattern to 1100 1101
    memset( &rDat1, 0, sizeof( rDat1));  //  clear platform data structure 1
    rDat1.mpxID = rDat1ID;               //  set ID bit pattern to 1001 0110
    memset( &rDat2, 0, sizeof( rDat2));  //  clear platform data structure 2
    rDat2.mpxID = rDat2ID;               //  set ID bit pattern to 0110 1001
}
// - - - - - - - - -  End of Clear Radio Data Structure  - - - - - - - - -



#ifdef __AVR_ATmega2560__
  //  ------------------------------------------------------
  //  Custom routine for reading analog to digital inputs
  //  ------------------------------------------------------
  //  Sets multiplexer to input channel and gets ADC data
  //  Includes a recommended discard of the first conversion
  //  Returns an unsigned integer value.
  uint16_t readADCInput( uint8_t adcIn)
  {
      //  Register Description begins on p.281 of the Mega256 data sheet
      uint16_t result;       // Declare variable 'result'
      //  The ADMUX register controls the inputs to the ADC
      //  First 8 analog inputs (A0-A7) are ATmega pins 54 - 61
      //  Second 8 analog inputs (A8-A15) are ATmega pins 62 - 69
      ADMUX = adcIn - 54;    // set pin numbers 0 - 15 in ADMUX3:0
      if( adcIn > 61)        // If reading inputs above A7,
      {                      // such as for motor current,
          CLR( ADMUX, 3);    // clear MUX 3, and turn on
          SET( ADCSRB, 3);   // MUX 5 (ADCSRB:3)instead
          SET( ADMUX, 7);    // set 1.1V as reference
      }
      else
      {                        // When reading IR sensor
          SET( ADMUX, 6);      // set AVCC as reference
   //       SET( ADMUX, 6);    // set 2.56V as reference
   //       SET( ADMUX, 7);
          CLR( ADCSRB, 3);     // and turn OFF MUX5
      }
      SET( ADCSRA, 7);       // Set Reg A bit 7 = Enable ADC (ADEN)
      SET( ADCSRA, 6);       // Set Reg A bit 6 = Start Conversion (ADSC)
      while( CHK( ADCSRA, 6));  // Wait for conversion to complete
      SET( ADCSRA, 6);       // Discard first conversion. Start again.
      while( CHK( ADCSRA, 6));  // Wait for conversion to complete
      result = ADCL;         // Get low order bits
      result |= ADCH << 8;   // Get high order bits
      CLR( ADCSRA, 7);       // Set Reg A bit 7 = AD Enable OFF
      return result;
  }

  //  Set registers for Analog to Digital Converter (ADC)
  void setupADCregisters()
  {
    cli();  //  stop all interrupts
      //  Turn on the Analog Comparator
      ACSR = 0;
      //  Status Reg A: ADC Enable, 128 prescalar
      ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
      //  Status Reg B: Multiplex Enabled (ACME), no trigger (free running)
      ADCSRB = 0x40;
      //  MUX Reg: AVCC with external capacitor at AREF pin
      ADMUX = 0x40;
      //  Disable all inputs except A0-A3 & A8-A11. Saves power?
      //  When bit is written logic one, the digital input buffer
      //  on the corresponding ADC pin is disabled.
      DIDR0 = 0xF0;        //  Disable inputs 7-4
      DIDR2 = 0xF0;        //  Disable inputs 15-12
    sei();  //  allow interrupts again
  }

  //  Set Timer 3 to interrupt once every 50 milliseconds
  void setupTimer3()
  {
    cli();  //  stops all interrupts
        TCCR3A = 0;                 // clear the 3A register
        TCCR3B = 0;                 // clear the 3B register
        // Register is set by shifting 0b00000001 by CS12 bits to the left.
        // Then bitwise OR into the current value of TCCR3B, which is zero.
        TCCR3B |= ( 1 << WGM32) | ( 1 << CS32);  // CTC Mode and 256 prescaler
        //  TCCR3B |= (  1 << WGM32) | ( 1 << CS31) | ( 1 << CS30);  // CTC Mode and 64 prescaler
        //  TCCR3B |= ( 1 << CS32) | ( 1 << CS30); // sets 1024 as prescaler
        //  Similar Code:  TCCR3B |= _BV(CS12); or TCCR3B = 4;
        TIMSK3 |= ( 1 << OCIE3A);    // enable CTC or "Clear Timer on Compare" interrupt
        TCNT3 = 0;      // initialize 16 bit counter value to 0
                        // this will be reset after every compare
        OCR3A =  3124;  // set Output Compare Register 3A to 0.05 sec
                        // 16,000,000/sec clock / 256 prescalar = 62,500/sec
                        // 62,500/sec * 0.05 sec = 3125
                        // OCR3A = 3125 - 1
        //  OCR3A =  1562;  // set Output Compare Register 3A to 0.025 sec
    sei();  //  allows interrupts again
  }

  //  Print various platform data to platform serial port
  //  For test purposes - Must be connected to serial line
  void printRobotData()
  {
      printf("jDat:");
      printf( "|JSC%10lu", jDat.joyClock);  //   Platform Clock in milliseconds
      printf( "|JLC%10lu", jDat.joyLoop);   // joystick Loop Counter
      printf( "|JSB");
      for( int x = 0; x < 16; ++x)
      {
          if( ( jDat.sigBits >> x) & 1) printf( "1"); else printf( "0");
      }

      printf( " |JX%4i|JY%4i", jDat.jvX, jDat.jvY);      //  joystick values: 0 - 1023
      printf( " |PV% 04i|PR% 04i", jDat.velocity, jDat.rotation);  //  motion values: -255 to 255

      printf( " rDat1");
      printf( " |RV% 04i |RR% 04i", rDat1.velocity, rDat1.rotation);
      printf( " |SP% 04i", rDat1.srvPos);  //  Servo Pan Position: 0 - 180°

      printf("|FB");
      for( int x = 0; x < 32; ++x)
      {
          if( ( rDat1.flagBits >> x) & 1) printf( "1"); else printf( "0");
      }
      printf( " rDat2");
      printf( "|BSC%10lu", rDat2.botClock);   //  roBot System Clock in milliseconds
      printf( "|BLC%10lu", rDat2.botLoop);  //  roBot Loop Counter

      printf("eD:% 7iL ", eDatL.epc);        //  total Left encoder count
      printf( "dP% 7iY", rDat2.drPosY);      // dead reckoning position Y

      //      printf("tSpd: % 4iL | % 4iR ", motDatL.mSpdTarget, motDatR.mSpdTarget);
//      printf("mDir: % 2iL | % 2iR ", motDatL.mDir, motDatR.mDir);
//      printf("mSpdL: % 4i ", rDat1.motorSpeedLeft);       //  speed ±255
//      printf("mSpdR: % 4i ", rDat1.motorSpeedRight);

//      printf("Course: %03u° ", rDat1.course);  //  desired direction 0 - 359°
//      printf("Head: %03u° ",  rDat1.head);     //  actual direction 0 - 359°
//      printf("Bear: %03i° | ",  rDat1.bear);   //  difference of course and heading ±180

//      printf("eCount: % 3uL | % 3uR");      //  Encoder interrupt counters - reset every 50ms.
//      printf("M%1u ", fbCHK( fbMotor));     //  isMotorON
//      printf("S%1u ", fbCHK( fbServo));     //  isServoON
//      printf("A%1u ", fbCHK( fbAuto));      //  isAutoMode
      //printf("E%1u ", fbCHK( fbEvade));     //  is running evasion program
      //printf("| mCurrent: ");
      //printf("LF %05u ", rDat2.motCurRay[ 0]); //  Motor Current
      //printf("RF %05u ", rDat2.motCurRay[ 1]);
      //printf("LR %05u ", rDat2.motCurRay[ 2]);
      //printf("RR %05u", rDat2.motCurRay[ 3]);

      printf("\r\n");                       //  end of line
  }
  
  // = = = = = = =  Print All Data Structures  = = = = = = =
  // - - - - - - -  only for testing purposed  - - - - - - -

      // subroutine for 'printAllData' function
      void printDataLine( uint8_t charRay[] )
      {
          // Lookup table for Hexidecimal numbers
          const char hex_str[] = "0123456789ABCDEF";
          static char buf2[] = "00000000|00000000|00000000|00000000|00000000|00000000|00000000|00000000";
          int z = 0;
          for (int x = 0; x < 8; ++x)
          {
              for( int y = 0; y < 4; ++y)
              {
                  buf2[ z] = hex_str[ ( charRay[ (x * 4) + y] >> 4) & 0x0F];
                  ++z;
                  buf2[ z] = hex_str[ ( charRay[  (x * 4) + y]) & 0x0F];
                  ++z;
              }
              ++z;
          }
          printf("%s\r\n", buf2);
      }

  void printAllData()
  {
      printf( DASHLINE);
      static uint8_t charRay[ 32];
      memmove( &charRay, &jDat, 32);
      printf( " jDat: ");
      printDataLine( charRay);
      memmove( &charRay, &rDat1, 32);
      printf( "rDat1: ");
      printDataLine( charRay);
      memmove( &charRay, &rDat2, 32);
      printf( "rDat2: ");
      printDataLine( charRay);
  }
  // - - - - - - -  End of Print All Data Structure  - - - - - - -

#endif   // - - - - - #if defined(__AVR_ATmega2560__) - - - - - - - -

//  = = = = = = = =  Joystick Only Routines  = = = = = = = = =
//  - - - - - - -  Do NOT compile for Mega2560  - - - - - - -
#ifndef __AVR_ATmega2560__

  //  Set Timer 2A to interrupt once every 25 milliseconds
  void setupTimer2()
  {
    cli();  //  stop all interrupts
        TCCR2A = 0;                 // clear the register
        TCCR2A |= ( 1 << WGM21);    // Clear Timer on Compare (CTC)
        TCCR2B = 0;                 // clear the register
        TCCR2B |= ( 1 << CS22) | ( 1 << CS21) | ( 1 << CS20);   // 1024 prescaler
        // Register is set by shifting 0b00000001 by CS2x bits to the left.
        // Then bitwise OR into the current value of TCCR2B, which is zero.
        // Similar Code:  TCCR2B = 7;
        // TCCR2B |= ( 1 << CS22) | ( 1 << CS21); // sets 256 as prescaler
        TIMSK2 |= ( 1 << OCIE2A);    // Output Compare A Match interrupt enable
        TCNT2 = 0;      // initialize 16bit counter value to 0
                        // 16,000,000/sec clock / 1024 prescalar = 15625/sec
                        // 15625/sec * 0.0125 sec = 195
                        // OCR2A = 195 - 1
        OCR2A =  194;   // set Output Compare Register 2A to 0.0125 sec
    sei();  //  allow interrupts again
  }

#endif
// - - - -  #if not defined(__AVR_ATmega2560__)  - - - - -
// - - - - - -  End of Joystick Routines  - - - - - - - - -