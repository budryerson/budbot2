/* File Name: bb_routines.h
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Last Work: 14NOV15 - removed tBump()
 * Described: Header file declarations that define general and universal
 *            budbot routines.  These are classless.
 */

#ifndef BB_ROUTINES_H   //  'include guard' macro to prevent errors
#define BB_ROUTINES_H   //  at compilation time from multiple calls

#include <Arduino.h>    //  It is always important to include this!

void microDelay( int ud);
void milliDelay( int md);
long floatToLong( float value);
int  floatToInt( float value);
int  intConstrain( int value, int high, int low);
void printFloat( float val, byte precision);
int  intDiv( int n, int d);       //  Integer Division with Rounding
long longDiv( long n, long d);   //  Long Integer Division with Rounding

// Flagbit Manipulation Routines
bool fbCHK( int index);
void fbSET( int index);
void fbTOG( int index);
void fbCLR( int index);
void fbBUT( int index, bool value);
void clearDataStructures();
void printAllData();
//bool buttonPress( int buttonPin, int buttonValue);

  #ifdef __AVR_ATmega2560__
    uint16_t readADCInput( uint8_t adcIn);
    void setupADCregisters();
    void setupTimer3();
    void setupTimer4();
    void printRobotData();
  #endif

  #ifndef __AVR_ATmega2560__
    void setupTimer2();
  #endif


#endif  // end of BB_ROUTINES_H include guard