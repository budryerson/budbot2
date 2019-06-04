/* File Name: bb_notes.cpp
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Last Work: 2JUN2015
 * 20150909 - added "Take On Me"
 * Described: Arrays of notes and songs.
 */

//  Compile this entire file only for the MEGA2560
#ifdef __AVR_ATmega2560__

  #include <Arduino.h>    //  It is always important to include this!

  #include <bb_defines.h>   // sets the 'speaker' pin number
  #include <bb_routines.h>  // gives us 'milliDelay()'
  #include <bb_notes.h>     // defines various notes: NO_XX


  //const int wakeSong[] = { NO_C4, NO_G3, NO_G3, NO_A3, NO_G3,  0, NO_B3, NO_C4};
  //const int wakeDurs[] = {   200,   100,   100,   200,   200,  200,  200,  200};
  //const int MrFrog1[] = { NO_C4, NO_E4, NO_G4, NO_E4, NO_F4, NO_D4, NO_E4};
  //const int MrFrog1Durs[] =  {  200,  200,  200,  200,  200,  200,  200};
  //const int MrFrog2[]  =  { NO_C4, NO_A4, NO_F5, NO_A4, NO_G5, NO_B4, NO_C4};
  //const int MrFrog2Durs[] =  {  200,  200,  200,  200,  200,  200,  200};

  const int noteScale[ 32] =  { NO_C4, NO_D4, NO_E4, NO_F4, NO_G4, NO_A5, NO_B5, NO_C5,
                                NO_D5, NO_E5, NO_F5, NO_G5, NO_A6, NO_B6, NO_C6, NO_D6,
                                NO_C4, NO_D4, NO_E4, NO_F4, NO_G4, NO_A5, NO_B5, NO_C5,
                                NO_D5, NO_E5, NO_F5, NO_G5, NO_A6, NO_B6, NO_C6, NO_D6 };

  // "Take On Me" synthesizer solo
  const int tomNote[ 24] = { NO_FS5, NO_FS5, NO_D5, NO_B5, NO_B5, NO_E5, NO_E5, NO_E5,
                             NO_GS5, NO_GS5, NO_A6, NO_B6, NO_A6, NO_A6, NO_A6, NO_E5,
                             NO_D5, NO_FS5, NO_FS5, NO_FS5,NO_E5, NO_E5, NO_FS5, NO_E5 };
  const uint8_t tomDurs[ 24] = {  100,  100,  100,  200,  200,  200,  200,  100,
                                  100,  100,  100,  100,  100,  100,  100,  200,
                                  200,  200,  200,  100,  100,  100,  100,  100 };

  //  play 12 tone ready music: "Take On Me"
  void playReadyMusic()
  {
      for( uint8_t x = 0; x < 12; x++)
      {
          tone( Speaker, tomNote[ x], tomDurs[ x]);    // Play one tone for each read
          milliDelay( tomDurs[ x] * 2);
      }
  }

  void playNote( uint8_t x)                //  called by IREye Calibration routine
  {
      tone( Speaker, noteScale[ x], 90);   //  Play one 0.09 second tone for each read
      milliDelay( 160);                    //  Pause for 0.16 seconds after each
  }

#endif  // End of #ifdef __AVR_ATmega2560__
