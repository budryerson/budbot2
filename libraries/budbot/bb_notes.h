/* File Name: bb_notes.h
 * First Day: 10MAY2015
 * Developer: Bud Ryerson
 * Last Work: 2JUN2015
 * 20150909 - added "Take On Me"
 * Described: Definitions of musical notes
 */
 
 #define NO_B0  31  //  B
 #define NO_C1  33  //  C
 #define NO_CS1 35  //  C#
 #define NO_D1  37  //  D
 #define NO_DS1 39  //  D#
 #define NO_E1  41  //  etc
 #define NO_F1  44
 #define NO_FS1 46
 #define NO_G1  49
 #define NO_GS1 52
 #define NO_A1  55
 #define NO_AS1 58
 #define NO_B1  62
 #define NO_C2  65
 #define NO_CS2 69
 #define NO_D2  73
 #define NO_DS2 78
 #define NO_E2  82
 #define NO_F2  87
 #define NO_FS2 93
 #define NO_G2  98
 #define NO_GS2 104
 #define NO_A2  110
 #define NO_AS2 117
 #define NO_B2  123
 #define NO_C3  131
 #define NO_CS3 139
 #define NO_D3  147
 #define NO_DS3 156
 #define NO_E3  165
 #define NO_F3  175
 #define NO_FS3 185
 #define NO_G3  196
 #define NO_GS3 208
 #define NO_A3  220
 #define NO_AS3 233
 #define NO_B3  247
 #define NO_C4  262
 #define NO_CS4 277
 #define NO_D4  294
 #define NO_DS4 311
 #define NO_E4  330
 #define NO_F4  349
 #define NO_FS4 370
 #define NO_G4  392
 #define NO_GS4 415
 #define NO_A4  440
 #define NO_AS4 466
 #define NO_B4  494
 #define NO_C5  523
 #define NO_CS5 554
 #define NO_D5  587
 #define NO_DS5 622
 #define NO_E5  659
 #define NO_F5  698
 #define NO_FS5 740
 #define NO_G5  784
 #define NO_GS5 831
 #define NO_A5  880
 #define NO_AS5 932
 #define NO_B5  988
 #define NO_C6  1047
 #define NO_CS6 1109
 #define NO_D6  1175
 #define NO_DS6 1245
 #define NO_E6  1319
 #define NO_F6  1397
 #define NO_FS6 1480
 #define NO_G6  1568
 #define NO_GS6 1661
 #define NO_A6  1760
 #define NO_AS6 1865
 #define NO_B6  1976
 #define NO_C7  2093
 #define NO_CS7 2217
 #define NO_D7  2349
 #define NO_DS7 2489
 #define NO_E7  2637
 #define NO_F7  2794
 #define NO_FS7 2960
 #define NO_G7  3136
 #define NO_GS7 3322
 #define NO_A7  3520
 #define NO_AS7 3729
 #define NO_B7  3951
 #define NO_C8  4186
 #define NO_CS8 4435
 #define NO_D8  4699
 #define NO_DS8 4978
 
void playReadyMusic();
void playNote( uint8_t);
