/*  File Name: bb_joystick.h
 *  Developer: Bud Ryerson
 *  Last Work: 10MAY2015
 *  Described: header file to define the Joystick
               class of routines and procedures

  - Read two analog pins connected to two joysticks
    made of 100K pots with 5 volts across each

  - Read three digital pins connected to three pushbuttons
    that connect to ground when closed
    and internal PullUp Resistors when open.

  - Gravis joystick has new internal wiring
    Buttons connect to Pin 4 or GND when pressed.
    --------------------------------------
    1 BROWN  = VCC or +5 volts
    2 RED    = Button 1 or Left Upper Button
    3 ORANGE = Joystick X or Left/Right where Left = 0 and Right = 1024
    4 YELLOW = GND or Common or Ground
    5        = (not connected)
    6 BLUE   = Joystick Y or Forward/Back where Back = 0 and Forward = 1024
    7 PURPLE = Button 2 or Joystick Fire Button
    8 & 9    = (not connected)
   10 GREEN  = Button 3 or Left Lower Button

 */
 
//  --  --  --  --  --  Do NOT compile for Mega2560  --  --  --  --  --
//  Pin numbers will get confused
#ifndef __AVR_ATmega2560__

  #ifndef BB_JOYSTICK_H    //  Do not re-compile if previously included
  #define BB_JOYSTICK_H

  #include <Arduino.h>      //  It is always important to include this!

  class bb_joystick
  {
      public:
          bb_joystick();    //  library constructor
          ~bb_joystick();   //  library destructor
          bool  contact;    //  Contact established with PC
          bool  putFlag;    //  Ready to write data to PC
          bool  getFlag;    //  Ready to read data back from PC.
         
          void setup();
          void checkJoyButtons();
          void readData();
          void showLights();
          void printJoyData();
          void putPaneData();
          void getPaneData();
          void callPane();
          void paneDataLoop();
          void printAllData();
  };

  #endif  //  - - - - - -  End of Do NOT re-compile  - - - - - - -

#endif  //  - - - - - -  End of Do NOT compile for Mega2560 - - - - - - -