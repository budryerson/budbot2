/* File Name: bb_alarm.h
 * First Day: lost in the mists of time
 * Developer: Bud Ryerson
 * Last Work: 06MAR16
 * 29JAN17 - No longer an object. Alarm routines are now normal code.
 * Described: declaration of variables and functions supporting "alarm"
 *            there are four kinds of alarms
 *            1. Over current on a motor
 *            2. Corner sensor hit
 *            3. Compound eye too close
 *            4. Loss of radio communication
 */

#ifndef _BB_ALARM_H         //  If this file is not already defined
#define _BB_ALARM_H         //  Then define a token for this file

#include <Arduino.h>    //  It is very important to remember this!

class bb_alarm
{
  public:
      bb_alarm();
      ~bb_alarm();

      void setup();  //  setup corner in and out pins
      void showLED();
      void toggleFlagBits();
      bool testMotorCurrent();
      bool testObjectDetect();   // test each corner for Collision Detect
//       void toggleObjectDetect();
//       void resetObjectDetect();  // rest Collision Detect flags
//       void readAlarmSensors();
//       void zeroDistance();
//       void resetRobotButtons();
      void checkPlatformButtons();
      void printStatus();

  private:
      uint16_t sigState;       // signal state storage variable
      bool testPlatformButton( uint8_t pButNmbr);
};

#endif