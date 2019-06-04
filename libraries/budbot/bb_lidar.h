/* File Name: bb_lidar.cpp
 * Developer: Bud Ryerson
 * Described: Declarations for 'bb_lidar.cpp', the lidar code file.
 * Inception: 28MAR19
 */

#ifndef BB_LIDAR_H
#define BB_LIDAR_H

#include <Arduino.h>  // It is very important to always remember this!

class bb_lidar
{
    public:
        bb_lidar();   // constructor
        ~bb_lidar();  // destructor

        void setup();  // setup lidar sensor
        void read();   // read Lidar sensor and get data

        void factoryReset();
        void frameRate( uint16_t rate);
        void saveSettings();
};

#endif

