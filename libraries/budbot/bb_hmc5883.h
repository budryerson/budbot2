/* File name: bb_hmc5883.h
 * Developer: Bud Ryerson
 * First day: 11DEC15
 * Last work: 06MAR16
 * Description: Header for the 'hmc5883.cpp' file
 * to interface a Honeywell HMC5883L magnetometer
 * to the 'bb_compass' library using the Arduino I2C.
 */

#include <Arduino.h>
#include <Wire.h>
#include <bb_defines.h>

#ifndef BB_HMC5883_H
#define BB_HMC5883_H

#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0  // Configuration Register A
#define HMC58X3_R_CONFB 1  // Configuration Register B
#define HMC58X3_R_MODE  2  // Mode Register
#define HMC58X3_R_XM    3  // Register X MSB (Most Significant Bit)
#define HMC58X3_R_XL    4  // Register X LSB (Least Significant Bit)
#define HMC58X3_R_ZM    5  // Note: Z Register is read before Y in the HMC5883
#define HMC58X3_R_ZL    6
#define HMC58X3_R_YM    7
#define HMC58X3_R_YL    8
#define HMC58X3_R_STAT  9  // Status register

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       // Y axis level when bias current is applied.

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   // High limit when gain is 5.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

class bb_hmc5883
{
  public:
    bb_hmc5883();    //  library constructor
    //~bb_hmc5883();   //  library destructor
    void init();
//    void getValues( float *x, float *y, float *z);
//    void getValues( float *xyz);
    void getValue( void *value);
    void getRaw( int *x, int *y, int *z);
//    void getRaw( int *xyz);
    void getScale( uint8_t gain, uint8_t n_samples, void *bias);
    void getOffset( int n_samples, void *offset);
    byte getGain();
    void setBias( uint8_t mode);
    void setSamples( uint8_t mode);
    void setMode( uint8_t mode);
    void setDOR( uint8_t DOR);
    void setGain( uint8_t gain);

    void getID( char id[ 3]);
  private:
    void writeReg( uint8_t reg, uint8_t val);
    byte readReg( uint8_t reg);
//    float scale[ 3], offset[ 3];
};

#endif // BB_HMC5883_H

