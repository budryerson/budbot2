/* File Name: bb_hmc5883.cpp
 * Developer: Bud Ryerson
 * First day: 11DEC15
 * Last work: 11DEC15
 * Description: Interface a Honeywell HMC5883L magnetometer
 * to the 'bb_compass' library using the Arduino I2C
 */

//  This entire library file is defined to compile only
//  for the robot body platform's MEGA2560 controller
#if defined(__AVR_ATmega2560__)

#include <Arduino.h>       //  like a damned belly button
//#include <Wire.h>

#include <bb_shared.h>
#include <bb_routines.h>
#include <bb_hmc5883.h>

//  Comment out DEBUG to keep warning messages from compiling
//#define DEBUG (1)

#ifdef DEBUG                    //  If DEBUG defined above,
    #define DEBUG_PRINT printf  //  compile as 'printf'.
#else                           //  Otherwise...
    #define DEBUG_PRINT         //  do not compile.
#endif

// From HMC5883 Data Sheet: Counts/Gauss for each Gain setting
const uint16_t counts_per_gauss[ 8] =
    { 1370, 1090, 820, 660, 440, 390, 330, 230};
// and their inverse: milliGauss per count
const float milligauss_per_count[ 8] =
    { 0.73, 0.917, 1.23, 1.52, 2.27, 2.56, 3.03, 4.35};

/* PUBLIC METHODS */

bb_hmc5883::bb_hmc5883()
{
//  scale[0] = 1.0F;
//  scale[1] = 1.0F;
//  scale[2] = 1.0F;
}

void bb_hmc5883::init()
{
  writeReg(HMC58X3_R_CONFA, 0x74);  // 8 sample average, 30Hz data rate, no bias.
  writeReg(HMC58X3_R_CONFB, 0x00);  // gain = 0 (default = 1)
  writeReg(HMC58X3_R_MODE,  0x00);  // set Mode to Continuous-Measurement                        ///
}

// ==========  Configuration Register A  ==============
// Set samples to average between 0 and 3, 0 is default
// 0 = one sample, 1 = two, 2 = four, 3 = eight samples
void bb_hmc5883::setSamples( uint8_t samples)
{
  if( samples > 3) return;
    {
        printf( "Samples averaged setting %d is too high.\r\n", samples);
        return;
  }
  uint8_t val;
  val = readReg( HMC58X3_R_CONFA);  //  get Register A
  val = val & 0b10011111;           //  use AND to clear Samples bits
  val = val | samples << 5;         //  set samples bits
  writeReg( HMC58X3_R_CONFA, val);  //  replace Reg A
}

// Set data output rate in Continuous Measurement Mode
// to a value between 0 and 6. DOR 4 (15Hz) is default.
void bb_hmc5883::setDOR( uint8_t DOR)
{
  if( DOR > 6) return;
    {
        DEBUG_PRINT( "Data Rate setting %d is too high.\r\n", DOR);
        return;
  }
  uint8_t val;
  val = readReg( HMC58X3_R_CONFA);  //  get Register A
  val = val & 0b11100011;           //  use AND to clear DOR bits
  val = val | DOR << 2;             //  set DOR bits
  writeReg( HMC58X3_R_CONFA, val);  //  replace Reg A
}

// Set Bias to value between 0 and 2, 0 is default
// 0 = no bias, 1 = positive bias, 2 = negative bias
void bb_hmc5883::setBias( uint8_t bias)
{
    if( bias > 2)
    {
          DEBUG_PRINT( "Bias setting %d is too high.\r\n", bias);
          return;
    }
    uint8_t val;
    val = readReg( HMC58X3_R_CONFA);  //  get Register A
    val = val & 0b11111100;           //  use AND to clear last two bits
    val = val | bias;                 //  set bias bits
    writeReg(HMC58X3_R_CONFA, val);
}
//  ------- End of Configuration Register A  --------

//  ======= Configuration Register B ==================
//  Set Gain to value between 0 and 7, 1 is default
void bb_hmc5883::setGain( unsigned char gain)
{
    if( gain > 7)
    {
        DEBUG_PRINT( "Gain setting %d is too high.\r\n", gain);
        return;
    }
    writeReg( HMC58X3_R_CONFB, gain << 5);  //  Set Reg B
    //  Reg B bits 4 - 0 must always be 0 for proper operation.
}

byte bb_hmc5883::getGain()
{
    uint8_t val;
    val = readReg( HMC58X3_R_CONFB) >> 5;
    return val;
}
//  ------- End of Configuration Register B  --------

//  ===============  Mode Register B  ==================
//  Set Measurement Mode to value between 0 and 3, 1 is default
//  0 = Continuous, 1 = Single, 2 & 3 are Idle Mode
void bb_hmc5883::setMode( uint8_t mode)
{
    if( mode > 3)
    {
        DEBUG_PRINT( "Mode setting %d is too high.\r\n", mode);
        return;
    }
    uint8_t val;
    val = readReg( HMC58X3_R_MODE);  //  get Mode Register
    val = val & 0b11111100;          //  Clear last two bits
    val = val | mode;
    writeReg( HMC58X3_R_MODE, val);
}
//  -----------  End of Mode Register B  -------------

//  ===============  Calibration  ==================
/*  Calibrate using the self test operation.
    Average the values using bias mode to obtain the scale factors.

    gain = Gain number (0 - 7) for the sensor.
    n_samples = Number of samples to average
    Returns FALSE if any of the following occurs:
        # Invalid input parameters. (Gain > 7 or n_samples = 0).
        # Any ID register is wrong. (Cannot distinguish between HMC5843 and HMC5883L.)
        # Any reading saturates during positive or negative bias test.
        # Average reading is outside of the expected range for bias test.
*/
void bb_hmc5883::getScale( uint8_t gain, uint8_t n_samples, void* bias)
{
    int xyz[ 3];                          // 16 bit integer values for each axis.
    float xyz_total[ 3] = { 0, 0, 0};     // working floats
    long int pos_total[ 3] = { 0, 0, 0};  // 32 bit totals so they won't overflow.
    long int neg_total[ 3] = { 0, 0, 0};

    //  Check for gain setting of less than 8 and a positive sample setting
    //  And that all responses are within range and no saturation is detected
    bool bret = ( 8 > gain) && ( 0 < n_samples);
    if( !bret) DEBUG_PRINT( "One or more Self Test parameters are not correct.");
    else
    {
        setGain( gain);     //  Set Gain to given calibration value
        printf( "Calibrating with gain of %d.\r\n", gain);

        //  =========  Positive Bias ================
        setBias( 1);        //  Set positive bias current to add ~1.1 Gauss field to each axis.
        DEBUG_PRINT( "Testing positive bias.\r\n");

        //  The first measurement after a gain change maintains the same gain
        //  as the previous setting. The new gain setting is effective from
        //  the second measurement on.
        //  Discard first three measurements
        delay( 134);                            //  Wait fore data ready
        getRaw( &xyz[ 0], &xyz[ 1], &xyz[ 2]);  //  Discard first read
        for( int i = 0; i < n_samples; i++)
        {
            setMode( 1);          // Set to Single Measurement Mode every damn time
            getRaw( &xyz[ 0], &xyz[ 1], &xyz[ 2]);   // Get raw values
            DEBUG_PRINT( "Raw values: X= %d Y= %d Z= %d\r\n", xyz[ 0], xyz[ 1], xyz[ 2]);
            //  Detect saturation: If the ADC overflows or underflows, or a math overflow
            //  occurs during the measurement, the data register will contain -4096.
            //  The data register will clear after the next valid measurement.
            if( xyz[ 0] == -4096 || xyz[ 1] == -4096 || xyz[ 2] == -4096)
            {
                DEBUG_PRINT( "Pos bias test saturated. Increase gain.\r\n");
                bret = false;
                break;
            }
            //  Total the positive bias measurements
            //  to create an average reading over n_samples.
            for( int j = 0; j < 3; j++) pos_total[ j] += xyz[ j];
        }
        DEBUG_PRINT( "Positive totals: X= %ld Y= %ld Z= %ld\r\n", pos_total[0],pos_total[1],pos_total[2]);
        //  - - - - - - - - -  End of Positive Bias  - - - - - - - - - -

        //  ================  Negative Bias  ================
        //  Keep the same Gain and apply Negative Bias.
        setBias( 2);        //  Set negative bias current.
        DEBUG_PRINT( "Testing negative bias.\r\n");
        delay( 134);                            //  Wait fore data ready
        getRaw( &xyz[ 0], &xyz[ 1], &xyz[ 2]);  //  Discard first read
        for( int i = 0; i < n_samples; i++)
        {
            setMode( 1);        //  Set to Single Measurement mode.        
            getRaw( &xyz[ 0], &xyz[ 1], &xyz[ 2]);
            DEBUG_PRINT( "Raw values: X= %d Y= %d Z= %d\r\n", xyz[ 0], xyz[ 1], xyz[ 2]);
            //  Check whether any saturation has occurred
            if( xyz[ 0] == -4096 || xyz[ 1] == -4096 || xyz[ 2] == -4096)
            {
                DEBUG_PRINT( "Neg bias test saturated. Increase gain.\r\n");
                bret = false;
                break;
            }
            //  Total the Negative bias measurements for each axis
            //  Use subtraction to make the total positive.
            for( int j = 0; j < 3; j++) neg_total[ j] -= xyz[ j];
        }
        DEBUG_PRINT( "Negative totals: X= %ld Y= %ld Z= %ld\r\n", neg_total[0],neg_total[1],neg_total[2]);
        //  - - - - - - - - -  End of Negative Bias  - - - - - - - - - -
        
        //  Find nominal limits for the data range
        //  Use the Gain = 5 'counts/Gauss' limits from the Data Sheet.
        //  Adjust the limits for any difference from 5 in Gain setting.
        //  Multiply by the total number of samples in each bias direction( 2 * n_samples ).
        uint32_t doubleSample = ( 2 * n_samples);  // forces math below into long integer framework
        uint16_t low_limit  = doubleSample * counts_per_gauss[ gain] * 243 / 390;
        uint16_t high_limit = doubleSample * counts_per_gauss[ gain] * 575 / 390;
        DEBUG_PRINT( "High Limit: %u Low Limit: %u\r\n", high_limit, low_limit);

        //  Check that all axis totals are within range.
        for( int i = 0; i < 3; i++)
        {
            uint16_t total = pos_total[ i] + neg_total[ i];
            if( ( low_limit > total) || ( total > high_limit))
            {
                 DEBUG_PRINT( "Average data %u out of range.\r\n", total);
                 bret = false;
                 break;
           }
        }

        float test_ref_value = doubleSample * 1.1F * counts_per_gauss[gain];
        //  If all is still okay, create a calibrated scale factor for each axis.
        for( int i = 0; i < 3; i++)
        {
            xyz_total[ i] = test_ref_value / ( pos_total[ i] + neg_total[ i]);
        }
        memcpy( bias, xyz_total, sizeof( xyz_total));

        // reinitialize: 8 sample,15Hz data rate, no bias, gain 1, continuous
        init();
    }
    if( !bret) printf( "Calibration failed.\r\n");
}
//  - - - - - - - - - - - - -  End of Calibration   - - - - - - - - - - - - -

//  Get an offset value for each axis.
void bb_hmc5883::getOffset( int n_samples, void* offset )
{
    printf( "Please rotate platform slowly and completely 2 - 3 times.\r\n");
    float xyz[ 3] = { 0, 0, 0};
    float maxV[ 3] = { 0, 0, 0};
    float minV[ 3] = { 0, 0, 0};

    printf( "Offset sample");
    for( int i = 0; i < n_samples; i++)       //  for 300 times
    {
        // getValues( &xyz[ 0], &xyz[ 1], &xyz[ 2]);  //  get values
        getValue( &xyz);  //  get values
        for( int j = 0; j < 3; j++)
        {
            if( xyz[ j] > 0)
            {
                if( xyz[ j] > maxV[ j]) maxV[ j] = xyz[ j];  //  find the max value
            }
            else
            {
                if( xyz[ j] < minV[ j]) minV[ j] = xyz[ j];  //  find the min value
            }
        }
        printf( ".");
        delay(100);
    }
    printf( "\r\n");
    for( int i = 0; i < 3; i++)
    {
        xyz[ i] = ( maxV[ i] + minV[ i]) / 2;  //  offset = half the difference
    }
    memcpy( offset, xyz, sizeof( xyz));
    printf( "Offset calculation complete.\r\n");
}
//  - - - - - - - - - -   End of getOffset()  - - - - - - - - - - 

//  Write one byte to specified register
//  Using I2C serial protocol 
void bb_hmc5883::writeReg( uint8_t reg, uint8_t val)
{
    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.write( reg);        // send register address
    Wire.write( val);        // send value to write
    Wire.endTransmission();  //end transmission
}

//  Return one byte from specified register
//  Using I2C serial protocol 
byte bb_hmc5883::readReg( uint8_t reg)
{
    byte val;

    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.write( reg);
    Wire.endTransmission();
    Wire.requestFrom( HMC58X3_ADDR, 1);  //  number of bytes requested = 1
    val = Wire.read();
    Wire.endTransmission();

    return val;
}

void bb_hmc5883::getValue( void *value)
{
    float xyz[ 3] = { 0, 0, 0};
    //  Set HMC5883 data address pointer to start at DATA X MSB
    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.write( HMC58X3_R_XM);
    Wire.endTransmission();

    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.requestFrom( HMC58X3_ADDR, 6);
    if( 6 == Wire.available())
    {
        // read out 3 values, 2 bytes each.
        xyz[ 0] = (float)( ( Wire.read() << 8) | Wire.read());
        // NOTE: the Z regs come before the Y regs in the HMC5883L
        xyz[ 2] = (float)( ( Wire.read() << 8) | Wire.read());
        xyz[ 1] = (float)( ( Wire.read() << 8) | Wire.read());
        // HMC5883 data pointer wraps around on the next request
    }
    Wire.endTransmission();
    memcpy( value, xyz, sizeof( xyz));
}

//  Get values as positive or negative integers
void bb_hmc5883::getRaw(int *xr, int *yr, int *zr)
{
    //  Set HMC5883 data address pointer to start at DATA X MSB
    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.write( HMC58X3_R_XM);
    Wire.endTransmission();

    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.requestFrom( HMC58X3_ADDR, 6);
    if( 6 == Wire.available())
    {
        // read out 3 values, 2 bytes each.
        *xr = (Wire.read() << 8) | Wire.read();
        // the Z regs come before the Y regs in HMC5883L
        *zr = (Wire.read() << 8) | Wire.read();
        *yr = (Wire.read() << 8) | Wire.read();
        // HMC5883 data pointer will wrap around on the next request
    }
    Wire.endTransmission();
}

//  Get the values of the three ID registers.
//  Both HMC5843 and HMC5883L have the same
//  'H43' ID register values. Why?
void bb_hmc5883::getID( char id[ 3])
{
    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.write( HMC58X3_R_IDA);             // Start reading from ID Register A.
    Wire.endTransmission();

    Wire.beginTransmission( HMC58X3_ADDR);
    Wire.requestFrom( HMC58X3_ADDR, 3);     //  Request three bytes
    if( 3 == Wire.available())
    {
        id[ 0] = Wire.read();
        id[ 1] = Wire.read();
        id[ 2] = Wire.read();
    }
    else
    {
          id[ 0] = 0;
          id[ 1] = 0;
          id[ 2] = 0;
    }
    Wire.endTransmission();
}
//  -----------  End of getID()  ---------------------

#endif  //  End of #if defined(__AVR_ATmega2560__)