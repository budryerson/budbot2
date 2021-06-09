/****************************************************************************
* File Name: bb_NAxisMotion.cpp
* Modified by: Bud Ryerson
* Usage: Source file of the C++ Wrapper for the BNO055 Sensor API
* Last Work: 07JUL16 - added axis remap function - line #942
****************************************************************************
* Original: NAxisMotion.cpp rev3.0
* Date: 2015/02/10
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Added Arduino M0/M0 Pro support
* Date: 2015/07/27
* Modified by: Arduino.org development Team.
****************************************************************************/

#include "bb_NAxisMotion.h"

//Function Definitions
/*******************************************************************************************
*Description: Constructor of the class with the default initialization
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
bb_NAxisMotion::bb_NAxisMotion()
{
  //Blank
}

/*******************************************************************************************
*Description: Function with the bare minimum initialization
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::initSensor(unsigned int address)
{
  //Initialize the GPIO peripheral
  pinMode(INT_PIN, INPUT_PULLUP);   //Configure Interrupt pin
  pinMode(RESET_PIN, OUTPUT);     //Configure Reset pin

  //Power on the BNO055
  resetSensor(address);
}

/*******************************************************************************************
*Description: This function is used to reset the BNO055
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::resetSensor(unsigned int address)
{
  //Reset sequence
  digitalWrite(RESET_PIN, LOW);   //Set the Reset pin LOW
  delay(RESET_PERIOD);        //Hold it for a while
  digitalWrite(RESET_PIN, HIGH);    //Set the Reset pin HIGH
  delay(INIT_PERIOD);         //Pause for a while to let the sensor initialize completely (Anything >500ms should be fine)
  //Initialization sequence
  //Link the function pointers for communication (late-binding)
  myBNO.bus_read = BNO055_I2C_bus_read;
  myBNO.bus_write = BNO055_I2C_bus_write;
  myBNO.delay_msec = _delay;

  //Set the I2C address here !!! ADDR1 is the default address
  //myBNO.dev_addr = BNO055_I2C_ADDR1;
  myBNO.dev_addr = address;
  //myBNO.dev_addr = BNO055_I2C_ADDR2;

  //Initialize the BNO055 structure to hold the device information
  bno055_init(&myBNO);

  //Post initialization delay
  delay(POST_INIT_PERIOD);

  //To set the output data format to the Android style
  bno055_set_data_output_format(ANDROID);

  //Set the default data update mode to auto
  dataUpdateMode = AUTO;
}

/*******************************************************************************************
*Description: This function is used to set the operation mode of the BNO055
*Input Parameters:
* byte operationMode: To assign which operation mode the device has to
*       ---------------------------------------------------
*       Constant Definition     Constant Value  Comment
*       ---------------------------------------------------
*       OPERATION_MODE_CONFIG   0x00      Configuration Mode
*                               (Transient Mode)
*       OPERATION_MODE_ACCONLY    0x01      Accelerometer only
*       OPERATION_MODE_MAGONLY    0x02      Magnetometer only
*       OPERATION_MODE_GYRONLY    0x03      Gyroscope only
*       OPERATION_MODE_ACCMAG     0x04      Accelerometer and Magnetometer only
*       OPERATION_MODE_ACCGYRO    0x05      Accelerometer and Gyroscope only
*       OPERATION_MODE_MAGGYRO    0x06      Magnetometer and Gyroscope only
*       OPERATION_MODE_AMG        0x07      Accelerometer, Magnetometer and
*                                           Gyroscope (without fusion)
*       OPERATION_MODE_IMUPLUS    0x08      Inertial Measurement Unit
*                                           (Accelerometer and Gyroscope
*                                            Sensor Fusion Mode)
*       OPERATION_MODE_COMPASS    0x09      Tilt Compensated Compass
*                                           (Accelerometer and Magnetometer
*                                           Sensor Fusion Mode)
*       OPERATION_MODE_M4G        0x0A      Magnetometer and Gyroscope Sensor
*                                            Fusion Mode
*       OPERATION_MODE_NDOF_FMC_OFF 0x0B    9 Degrees of Freedom Sensor Fusion
*                                           with Fast Magnetometer Calibration Off
*       OPERATION_MODE_NDOF       0x0C      9 Degrees of Freedom Sensor Fusion
*
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::setOperationMode(byte operationMode)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_operation_mode(operationMode);      //Set the Operation Mode
}

/*******************************************************************************************
*Description: This function is used to set the power mode
*Input Parameters:
* byte powerMode: To assign the power mode the device has to switch to
*       --------------------------------------
*       Constant Definition   Constant Value
*       --------------------------------------
*       POWER_MODE_NORMAL   0x00
*       POWER_MODE_LOWPOWER   0x01
*       POWER_MODE_SUSPEND    0x02
*Return Parameter:
*******************************************************************************************/
void bb_NAxisMotion::setPowerMode(byte powerMode)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_power_mode(powerMode);          //Set the Power Mode
}

/*******************************************************************************************
*Description: This function is used to update the accelerometer data in m/s2
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateAccel(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_accel_xyz_msq(&accelData);  //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the magnetometer data in microTesla
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateMag(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_mag_xyz_uT(&magData); //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the gyroscope data in deg/s
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateGyro(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_gyro_xyz_dps(&gyroData);    //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the quaternion data
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateQuat(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_read_quaternion_wxyz(&quatData);      //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the euler data in degrees
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateEuler(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_euler_hpr_deg(&eulerData);  //Read the data from the sensor
}


/*******************************************************************************************
*Description: This function is used to update the linear acceleration data in m/s2
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateLinearAccel(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_linear_accel_xyz_msq(&linearAccelData); //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the gravity acceleration data in m/s2
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateGravAccel(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_convert_float_gravity_xyz_msq(&gravAccelData);  //Read the data from the sensor
}

/*******************************************************************************************
*Description: This function is used to update the calibration status
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateCalibStatus(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_get_accel_calib_stat(&calibStatus.accel);
  comRes = bno055_get_mag_calib_stat(&calibStatus.mag);
  comRes = bno055_get_gyro_calib_stat(&calibStatus.gyro);
  comRes = bno055_get_sys_calib_stat(&calibStatus.system);
}

/*******************************************************************************************
*Description: This function is used to write the accelerometer configurations
*Input Parameters:
* uint8_t range: To assign the range of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_RANGE_2G      0X00
*     ACCEL_RANGE_4G      0X01
*     ACCEL_RANGE_8G      0X02
*     ACCEL_RANGE_16G     0X03
* uint8_t bandwidth: To assign the filter bandwidth of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_BW_7_81HZ     0x00
*     ACCEL_BW_15_63HZ    0x01
*     ACCEL_BW_31_25HZ    0x02
*     ACCEL_BW_62_5HZ     0X03
*     ACCEL_BW_125HZ      0X04
*     ACCEL_BW_250HZ      0X05
*     ACCEL_BW_500HZ      0X06
*     ACCEL_BW_1000HZ     0X07
* uint8_t powerMode: To assign the power mode of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_NORMAL      0X00
*     ACCEL_SUSPEND     0X01
*     ACCEL_LOWPOWER_1    0X02
*     ACCEL_STANDBY     0X03
*     ACCEL_LOWPOWER_2    0X04
*     ACCEL_DEEPSUSPEND   0X05
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::writeAccelConfig(uint8_t range, uint8_t bandwidth, uint8_t powerMode)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_accel_range(range);
  comRes = bno055_set_accel_bw(bandwidth);
  comRes = bno055_set_accel_power_mode(powerMode);
}

/*******************************************************************************************
*Description: This function is used to update the accelerometer configurations
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::updateAccelConfig(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_get_accel_range(&accelStatus.range);
  comRes = bno055_get_accel_bw(&accelStatus.bandwidth);
  comRes = bno055_get_accel_power_mode(&accelStatus.powerMode);
}

/*******************************************************************************************
*Description: This function is used to control which axis of the accelerometer triggers the
*       interrupt
*Input Parameters:
* bool xStatus: To know whether the x axis has to trigger the interrupt
*       ---------------------------------------------------
*       Constant Definition   Constant Value  Comment
*       ---------------------------------------------------
*       ENABLE          1       Enables interrupts from that axis
*       DISABLE         0       Disables interrupts from that axis
* bool yStatus: To know whether the x axis has to trigger the interrupt
*       ---------------------------------------------------
*       Constant Definition   Constant Value  Comment
*       ---------------------------------------------------
*       ENABLE          1       Enables interrupts from that axis
*       DISABLE         0       Disables interrupts from that axis
* bool zStatus: To know whether the x axis has to trigger the interrupt
*       ---------------------------------------------------
*       Constant Definition   Constant Value  Comment
*       ---------------------------------------------------
*       ENABLE          1       Enables interrupts from that axis
*       DISABLE         0       Disables interrupts from that axis
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::accelInterrupts(bool xStatus, bool yStatus, bool zStatus)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS, xStatus);
  comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS, yStatus);
  comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS, zStatus);
}

/*******************************************************************************************
*Description: This function is used to reset the interrupt line
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::resetInterrupt(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_intr_rst(ENABLE);
}

/*******************************************************************************************
*Description: This function is used to enable the any motion interrupt based on the
*       accelerometer
*Input Parameters:
* uint8_t threshold: The threshold that triggers the any motion interrupt
*       The threshold should be entered as an integer. The corresponding value of
*         the threshold depends on the range that has been set on the
*         accelerometer. Below is a table showing the value of 1LSB in
*         corresponding units.
*       Resolution:
*         ACCEL_RANGE_2G, 1LSB = 3.91mg = ~0.03835m/s2
*         ACCEL_RANGE_4G, 1LSB = 7.81mg = ~0.07661m/s2
*         ACCEL_RANGE_8G, 1LSB = 15.6mg = ~0.15303m/s2
*         ACCEL_RANGE_16G, 1LSB = 31.3mg = ~0.30705m/s2
*       Maximum:
*         ACCEL_RANGE_2G, 1LSB = 996mg = ~9.77076m/s2,
*         ACCEL_RANGE_4G, 1LSB = 1.99g = ~19.5219m/s2
*         ACCEL_RANGE_8G, 1LSB = 3.98g = ~39.0438m/s2
*         ACCEL_RANGE_16G, 1LSB = 7.97g = ~97.1857m/s2
* uint8_t duration: The duration for which the desired threshold exist
*       The time difference between the successive acceleration signals depends
*       on the selected bandwidth and equates to 1/(2*bandwidth).
*       In order to suppress false triggers, the interrupt is only generated (cleared)
*       if a certain number N of consecutive slope data points is larger (smaller)
*       than the slope 'threshold'. This number is set by the 'duration'.
*       It is N = duration + 1.
*       Resolution:
*         ACCEL_BW_7_81HZ, 1LSB = 64ms
*         ACCEL_BW_15_63HZ, 1LSB = 32ms
*         ACCEL_BW_31_25HZ, 1LSB = 16ms
*         ACCEL_BW_62_5HZ, 1LSB = 8ms
*         ACCEL_BW_125HZ, 1LSB = 4ms
*         ACCEL_BW_250HZ, 1LSB = 2ms
*         ACCEL_BW_500HZ, 1LSB = 1ms
*         ACCEL_BW_1000HZ, 1LSB = 0.5ms
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::enableAnyMotion(uint8_t threshold, uint8_t duration)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_accel_any_motion_thres(threshold);
  comRes = bno055_set_accel_any_motion_durn(duration);
  comRes = bno055_set_intr_accel_any_motion(ENABLE);
  comRes = bno055_set_intr_mask_accel_any_motion(ENABLE);
}

/*******************************************************************************************
*Description: This function is used to disable the any motion interrupt
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::disableAnyMotion(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_intr_accel_any_motion(DISABLE);
  comRes = bno055_set_intr_mask_accel_any_motion(DISABLE);
}

/*******************************************************************************************
*Description: This function is used to enable the slow or no motion interrupt based on the
*       accelerometer
*Input Parameters:
* uint8_t threshold: The threshold that triggers the no motion interrupt
*       The threshold should be entered as an integer. The corresponding value of
*         the threshold depends on the range that has been set on the
*         accelerometer. Below is a table showing the value of 1LSB in
*         corresponding units.
*       Resolution:
*         ACCEL_RANGE_2G, 1LSB = 3.91mg = ~0.03835m/s2
*         ACCEL_RANGE_4G, 1LSB = 7.81mg = ~0.07661m/s2
*         ACCEL_RANGE_8G, 1LSB = 15.6mg = ~0.15303m/s2
*         ACCEL_RANGE_16G, 1LSB = 31.3mg = ~0.30705m/s2
*       Maximum:
*         ACCEL_RANGE_2G, 1LSB = 996mg = ~9.77076m/s2,
*         ACCEL_RANGE_4G, 1LSB = 1.99g = ~19.5219m/s2
*         ACCEL_RANGE_8G, 1LSB = 3.98g = ~39.0438m/s2
*         ACCEL_RANGE_16G, 1LSB = 7.97g = ~97.1857m/s2
* uint8_t duration: The duration for which the desired threshold should be surpassed
*       The time difference between the successive acceleration signals depends
*       on the selected bandwidth and equates to 1/(2*bandwidth).
*       In order to suppress false triggers, the interrupt is only generated (cleared)
*       if a certain number N of consecutive slope data points is larger (smaller)
*       than the slope 'threshold'. This number is set by the 'duration'.
*       It is N = duration + 1.
*       Resolution:
*         ACCEL_BW_7_81HZ, 1LSB = 64ms
*         ACCEL_BW_15_63HZ, 1LSB = 32ms
*         ACCEL_BW_31_25HZ, 1LSB = 16ms
*         ACCEL_BW_62_5HZ, 1LSB = 8ms
*         ACCEL_BW_125HZ, 1LSB = 4ms
*         ACCEL_BW_250HZ, 1LSB = 2ms
*         ACCEL_BW_500HZ, 1LSB = 1ms
*         ACCEL_BW_1000HZ, 1LSB = 0.5ms
* bool motion: To trigger either a Slow motion or a No motion interrupt
*       ---------------------------------------------------
*       Constant Definition   Constant Value  Comment
*       ---------------------------------------------------
*       NO_MOTION       1       Enables the no motion interrupt
*       SLOW_MOTION       0       Enables the slow motion interrupt
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::enableSlowNoMotion(uint8_t threshold, uint8_t duration, bool motion)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_accel_slow_no_motion_enable(motion);
  comRes = bno055_set_accel_slow_no_motion_thres(threshold);
  comRes = bno055_set_accel_slow_no_motion_durn(duration);
  comRes = bno055_set_intr_accel_no_motion(ENABLE);
  comRes = bno055_set_intr_mask_accel_no_motion(ENABLE);
}

/*******************************************************************************************
*Description: This function is used to disable the slow or no motion interrupt
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::disableSlowNoMotion(void)
{
  BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;   //Holds the communication results
  comRes = bno055_set_intr_accel_no_motion(DISABLE);
  comRes = bno055_set_intr_mask_accel_any_motion(DISABLE);
}

/*******************************************************************************************
*Description: This function is used to change the mode of updating the local data
*Input Parameters: None
*Return Parameter: None
*******************************************************************************************/
void bb_NAxisMotion::setUpdateMode(bool updateMode)
{
  dataUpdateMode = updateMode;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the accelerometer data
*Input Parameters: None
*Return Parameter:
* float:  X-axis accelerometer data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readAccelX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccel();
  }
  return accelData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the accelerometer data
*Input Parameters: None
*Return Parameter:
* float:  Y-axis accelerometer data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readAccelY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccel();
  }
  return accelData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the accelerometer data
*Input Parameters: None
*Return Parameter:
* float:  Z-axis accelerometer data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readAccelZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccel();
  }
  return accelData.z;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the gyroscope data
*Input Parameters: None
*Return Parameter:
* float:  X-axis gyroscope data in deg/s
*******************************************************************************************/
float bb_NAxisMotion::readGyroX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGyro();
  }
  return gyroData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the gyroscope data
*Input Parameters: None
*Return Parameter:
* float:  Y-axis gyroscope data in deg/s
*******************************************************************************************/
float bb_NAxisMotion::readGyroY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGyro();
  }
  return gyroData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the gyroscope data
*Input Parameters: None
*Return Parameter:
* float:  Z-axis gyroscope data in deg/s
*******************************************************************************************/
float bb_NAxisMotion::readGyroZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGyro();
  }
  return gyroData.z;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the magnetometer data
*Input Parameters: None
*Return Parameter:
* float:  X-axis magnetometer data in µT
*******************************************************************************************/
float bb_NAxisMotion::readMagX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateMag();
  }
  return magData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the magnetometer data
*Input Parameters: None
*Return Parameter:
* float:  Y-axis magnetometer data in µT
*******************************************************************************************/
float bb_NAxisMotion::readMagY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateMag();
  }
  return magData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the magnetometer data
*Input Parameters: None
*Return Parameter:
* float:  Z-axis magnetometer data in µT
*******************************************************************************************/
float bb_NAxisMotion::readMagZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateMag();
  }
  return magData.z;
}

/*******************************************************************************************
*Description: This function is used to return the w-axis of the quaternion data
*Input Parameters: None
*Return Parameter:
* int16_t:  W-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
*******************************************************************************************/
int16_t bb_NAxisMotion::readQuatW(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateQuat();
  }
  return quatData.w;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the quaternion data
*Input Parameters: None
*Return Parameter:
* int16_t:  X-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
*******************************************************************************************/
int16_t bb_NAxisMotion::readQuatX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateQuat();
  }
  return quatData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the quaternion data
*Input Parameters: None
*Return Parameter:
* int16_t:  Y-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
*******************************************************************************************/
int16_t bb_NAxisMotion::readQuatY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateQuat();
  }
  return quatData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the quaternion data
*Input Parameters: None
*Return Parameter:
* int16_t:  Z-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
*******************************************************************************************/
int16_t bb_NAxisMotion::readQuatZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateQuat();
  }
  return quatData.z;
}

/*******************************************************************************************
*Description: This function is used to return the heading(yaw) of the euler data
*Input Parameters: None
*Return Parameter:
* float:  Heading of the euler data
*******************************************************************************************/
float bb_NAxisMotion::readEulerHeading(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateEuler();
  }
  return eulerData.h;
}

/*******************************************************************************************
*Description: This function is used to return the roll of the euler data
*Input Parameters: None
*Return Parameter:
* float:  Roll of the euler data
*******************************************************************************************/
float bb_NAxisMotion::readEulerRoll(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateEuler();
  }
  return eulerData.r;
}

/*******************************************************************************************
*Description: This function is used to return the pitch of the euler data
*Input Parameters: None
*Return Parameter:
* float:  Pitch of the euler data
*******************************************************************************************/
float bb_NAxisMotion::readEulerPitch(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateEuler();
  }
  return eulerData.p;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the linear acceleration data
*         (accelerometer data without the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  X-axis Linear Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readLinearAccelX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateLinearAccel();
  }
  return linearAccelData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the linear acceleration data
*         (accelerometer data without the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  Y-axis Linear Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readLinearAccelY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateLinearAccel();
  }
  return linearAccelData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the linear acceleration data
*         (accelerometer data without the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  Z-axis Linear Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readLinearAccelZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateLinearAccel();
  }
  return linearAccelData.z;
}

/*******************************************************************************************
*Description: This function is used to return the x-axis of the gravity acceleration data
*         (accelerometer data with only the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  X-axis Gravity Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readGravAccelX(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGravAccel();
  }
  return gravAccelData.x;
}

/*******************************************************************************************
*Description: This function is used to return the y-axis of the gravity acceleration data
*         (accelerometer data with only the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  Y-axis Gravity Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readGravAccelY(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGravAccel();
  }
  return gravAccelData.y;
}

/*******************************************************************************************
*Description: This function is used to return the z-axis of the gravity
*             acceleration data (accelerometer data with only the gravity vector)
*Input Parameters: None
*Return Parameter:
* float:  Z-axis Gravity Acceleration data in m/s2
*******************************************************************************************/
float bb_NAxisMotion::readGravAccelZ(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateGravAccel();
  }
  return gravAccelData.z;
}

/*******************************************************************************************
*Description: This function is used to return the accelerometer calibration status
*Input Parameters: None
*Return Parameter:
* uint8_t:  Accelerometer calibration status, 0-3 (0 - low, 3 - high)
*******************************************************************************************/
uint8_t bb_NAxisMotion::readAccelCalibStatus(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateCalibStatus();
  }
  return calibStatus.accel;
}

/*******************************************************************************************
*Description: This function is used to return the gyroscope calibration status
*Input Parameters: None
*Return Parameter:
* uint8_t:  Gyroscope calibration status, 0-3 (0 - low, 3 - high)
*******************************************************************************************/
uint8_t bb_NAxisMotion::readGyroCalibStatus(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateCalibStatus();
  }
  return calibStatus.gyro;
}

/********************************************************************************
*Description: This function is used to return the magnetometer calibration status
*Input Parameters: None
*Return Parameter:
* uint8_t:  Magnetometer calibration status, 0-3 (0 - low, 3 - high)
********************************************************************************/
uint8_t bb_NAxisMotion::readMagCalibStatus(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateCalibStatus();
  }
  return calibStatus.mag;
}

/********************************************************************************
*Description: This function is used to return the system calibration status
*Input Parameters: None
*Return Parameter:
* uint8_t:  System calibration status, 0-3 (0 - low, 3 - high)
********************************************************************************/
uint8_t bb_NAxisMotion::readSystemCalibStatus(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateCalibStatus();
  }
  return calibStatus.system;
}

/*******************************************************************************************
*Description: This function is used to return the accelerometer range
*Input Parameters: None
*Return Parameter:
* uint8_t range: Range of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_RANGE_2G      0X00
*     ACCEL_RANGE_4G      0X01
*     ACCEL_RANGE_8G      0X02
*     ACCEL_RANGE_16G     0X03
********************************************************************************/
uint8_t bb_NAxisMotion::readAccelRange(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccelConfig();
  }
  return accelStatus.range;
}

/*******************************************************************************************
*Description: This function is used to return the accelerometer bandwidth
*Input Parameters: None
*Return Parameter:
* uint8_t bandwidth: Bandwidth of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_BW_7_81HZ     0x00
*     ACCEL_BW_15_63HZ    0x01
*     ACCEL_BW_31_25HZ    0x02
*     ACCEL_BW_62_5HZ     0X03
*     ACCEL_BW_125HZ      0X04
*     ACCEL_BW_250HZ      0X05
*     ACCEL_BW_500HZ      0X06
*     ACCEL_BW_1000HZ     0X07
********************************************************************************/
uint8_t bb_NAxisMotion::readAccelBandwidth(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccelConfig();
  }
  return accelStatus.bandwidth;
}

/*******************************************************************************
*Description: This function is used to return the accelerometer power mode
*Input Parameters: None
*Return Parameter:
* uint8_t powerMode: Power mode of the accelerometer
*     --------------------------------------
*     Constant Definition   Constant Value
*     --------------------------------------
*     ACCEL_NORMAL      0X00
*     ACCEL_SUSPEND     0X01
*     ACCEL_LOWPOWER_1    0X02
*     ACCEL_STANDBY     0X03
*     ACCEL_LOWPOWER_2    0X04
*     ACCEL_DEEPSUSPEND   0X05
********************************************************************************/
uint8_t bb_NAxisMotion::readAccelPowerMode(void)
{
  if (dataUpdateMode == AUTO)
  {
    updateAccelConfig();
  }
  return accelStatus.powerMode;
}

/************************************************************
* Remap Axises and Signs to correct for IMU position
* So-called position P0 = Swap X/Y and set X negative
* Calls the BNO055 library functions directly
*
*     value     |   definition      | comments
*   ------------|-------------------|------------
*      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
*      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
*      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
*      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
*      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
*      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
*
/*************************************************************/
void bb_NAxisMotion::remapAxisPosition()
{
    // Each operation sets Operation Mode to CONFIG to work;
    // but also saves and restores the current Operation Mode.
    bno055_set_axis_remap_value( REMAP_X_Y);              // 0x21 = REMAP_X_Y
    bno055_set_remap_x_sign( REMAP_AXIS_NEGATIVE);        // 0x01 = REMAP_AXIS_NEGATIVE
    bno055_set_remap_y_sign( REMAP_AXIS_POSITIVE);        // 0x00 = REMAP_AXIS_POSITIVE
    delay(30);
}

/*******   Bridge Functions for the Sensor API to control the Arduino Hardware   ********/
signed char BNO055_I2C_bus_read( unsigned char dev_addr,  unsigned char reg_addr,
                                 unsigned char *reg_data, unsigned char cnt)
{
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
  I2C.beginTransmission( dev_addr); // Start of transmission
  I2C.write( reg_addr);             // Desired start register
  comres = I2C.endTransmission();   // Stop of transmission
  delayMicroseconds(150);           // Caution Delay
  I2C.requestFrom( dev_addr, cnt);  // Request data
  while( I2C.available())           // The slave device may send less than requested (burst read)
  {
    *reg_data = I2C.read();         // Receive a byte
    reg_data++;                     // Increment pointer
  }
  return comres;
}

signed char BNO055_I2C_bus_write( unsigned char dev_addr,  unsigned char reg_addr,
                                  unsigned char *reg_data, unsigned char cnt)
{
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
  I2C.beginTransmission( dev_addr); // Start transmission
  I2C.write( reg_addr);             // Desired start register
  for(unsigned char index = 0; index < cnt; index++) // Note that the BNO055 supports burst write
  {
    I2C.write( *reg_data);          // Write the data
    reg_data++;                     // Increment pointer
  }
  comres = I2C.endTransmission();   // Stop transmission
  delayMicroseconds( 150);          // Caution Delay
  return comres;
}

void _delay(u_32 period)
{
  delay( period);
}