 #include "Arduino.h"

#include <Wire.h>
#include <limits.h>

#include "AccMag.h"

#define ACCEL_MG_LSB_2G (0.000244F)
#define ACCEL_MG_LSB_4G (0.000488F)
#define ACCEL_MG_LSB_8G (0.000976F)
#define MAG_UT_LSB      (0.1F)

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

void AccMag::write8(byte reg, byte value)
{
  _wire.beginTransmission(FXOS8700_ADDRESS);
  _wire.write((uint8_t)reg);
  _wire.write((uint8_t)value);

  _wire.endTransmission();
}

byte AccMag::read8(byte reg)
{
  byte value;

  _wire.beginTransmission((byte)FXOS8700_ADDRESS);
  _wire.write(reg);
  _wire.endTransmission();
  _wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)1);
  value = _wire.read();


  return value;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

byte AccMag::checkstatus()

// check the data-ready status
// 0 = no data available
// 15 = data ready
// 255 = overwrite occurrred

{
  return read8(FXOS8700_REGISTER_STATUS);
}

bool AccMag::checktiming()

  // The sensor has a struct var containing two times corresponding to ODRs.
  // Get these by continuously polling the registers for fresh data. Throw away the first packet, then get the timings on arrival of packet 2 and 3
  // If in between we get a 255 (data overwritten) then start again
  // These values should be separated by 1/ODR
  // In between polls, read the data registers to reset the data-ready register
  // Note that the times returned are accurate only to within the duration of the function (109us for accmag0, 120 for accmag1) - so the real times may be 120us less

{
  byte value;
  int count = 0;
  int i = 0;

  unsigned long int t = micros();

  while (true){

    unsigned long int start_t = micros();

    // If this takes >1s, it means the sensors are broken: return false
    if (start_t - t >1000000){
      return false;
    }

    value = checkstatus();

    switch (value){

      default: timings.looptimes[i] = micros() - start_t;
      i ++;
      continue;

      case 15: 
      
      if (count==1){
        timings.looptimes[i] = micros() - start_t;
        i ++;

        timings.first_t = micros();
      }
      if (count==2){
        timings.second_t = micros();
        return true;
      }

      count ++;
      break;

      case 255: count = 0;
      break;

    }

    _wire.beginTransmission((byte)FXOS8700_ADDRESS);
        _wire.write(FXOS8700_REGISTER_STATUS | 0x80); 
        _wire.endTransmission();
        _wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)13);
        uint8_t status = _wire.read();
        uint8_t axhi = _wire.read();
        uint8_t axlo = _wire.read();
        uint8_t ayhi = _wire.read();
        uint8_t aylo = _wire.read();
        uint8_t azhi = _wire.read();
        uint8_t azlo = _wire.read();
        uint8_t mxhi = _wire.read();
        uint8_t mxlo = _wire.read();
        uint8_t myhi = _wire.read();
        uint8_t mylo = _wire.read();
        uint8_t mzhi = _wire.read();
        uint8_t mzlo = _wire.read();
  }
}

bool AccMag::begin(fxos8700AccelRange_t rng)
{
  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  /* Addressing is broken - can only read starting at 0x00. So omit ID check */
  // int16_t id = read8(FXOS8700_REGISTER_WHO_AM_I);
  // if (id != FXOS8700_ID) //should be 199
  // {return false;}

  /* Set to standby mode (required to make changes to this register) */
  write8(FXOS8700_REGISTER_CTRL_REG1, 0);

  /* Configure the accelerometer */
  switch (_range) {
      case (ACCEL_RANGE_2G):
        write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
      break;
      case (ACCEL_RANGE_4G):
        write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01);
      break;
      case (ACCEL_RANGE_8G):
        write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02);
      break;
  }

  // Accelerometer settings // 
  /* High resolution mode*/
  write8(FXOS8700_REGISTER_CTRL_REG2, 0x02);
  /* Active, Normal Mode, Low Noise, 400Hz in Hybrid Mode */
  write8(FXOS8700_REGISTER_CTRL_REG1, 0x05); //00000101
  /* Magnetometer settings */
  /* Hybrid Mode, Over Sampling Rate = 16 */
  write8(FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
  /* Jump to reg 0x33 after reading 0x06 (auto-increment from accelerometer to mag during burst read)*/
  write8(FXOS8700_REGISTER_MCTRL_REG2, 0x20);



  return true;
}

/**************************************************************************/
/*!
 Read the sensor
*/
/**************************************************************************/
bool AccMag::getEvent(sensors_event_t* accelEvent, sensors_event_t* magEvent)
{
  /* Clear the event */
  memset(accelEvent, 0, sizeof(sensors_event_t));
  memset(magEvent, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  /* Check if there is new data available */
  // If full sample is not available, exit without going further
  accel_raw.status = checkstatus();
  if(accel_raw.status <15){
    return false;
  }

  /* Data available. Set the timestamps */
  accelEvent->timestamp = micros();

  // For some reason, we have to restart the teansmission at the status register.
  // Restarting at the first data register gives bad values //
  _wire.beginTransmission((byte)FXOS8700_ADDRESS);
  _wire.write(FXOS8700_REGISTER_STATUS); 
  _wire.endTransmission();
  _wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)13);

  accel_raw.status = _wire.read();

  uint8_t axhi = _wire.read();
  uint8_t axlo = _wire.read();
  uint8_t ayhi = _wire.read();
  uint8_t aylo = _wire.read();
  uint8_t azhi = _wire.read();
  uint8_t azlo = _wire.read();
  uint8_t mxhi = _wire.read();
  uint8_t mxlo = _wire.read();
  uint8_t myhi = _wire.read();
  uint8_t mylo = _wire.read();
  uint8_t mzhi = _wire.read();
  uint8_t mzlo = _wire.read();

  /* Shift values to create properly formed integers */
  accel_raw.x = (int16_t)((axhi << 8) | axlo) >> 2;
  accel_raw.y = (int16_t)((ayhi << 8) | aylo) >> 2;
  accel_raw.z = (int16_t)((azhi << 8) | azlo) >> 2;
  mag_raw.x = (int16_t)((mxhi << 8) | mxlo);
  mag_raw.y = (int16_t)((myhi << 8) | mylo);
  mag_raw.z = (int16_t)((mzhi << 8) | mzlo);

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void  AccMag::getSensor(sensor_t* accelSensor, sensor_t* magSensor)
{
  /* Clear the sensor_t object */
  memset(accelSensor, 0, sizeof(sensor_t));
  memset(magSensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (accelSensor->name, "FXOS8700", sizeof(accelSensor->name) - 1);
  accelSensor->name[sizeof(accelSensor->name) - 1] = 0;
  accelSensor->version     = 1;
  accelSensor->sensor_id   = _accelSensorID;
  accelSensor->type        = SENSOR_TYPE_ACCELEROMETER;
  accelSensor->min_delay   = 0.01F; // 100Hz
  switch (_range) {
      case (ACCEL_RANGE_2G):
          accelSensor->max_value   = 2.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -1.999F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_4G):
          accelSensor->max_value   = 4.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -3.998F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ACCEL_RANGE_8G):
          accelSensor->max_value   = 8.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -7.996F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
  }

  strncpy (magSensor->name, "FXOS8700", sizeof(magSensor->name) - 1);
  magSensor->name[sizeof(magSensor->name) - 1] = 0;
  magSensor->version     = 1;
  magSensor->sensor_id   = _magSensorID;
  magSensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  magSensor->min_delay   = 0.01F; // 100Hz
  magSensor->max_value   = 1200.0F;
  magSensor->min_value   = -1200.0F;
  magSensor->resolution  = 0.1F;
}

/* To keep Adafruit_Sensor happy we need a single sensor interface */
/* When only one sensor is requested, return accel data */
bool AccMag::getEvent(sensors_event_t* accelEvent)
{
    sensors_event_t mag;

    return getEvent(accelEvent, &mag);
}

/* To keep Adafruit_Sensor happy we need a single sensor interface */
/* When only one sensor is requested, return accel data */
void  AccMag::getSensor(sensor_t* accelSensor)
{
    sensor_t mag;

    return getSensor(accelSensor, &mag);
}
