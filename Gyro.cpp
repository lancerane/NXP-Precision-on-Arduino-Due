
#include "Arduino.h"
#include <Wire.h>
#include <limits.h>

#include "Gyro.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

void Gyro::write8(byte reg, byte value)
{
  _wire.beginTransmission(FXAS21002C_ADDRESS);
  _wire.write((uint8_t)reg);
  _wire.write((uint8_t)value);
  _wire.endTransmission();
}

byte Gyro::read8(byte reg)
{
  uint8_t value;

  _wire.beginTransmission((byte)FXAS21002C_ADDRESS);
  _wire.write(reg);
  _wire.endTransmission();
  _wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)1);
  value = _wire.read();

  return value;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
byte Gyro::checkstatus()

// 0 = no data available
// >0, <15 = part data ready
// 15 = data ready
// 255 = overwrite occurrred

{
  return read8(GYRO_REGISTER_STATUS);
  // uint8_t value;

  // _wire.beginTransmission((byte)FXAS21002C_ADDRESS);
  // _wire.write(GYRO_REGISTER_STATUS); 
  // _wire.endTransmission();
  // _wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)1);
  // value = _wire.read();

  // return value;
}

bool Gyro::checktiming()

  // The sensor has a struct var containing two times corresponding to ODRs.
  // Get these by continuously polling the registers for fresh data. Throw away the first packet, then get the timings on arrival of packet 2 and 3
  // In between polls, read the data registers to reset the data-ready register
  // Note that the times returned are accurate only to within the duration of the function (109us for accmag0, 120 for gyro1) - so the real times may be 120us less

{

  byte value;
  int count = 0;
  int i = 0;
  unsigned long int t = micros();

  while (true){

    unsigned long int start_t = micros();
    if (start_t - t >1000000){ //1s
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
        _wire.beginTransmission((byte)FXAS21002C_ADDRESS);
        _wire.write(GYRO_REGISTER_STATUS | 0x80); 
        _wire.endTransmission();
        _wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)7);
        uint8_t status = _wire.read();
        uint8_t xhi = _wire.read();
        uint8_t xlo = _wire.read();
        uint8_t yhi = _wire.read();
        uint8_t ylo = _wire.read();
        uint8_t zhi = _wire.read();
        uint8_t zlo = _wire.read();

        // shouldnt need the end transmission
        // _wire.endTransmission();

  }
}

bool Gyro::begin(gyroRange_t rng)
{
  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  /* Addressing is broken - can only read starting at 0x00. So omit ID check  */
  // uint8_t id = read8((uint8_t)GYRO_REGISTER_WHO_AM_I );
  // if (id != FXAS21002C_ID){return false;} //215

  // Sensor config. See datasheet for options // 
  uint8_t ctrlReg0 = 0x00;

  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      ctrlReg0 = 0x03;
      break;
    case GYRO_RANGE_500DPS:
     ctrlReg0 = 0x02;
      break;
    case GYRO_RANGE_1000DPS:
      ctrlReg0 = 0x01;
      break;
    case GYRO_RANGE_2000DPS:
      ctrlReg0 = 0x00;
      break;
  }

  /* Reset then switch to active mode with 400Hz output */
  write8(GYRO_REGISTER_CTRL_REG1, 0x00);     // Standby
  write8(GYRO_REGISTER_CTRL_REG1, (1<<6));   // Reset

  write8(GYRO_REGISTER_CTRL_REG0, ctrlReg0); // Set sensitivity
  // write8(GYRO_REGISTER_CTRL_REG1, 0x0E);     // Active, 100Hz
  write8(GYRO_REGISTER_CTRL_REG1, 0x06);     // Active, 400Hz

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Gyro::getEvent(sensors_event_t* event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;
  
  /* Check if there is new data available */
  raw.status = checkstatus(); 

  // If full sample is not available, exit without going further
  if(raw.status <15){
    return false;
  }

  /* Data available. Set the timestamp */
  event->timestamp = micros();

  // For some reason, we have to restart the transmission at the status register. Restarting at the first data register gives bad values //
  _wire.beginTransmission((byte)FXAS21002C_ADDRESS);
  _wire.write(GYRO_REGISTER_STATUS); 
  _wire.endTransmission();
  _wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)7);

  raw.status = _wire.read();
  uint8_t xhi = _wire.read();
  uint8_t xlo = _wire.read();
  uint8_t yhi = _wire.read();
  uint8_t ylo = _wire.read();
  uint8_t zhi = _wire.read();
  uint8_t zlo = _wire.read();

  /* Shift values to create properly formed integer */
  raw.x = (int16_t)((xhi << 8) | xlo);
  raw.y = (int16_t)((yhi << 8) | ylo);
  raw.z = (int16_t)((zhi << 8) | zlo);

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data

    @param[out] sensor
                A reference to the sensor_t instances where the
                gyroscope sensor info should be written.
*/
/**************************************************************************/
void  Gyro::getSensor(sensor_t* sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "FXAS21002C", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = (float)this->_range * SENSORS_DPS_TO_RADS;
  sensor->min_value   = (this->_range * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution  = 0.0F; // TBD
}
