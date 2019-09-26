# NXP Precision on Arduino Due
Arduino library for reading dual Adafruit 9dof NXP Precision breakouts with an Arduino Due.
Based on the Adafruit libraries https://github.com/adafruit/Adafruit_FXOS8700 and https://github.com/adafruit/Adafruit_FXAS21002C. Depends on the unified Adafruit Sensor library.

Note that the chips seem to have an addressing issue that prevents reading any register other than the data-ready status (0x00), so data reads must be performed in burst-read mode.

Included are functions to poll the sensors for new data, and check the data output timing, for synchronisation purposes. Note that different sensor units tend to have some small variability in output data rates, however, so will drift out of sync over time.

The getEvent function has been overloaded to accept a pointer to an IMU data structure, for convenience.

## Usage
Connect each sensor to a separate bus (SDA0/SCL0, SDA1/SCL1) as described here: https://learn.adafruit.com/nxp-precision-9dof-breakout/pinout.

Raw sensor reads can be performed by running: 

```
/examples/dual_bus_read/dual_bus_read.ino 
```
Note that the values provided are raw 16/ 14bit integers.

## Default settings
- 400Hz output data rate, for all components
- Dynamic range 1000DPS for gyroscopes, 4G for accelerometers
- These settings can be changed in the .cpp files

