# NXP Precision on Arduino Due

Arduino library for reading dual Adafruit 9dof NXP Precision breakouts with an Arduino Due.
Based on the Adafruit libraries https://github.com/adafruit/Adafruit_FXOS8700 and https://github.com/adafruit/Adafruit_FXAS21002C.

## Usage
Connect each sensor to a separate bus (SDA0/SCL0, SDA1/SCL1) as described here: https://learn.adafruit.com/nxp-precision-9dof-breakout/pinout.
Run /examples/dual_bus_read/dual_bus_read.ino to read raw sensor data. 
Note that the values provided are raw 16/ 14bit integer values.

## Default settings
- 400Hz output data rate, for all components
- Dynamic range 1000DPS for gyroscopes, 4G for accelerometers
- These settings can be changed in the .cpp files

