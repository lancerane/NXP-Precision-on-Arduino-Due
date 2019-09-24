#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <AccMag.h>
#include <Gyro.h>

//Defines so the device can do a self reset
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

/* Create the sensor objects, specifying a unique ID and the bus */
AccMag accelmag0 = AccMag(0x8700A0, 0x8700B0, 0);
AccMag accelmag1 = AccMag(0x8700A1, 0x8700B1, 1);

Gyro gyro0 = Gyro(0x0021002C0, 0);
Gyro gyro1 = Gyro(0x0021002C1, 1);

void displaySensorDetails(void)
{
  sensor_t sensor;
  gyro0.getSensor(&sensor);
  sensor_t accel, mag;
  accelmag0.getSensor(&accel, &mag);
  
  Serial.println("------------------------------------");
  Serial.println("GYRO");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(sensor.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(mag.name);
  Serial.print  ("Driver Ver:   "); Serial.println(mag.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(mag.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(mag.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(mag.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(mag.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);

}

void eraseDataRegisters(void)
{
  byte status_arr[4];

  while (true){


  sensors_event_t event;
  sensors_event_t aevent, mevent;
  sensors_event_t event1;
  sensors_event_t aevent1, mevent1;
  
  gyro0.getEvent(&event);  
  accelmag0.getEvent(&aevent, &mevent);
  gyro1.getEvent(&event1);
  accelmag1.getEvent(&aevent1, &mevent1);

  status_arr[0] = gyro0.raw.status;
  status_arr[1] = gyro1.raw.status;
  status_arr[2] = accelmag0.accel_raw.status;
  status_arr[3] = accelmag1.accel_raw.status;


  if (status_arr[0] == 0 && status_arr[1] == 0 && status_arr[2] == 0 && status_arr[3] == 0){
      break;

    }
  }
  
}

void setup(void)
{
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  Serial.println("...");

  Wire.begin();
  Wire1.begin();

  // Max the I2C clocks 
  Wire.setClock(400000);
  Wire1.setClock(400000);

  /* Initialise the sensors */
  accelmag1.begin(ACCEL_RANGE_4G); 
  delayMicroseconds(4200); 
  accelmag0.begin(ACCEL_RANGE_4G); 

  gyro0.begin(GYRO_RANGE_1000DPS);
  gyro1.begin(GYRO_RANGE_1000DPS); 

  delayMicroseconds(10000);

  accelmag1.checktiming();
  accelmag0.checktiming();

//  for (int i=0; i<80; i++){
//  Serial.println(accelmag0.timings.looptimes[i]);
//  }
//  

  // ASSERT THAT ACC/MAG TIMINGS ARE AS EXPECTED
  while (accelmag0.timings.first_t > accelmag1.timings.first_t){
    accelmag0.timings.first_t -=2500;
  }

  const int offset_tolerance = 2500;

  if (abs(accelmag1.timings.first_t - accelmag0.timings.first_t) > offset_tolerance){
      REQUEST_EXTERNAL_RESET;
    }

  gyro0.checktiming();
  gyro1.checktiming();


  // Get gyro read times within 2500us of acc1
  while (gyro0.timings.first_t > accelmag1.timings.first_t){
    gyro0.timings.first_t -=2500;
  }
  while (gyro1.timings.first_t > accelmag1.timings.first_t){
    gyro1.timings.first_t -=2500;
  }

  unsigned long int offset0;
  unsigned long int offset1;
  offset0 = accelmag1.timings.first_t - gyro0.timings.first_t;
  offset1 = accelmag1.timings.first_t - gyro1.timings.first_t;
  
  // If the gyros aren't sync'd with the acc/mags to within a certain tolerance, reset
  if (offset0 > offset_tolerance || offset1 > offset_tolerance){
    REQUEST_EXTERNAL_RESET;
  }

   
  Serial.print("Acc1 init at: ");
  Serial.println(accelmag1.timings.first_t);
  Serial.print("Acc0 init at: ");
  Serial.println(accelmag0.timings.first_t);
  Serial.print("Gyro0 init at: ");
  Serial.println(gyro0.timings.first_t);
  Serial.print("Gyro1 init at: ");
  Serial.println(gyro1.timings.first_t);
 
  delay(100);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

// Need to deal with the fact that at first read, each sensor reading may be from t=0 or t=1. So can read and clear this data, 
// but then still out on next iteration

// So wait until all statuses are 0, then start. This works as long as the interval between successive reads is greater than the 
// sync offset. This interval is minimum 450us with a clock speed of 400000

  Serial.println("Clearing registers");
  eraseDataRegisters();
}

void loop(void)
{
  sensors_event_t event;
  sensors_event_t aevent, mevent;
  sensors_event_t event1;
  sensors_event_t aevent1, mevent1;
  
  gyro0.getEvent(&event);  
  accelmag0.getEvent(&aevent, &mevent);
  gyro1.getEvent(&event1);
  accelmag1.getEvent(&aevent1, &mevent1);


  /* Display raw data. NB These values require scaling */
  Serial.print("G ");
  Serial.print("X: "); Serial.print(gyro0.raw.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro0.raw.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro0.raw.z); Serial.print("  ");


  Serial.print("A ");
  Serial.print("X: "); Serial.print(accelmag0.accel_raw.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelmag0.accel_raw.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelmag0.accel_raw.z); Serial.print("  ");

  Serial.print("M ");
  Serial.print("X: "); Serial.print(accelmag0.mag_raw.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelmag0.mag_raw.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelmag0.mag_raw.z); Serial.print("  ");

  Serial.println("");

  Serial.print("G1 ");
   Serial.print("X1: "); Serial.print(gyro1.raw.x); Serial.print("  ");
  Serial.print("Y1: "); Serial.print(gyro1.raw.y); Serial.print("  ");
  Serial.print("Z1: "); Serial.print(gyro1.raw.z); Serial.print("  ");

  Serial.print("A1 ");
  Serial.print("X1: "); Serial.print(accelmag1.accel_raw.x); Serial.print("  ");
  Serial.print("Y1: "); Serial.print(accelmag1.accel_raw.y); Serial.print("  ");
  Serial.print("Z1: "); Serial.print(accelmag1.accel_raw.z); Serial.print("  ");

  Serial.print("M1 ");
  Serial.print("X1: "); Serial.print(accelmag1.mag_raw.x); Serial.print("  ");
  Serial.print("Y1: "); Serial.print(accelmag1.mag_raw.y); Serial.print("  ");
  Serial.print("Z1: "); Serial.print(accelmag1.mag_raw.z); Serial.print("  ");

  Serial.println("");
  delay(100);
}