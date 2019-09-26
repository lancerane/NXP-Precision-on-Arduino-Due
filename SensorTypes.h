
#ifndef SENSORTYPES_H
#define SENSORTYPES_H
    
struct IMUmeas {
int acc [3];   // Accelerometer
int gyro [3];  // Gyroscope
int mag [3];   // Magnetometer
uint8_t gyro_status;
uint8_t acc_mag_status;
};

// Total 360bits per packet = 45bytes
struct data_t { 
    struct IMUmeas IMU1; //160
    struct IMUmeas IMU2; //160
    uint8_t FSR;      // 8
    uint32_t time; //32
};

#endif /* SENSORTYPES_H */