
#ifndef SENSORTYPES_H
#define SENSORTYPES_H

// 112 bits
struct IMUmeas {
int acc [3];   // Accelerometer //32
int gyro [3];  // Gyroscope //32
int mag [3];   // Magnetometer //32
uint8_t gyro_status; // 8
uint8_t acc_mag_status; // 8
};

// Total 264bits per packet = 33bytes
struct data_t { 
    struct IMUmeas IMU1; // 112
    struct IMUmeas IMU2; // 112
    uint8_t FSR;      // 8
    uint32_t time; //32
};

#endif /* SENSORTYPES_H */