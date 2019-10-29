
#ifndef SENSORTYPES_H
#define SENSORTYPES_H

// 48*3 + 16 = 160 bits = 20 bytes
struct IMUmeas {
int16_t acc [3];   // Accelerometer 16*3= 48
int16_t gyro [3];  // Gyroscope 
int16_t mag [3];   // Magnetometer 
uint8_t gyro_status; // 8
uint8_t acc_mag_status; // 8
};

// Total 360 bits = 45bytes
struct data_t { 
    struct IMUmeas IMU1; // 20 bytes
    struct IMUmeas IMU2; // 20 bytes
    uint8_t FSR;      // 8bit 
    uint32_t time; //32 - could reduce to 8 if we store just the differential from LOG_INTERVAL
};

#endif /* SENSORTYPES_H */