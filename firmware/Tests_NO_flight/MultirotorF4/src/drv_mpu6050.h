#pragma once

bool mpu6050Detect(sensor_t * acc, sensor_t * gyro, uint8_t *scale);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
//static void SPI1_Write(uint8_t Address, uint8_t Data )
