#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include "printf.h"


#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

typedef enum {
    SENSOR_ACC = 1 << 0,
    SENSOR_BARO = 1 << 1,
    SENSOR_MAG = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS = 1 << 4,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6000 = 2,
    ACC_MMA8452 = 3,
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SPEKTRUM = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_GYRO_SMOOTHING = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
} AvailableFeatures;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK,
} GPSHardware;

typedef void (* sensorInitFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef int32_t (* baroCalculateFuncPtr)(void);             // baro calculation (returns altitude in cm based on static data collected)
typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data

typedef struct sensor_t
{
    sensorInitFuncPtr init;
    sensorReadFuncPtr read;
    sensorReadFuncPtr align;
    sensorReadFuncPtr temperature;
} sensor_t;

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t repeat_delay;
    sensorInitFuncPtr start_ut;
    sensorInitFuncPtr get_ut;
    sensorInitFuncPtr start_up;
    sensorInitFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

#define digitalLo(p, i)     { p->BSRRH = i; }
#define digitalHi(p, i)     { p->BSRRL = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO

#ifdef FY90Q
 // FY90Q
#define LED0_GPIO   GPIOC
#define LED0_PIN    GPIO_Pin_12
#define LED1_GPIO   GPIOA
#define LED1_PIN    GPIO_Pin_15

#define GYRO
#define ACC

#else
 // Afroflight32
#define LED0_GPIO   GPIOE
#define LED0_PIN    GPIO_Pin_0
#define LED1_GPIO   GPIOE
#define LED1_PIN    GPIO_Pin_1
#define LED2_GPIO   GPIOE
#define LED2_PIN    GPIO_Pin_2
#define LED3_GPIO   GPIOE
#define LED3_PIN    GPIO_Pin_3
#define BEEP_GPIO   GPIOD
#define BEEP_PIN    GPIO_Pin_11
#define BARO_GPIO   GPIOC
#define BARO_PIN    GPIO_Pin_13



#define GYRO
#define ACC
#define MAG
#define BARO
#define LEDRING
#define SONAR

#endif

// Helpful macros
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalLo(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalHi(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalLo(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalHi(LED1_GPIO, LED1_PIN);

#define LED2_TOGGLE              digitalToggle(LED2_GPIO, LED1_PIN);
#define LED2_OFF                 digitalLo(LED2_GPIO, LED2_PIN);
#define LED2_ON                  digitalHi(LED2_GPIO, LED2_PIN);

#define LED3_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED3_OFF                 digitalLo(LED3_GPIO, LED3_PIN);
#define LED3_ON                  digitalHi(LED3_GPIO, LED3_PIN);  

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalLo(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalHi(BEEP_GPIO, BEEP_PIN);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

#undef SOFT_I2C                 // enable to test software i2c

#ifdef FY90Q
 // FY90Q
#include "drv_system.h"         // timers, delays, etc
//#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_pwm.h"
#include "drv_uart.h"

#else
 // AfroFlight32
#include "drv_system.h"         // timers, delays, etc
#include "drv_adc.h"
#include "drv_adxl345.h"
//#include "drv_bmp085.h"
#include "drv_ms5611.h"
#include "drv_hmc5883l.h"
#include "drv_i2c.h"
#include "drv_ledring.h"
#include "drv_mma845x.h"
#include "drv_mpu3050.h"
//#include "drv_mpu6050.h"
#include "drv_mpu6000.h"
#include "drv_l3g4200d.h"
#include "drv_pwm.h"
#include "drv_uart.h"
#include "drv_hcsr04.h"
#include "drv_spi.h"

#endif
