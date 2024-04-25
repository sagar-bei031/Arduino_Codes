#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <memory.h>
#include "crc8.hpp"

/**************************************************
  Toggle Definition for specific implementatiom
**************************************************/
// #define DEBUG
#define ESP32
// #define PICO

// Use One of These
// #define USE_I2C
#define USE_SPI


#if !(defined ESP32 ^ defined PICO)
#error "Must define ESP32 or PICO exclusively"
#endif


/*************************************************
  Pin config for esp32
/************************************************/
#ifdef ESP32

#define LED_Pin 2
#define RXD2 16
#define TXD2 17

// For SPI mode
#define BNO08X_CS 5
#define BNO08X_INT 22
#ifdef USE_SPI

#define BNO08X_RESET 21
#else
#define BNO08X_RESET -1
#endif

#define BN008x_P0 25
#define BN008x_P1 26

#endif  // ESP32

/*************************************************
  Pin config for pico
/************************************************/
#ifdef PICO

#define LED_Pin 25
#define RXD2 9
#define TXD2 8

// For SPI mode
#define BNO08X_CS 17
#define BNO08X_INT 5

#ifdef USE_SPI
#define BNO08X_RESET 2
#else
#define BNO08X_RESET -1
#endif

#define BN008x_P0 3
#define BN008x_P1 4

#endif  // PICO


#define START_BYTE 0xA5

#if defined USE_I2C
const uint8_t p0_value = LOW;
const uint8_t p1_value = LOW;
#elif defined USE_UART
const uint8_t p0_value = LOW;
const uint8_t p1_value = HIGH;
#elif defined USE_SPI
const uint8_t p0_value = HIGH;
const uint8_t p1_value = HIGH;
#else
#error "Must defined USE_I2C or USE_SPI"
#endif

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

struct vector3f {
  float x;
  float y;;
  float z;
};

struct vector4f {
  float w;
  float x;
  float y;
  float z;
};

struct robot_state {
  vector4f orientation;
  vector3f velocity;
  vector3f accel;
};

void setReport(sh2_SensorId_t reportType);

void setReports(void);

void read_data();

void blick_led(uint8_t pin);

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr);

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr);

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr);