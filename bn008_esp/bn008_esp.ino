#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <memory.h>
#include "crc8.hpp"

#define DEBUG

// UART can be used but the ports might be reserved so better to use I2C and SPI
// To use UART, some modification must be done

// #define USE_I2C
#define USE_SPI

#define LED_Pin 2
#define RXD2 16
#define TXD2 17

// For SPI mode, we need a CS pin
#define BNO08X_CS 5
#define BNO08X_INT 22

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET 21  // -1

// #define FAST_MODE

// esp32
#define BN008x_P0 25
#define BN008x_P1 26

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

#define START_BYTE 0xA5

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr, offset_ypr, rel_ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
uint8_t sending_packet[2 + sizeof(rel_ypr)];

CRC8 crc8(7);

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

bool read_data() {
  bool is_read = false;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    is_read = true;
  }

  return is_read;
}

void caliberate(uint16_t n = 20) {
  offset_ypr.yaw = 0.0f;
  offset_ypr.pitch = 0.0f;
  offset_ypr.roll = 0.0f;

  int i = 0;
  while (i < n) {
    if (read_data()) {
      offset_ypr.yaw += ypr.yaw;
      offset_ypr.pitch += ypr.pitch;
      offset_ypr.roll += ypr.roll;
      i++;
    }
  }

  offset_ypr.yaw /= n;
  offset_ypr.pitch /= n;
  offset_ypr.roll /= n;
}

void setup(void) {
  pinMode(BN008x_P0, OUTPUT);
  pinMode(BN008x_P1, OUTPUT);
  digitalWrite(BN008x_P0, p0_value);
  digitalWrite(BN008x_P1, p1_value);

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

// Try to initialize!
#if defined USE_I2C
  if (!bno08x.begin_I2C()) {
#elif defined USE_UART
  if (!bno08x.begin_UART(&Serial2)) {  // Requires a device with > 300 byte UART buffer!
#elif defined USE_SPI
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
#endif
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.print("Caliberating");
  delay(100);
  caliberate(20);

  Serial.println("Reading events");
}

void loop() {
  read_data();

  static unsigned long last_tick = 0;
  unsigned long now = micros();
  unsigned long dt = now - last_tick;

  if (dt > 30000) {
    rel_ypr.yaw = ypr.yaw - offset_ypr.yaw;
    rel_ypr.pitch = ypr.pitch - offset_ypr.pitch;
    rel_ypr.roll = ypr.roll - offset_ypr.roll;

    sending_packet[0] = START_BYTE;
    memcpy(sending_packet + 1, (uint8_t*)&rel_ypr, sizeof(rel_ypr));
    sending_packet[1 + sizeof(rel_ypr)] = crc8.get_hash((uint8_t*)&rel_ypr, sizeof(rel_ypr));
    Serial2.write(sending_packet, sizeof(sending_packet));

    last_tick = now;

#ifdef DEBUG
    Serial.print(dt);
    Serial.print("\t");

    Serial.print(sensorValue.status);
    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw - offset_ypr.yaw);
    Serial.print("\t");
    Serial.print(ypr.pitch - offset_ypr.pitch);
    Serial.print("\t");
    Serial.println(ypr.roll - offset_ypr.roll);
#endif
  }
}
