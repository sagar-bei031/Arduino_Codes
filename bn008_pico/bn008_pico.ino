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
} ypr, offset_ypr;

struct vector3f {
  float x;
  float y;
  float z;
};

struct robot_state {
  euler_t orientation;
  vector3f velocity;
  vector3f accel;
} rstate;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

uint8_t sending_packet[2 + sizeof(rstate)];

CRC8 crc8(7);

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr);
}

void setReport(sh2_SensorId_t reportType) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  Serial.println("Setting desired reports");
  // if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
  //   Serial.println("Could not enable accelerometer");
  // }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  // if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
  //   Serial.println("Could not enable magnetic field calibrated");
  // }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  // if (!bno08x.enableReport(SH2_GRAVITY)) {
  //   Serial.println("Could not enable gravity vector");
  // }
  // if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
  //   Serial.println("Could not enable rotation vector");
  // }
  // if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
  //   Serial.println("Could not enable geomagnetic rotation vector");
  // }
  // if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
  //   Serial.println("Could not enable game rotation vector");
  // }
}

void calibrate(uint16_t n = 20) {
  offset_ypr.yaw = 0.0f;
  offset_ypr.pitch = 0.0f;
  offset_ypr.roll = 0.0f;

  int i = 0;
  while (i < n) {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReport(SH2_ARVR_STABILIZED_RV);
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
        offset_ypr.yaw += ypr.yaw;
        offset_ypr.pitch += ypr.pitch;
        offset_ypr.roll += ypr.roll;
        i++;

        delay(10);
      }
    }
  }

  offset_ypr.yaw /= n;
  offset_ypr.pitch /= n;
  offset_ypr.roll /= n;
}

void read_data() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
        rstate.orientation.yaw = ypr.yaw - offset_ypr.yaw;
        rstate.orientation.pitch = ypr.pitch - offset_ypr.pitch;
        rstate.orientation.roll = ypr.roll - offset_ypr.roll;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        rstate.velocity.x = sensorValue.un.gyroscope.x;
        rstate.velocity.y = sensorValue.un.gyroscope.y;
        rstate.velocity.z = sensorValue.un.gyroscope.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        rstate.accel.x = sensorValue.un.linearAcceleration.x;
        rstate.accel.y = sensorValue.un.linearAcceleration.y;
        rstate.accel.z = sensorValue.un.linearAcceleration.z;
        break;
    }
  }
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

  setReport(SH2_ARVR_STABILIZED_RV);
  Serial.print("Calibrating");
  delay(100);
  calibrate(20);

  setReports();
  Serial.println("Reading events");
}

void loop() {
  static unsigned long last_tick = 0;
  unsigned long now = micros();
  unsigned long dt = now - last_tick;

  if (dt > 30000) {
  read_data();

    sending_packet[0] = START_BYTE;
    memcpy(sending_packet + 1, (uint8_t*)&rstate, sizeof(rstate));
    sending_packet[1 + sizeof(rstate)] = crc8.get_hash((uint8_t*)&rstate, sizeof(rstate));
    Serial2.write(sending_packet, sizeof(sending_packet));

    last_tick = now;

#ifdef DEBUG
    Serial.print(dt);
    Serial.print("\t");

    Serial.print(sensorValue.status);
    Serial.print("\t");  // This is accuracy in the range of 0 to 3

    Serial.print(rstate.orientation.yaw * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(rstate.orientation.pitch * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(rstate.orientation.roll * RAD_TO_DEG);
    Serial.print("\t");

    Serial.print(rstate.velocity.x);
    Serial.print("\t");
    Serial.print(rstate.velocity.y);
    Serial.print("\t");
    Serial.print(rstate.velocity.z);
    Serial.print("\t");

    Serial.print(rstate.accel.x);
    Serial.print("\t");
    Serial.print(rstate.accel.y);
    Serial.print("\t");
    Serial.print(rstate.accel.z);
    Serial.println();
#endif
  }
}
