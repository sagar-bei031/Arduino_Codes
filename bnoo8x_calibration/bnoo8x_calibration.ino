#include "bnoo8x_calibration.hpp"

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
euler_t original_ypr, offset_ypr, calibrated_ypr;
vector3f angular_velocity;
vector3f acceleration;
unsigned long last_tick = 0;

void setup(void) {
  pinMode(LED_Pin, OUTPUT);
  pinMode(BN008x_P0, OUTPUT);
  pinMode(BN008x_P1, OUTPUT);

  digitalWrite(BN008x_P0, p0_value);
  digitalWrite(BN008x_P1, p1_value);

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
  unsigned long now = micros();
  unsigned long dt = now - last_tick;

    read_data();
    
  if (dt > 30000) {

    Serial.print(dt);

    Serial.print("\tStatus: ");
    Serial.print(sensorValue.status);  // This is accuracy in the range of 0 to 3

    Serial.print("\t ypr \t: ");
    Serial.print(calibrated_ypr.yaw * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(calibrated_ypr.pitch * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(calibrated_ypr.roll * RAD_TO_DEG);
    Serial.print("\t");

    Serial.print("\t vel \t: ");
    Serial.print(angular_velocity.x);
    Serial.print("\t");
    Serial.print(angular_velocity.y);
    Serial.print("\t");
    Serial.print(angular_velocity.z);
    Serial.print("\t");

    Serial.print("\t accel \t: ");
    Serial.print(acceleration.x);
    Serial.print("\t");
    Serial.print(acceleration.y);
    Serial.print("\t");
    Serial.print(acceleration.z);
    Serial.println();

    blick_led(LED_Pin);
    last_tick = now;
  }
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

void calibrate(uint16_t n) {
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
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &original_ypr);
        offset_ypr.yaw += original_ypr.yaw;
        offset_ypr.pitch += original_ypr.pitch;
        offset_ypr.roll += original_ypr.roll;
        i++;

        delay(10);
      }
    }
  }
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
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &original_ypr);
        calibrated_ypr.yaw = original_ypr.yaw - offset_ypr.yaw;
        calibrated_ypr.pitch = original_ypr.pitch - offset_ypr.pitch;
        calibrated_ypr.roll = original_ypr.roll - offset_ypr.roll;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        angular_velocity.x = sensorValue.un.gyroscope.x;
        angular_velocity.y = sensorValue.un.gyroscope.y;
        angular_velocity.z = sensorValue.un.gyroscope.z;
        break;

      case SH2_LINEAR_ACCELERATION:
        acceleration.x = sensorValue.un.linearAcceleration.x;
        acceleration.y = sensorValue.un.linearAcceleration.y;
        acceleration.z = sensorValue.un.linearAcceleration.z;
        break;
    }
  }
}

void blick_led(uint8_t pin) {
  digitalWrite(pin, !digitalRead(pin));
}

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