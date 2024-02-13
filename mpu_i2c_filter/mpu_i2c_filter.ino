#include <Wire.h>
const int MPU_addr = 0x68;
double pitchInput, rollInput, yawInput, altitudeInput;
double xAcc, yAcc, zAcc, xGyro, yGyro, zGyro;
double currentGyroMillis, previousGyroMillis, deltaGyroTime;
double pitchInputAcc, rollInputAcc, yawInputAcc;
double pitchInputGyro, rollInputGyro, yawInputGyro;
double rollGyroOffset, pitchGyroOffset, yawGyroOffset, rollAccOffset, pitchAccOffset, yawAccOffset;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //gyro config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  //accelometer config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  currentGyroMillis = millis();
  if (rollAccOffset == 0) {
    for (int i = 0; i < 200; i++) {

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 6, true);

      xAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;
      yAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;
      zAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;

      pitchAccOffset += (atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * RAD_TO_DEG);
      rollAccOffset += (atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * RAD_TO_DEG);

      if (i == 199) {
        rollAccOffset = rollAccOffset / 200;
        pitchAccOffset = pitchAccOffset / 200;
      }
    }
  }
  if (rollGyroOffset == 0) {
    for (int i = 0; i < 200; i++) {

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 6, true);

      xGyro = Wire.read() << 8 | Wire.read();
      yGyro = Wire.read() << 8 | Wire.read();
      zGyro = Wire.read() << 8 | Wire.read();

      rollGyroOffset += yGyro / 32.8;
      pitchGyroOffset += xGyro / 32.8;
      yawGyroOffset += zGyro / 32.8;
      if (i == 199) {
        rollGyroOffset = rollGyroOffset / 200;
        pitchGyroOffset = pitchGyroOffset / 200;
        yawGyroOffset = yawGyroOffset / 200;
      }
    }
  }
}

void loop() {
  previousGyroMillis = currentGyroMillis;
  currentGyroMillis = millis();
  deltaGyroTime = (currentGyroMillis - previousGyroMillis) / 1000;

  //gyro

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  xGyro = Wire.read() << 8 | Wire.read();
  yGyro = Wire.read() << 8 | Wire.read();
  zGyro = Wire.read() << 8 | Wire.read();


  xGyro = (xGyro / 32.8) - pitchGyroOffset;
  yGyro = (yGyro / 32.8) - rollGyroOffset;
  zGyro = (zGyro / 32.8) - yawGyroOffset;

  pitchInputGyro = xGyro * deltaGyroTime;
  rollInputGyro = yGyro * deltaGyroTime;
  yawInputGyro = zGyro * deltaGyroTime;

  //accelometer

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  xAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;
  yAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;
  zAcc = (Wire.read() << 8 | Wire.read()) / 4096.0;

  pitchInputAcc = (atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * RAD_TO_DEG) - pitchAccOffset; 
  rollInputAcc = (atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * RAD_TO_DEG) - rollAccOffset;


  Serial.print("X:");
  Serial.print(xAcc);
  Serial.print("\tY:");
  Serial.print(yAcc);
  Serial.print("\tZ:");
  Serial.print(zAcc);


  //complementary filter
  rollInput = 0.98 * (rollInput + rollInputGyro) + 0.02 * (rollInputAcc);
  pitchInput = 0.98 * (pitchInput + pitchInputGyro) + 0.02 * (pitchInputAcc);
  yawInputAcc = atan2((sin(rollInput) * cos(pitchInput) * xAcc + sin(pitchInput) * yAcc + cos(rollInput) * cos(pitchInput) * zAcc), sqrt(pow(sin(rollInput) * sin(pitchInput) * xAcc - cos(rollInput) * sin(pitchInput) * zAcc, 2) + pow(cos(pitchInput) * xAcc, 2))) - 1;
  yawInput = 0.98 * (yawInput + yawInputGyro) + 0.02 * (yawInputAcc);


  Serial.print("\tp:");
  Serial.print(pitchInput);
  Serial.print("\tr:");
  Serial.print(rollInput);
  Serial.print("\ty:");
  Serial.println(yawInput);

  delay(100);
}