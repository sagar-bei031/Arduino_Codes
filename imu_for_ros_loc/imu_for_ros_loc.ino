#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "crc8.hpp"
#include <memory.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// #define PICO
// #define DEBUG

#ifdef PICO
#define INTERRUPT_PIN 6
#define EXTERNAL_LED_PIN 7
#define TRANSFER_LED_PIN 25
#else
#define INTERRUPT_PIN 18
#define EXTERNAL_LED_PIN 19
#define TRANSFER_LED_PIN 2
// #define RXD2 16                                                                                                                                                                               
// #define TXD2 17
#endif


#define START_BYTE 0xA5

// #define DEBUG

#if defined USB_COM && defined UART_COM
#error "Can't implement both usb and uart"
#endif

// #ifdef PICO
// #ifdef ARDUINO_ARCH_MBED_RP2040
// // For the Arduino MBedOS Core, we must create Serial2
// UART Serial2(8, 9, NC, NC);
// #endif
// #endif

MPU6050 mpu;

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
VectorInt16 gyro;
VectorInt16 a;
// VectorInt16 aa;
// VectorInt16 aaa;
float ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
const float g = 9.80665;  // ROS standard

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

CRC8 crc(7);
float yaw;
unsigned long last_transmit_tick = 0;

struct ImuMsg {
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
} imu_msg;

uint8_t transmitting_bytes[sizeof(imu_msg) + 2];

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

#ifdef DEBUG
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();

  delay(1000);

  pinMode(INTERRUPT_PIN, INPUT);
  // configure LED for output
  pinMode(TRANSFER_LED_PIN, OUTPUT);
  pinMode(EXTERNAL_LED_PIN, OUTPUT);

  // verify connection
  bool testConnection = mpu.testConnection();
#ifdef DEBUG
  // Serial.println(F("Testing device connections..."));
  Serial.println(testConnection ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  // load and configure the DMP
#ifdef DEBUG
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // banngo wala
  // mpu.setXAccelOffset(-2463);
  // mpu.setYAccelOffset(-645);
  // mpu.setZAccelOffset(526);
  // mpu.setXGyroOffset(112);
  // mpu.setYGyroOffset(13);
  // mpu.setZGyroOffset(-19);

  //sidha wala
  mpu.setXAccelOffset(-31);
  mpu.setYAccelOffset(-1401);
  mpu.setZAccelOffset(2153);
  mpu.setXGyroOffset(365);
  mpu.setYGyroOffset(211);
  mpu.setZGyroOffset(-99);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro();
    mpu.CalibrateAccel();
#ifdef DEBUG
    mpu.PrintActiveOffsets();
#endif
    // turn on the DMP, now that it's ready
    // turn on the DMP, now that it's ready
#ifdef DEBUG
    Serial.println(F("Enabling DMP..."));
#endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#ifdef DEBUG
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
    error_handler();
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  unsigned long now = millis();

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&a, fifoBuffer);
    // mpu.dmpGetLinearAccel(&aa, &a, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaa, &aa, &q);

    yaw = ypr[0];

    imu_msg.orientation[0] = q.w;
    imu_msg.orientation[1] = q.x;
    imu_msg.orientation[2] = q.y;
    imu_msg.orientation[3] = q.z;

    /* AFS_SEL    Full Scale Range   LSB Sensitivity
     * 0           +/- 250  deg/s    131  LSB/deg/s
     * 1           +/- 500  deg/s    65.5 LSB/deg/s
     * 3           +/- 1000 deg/s    32.8 LSB/deg/s
     * 0           +/- 2000 deg/s    16.4 LSB/deg/s   #
     */
    imu_msg.angular_velocity[0] = (gyro.x * 2000.0 * PI) / (16.4 * 180.0);
    imu_msg.angular_velocity[1] = (gyro.y * 2000.0 * PI) / (16.4 * 180.0);
    imu_msg.angular_velocity[2] = (gyro.z * 2000.0 * PI) / (16.4 * 180.0);

    /* AFS_SEL    Full Scale Range   LSB Sensitivity
     * 0           +/- 2g            16384 LSB/g      #
     * 1           +/- 4g            8192  LSB/g
     * 2           +/- 8g            4096  LSB/g
     * 3           +/- 16g           2048  LSB/g
     */
    imu_msg.linear_acceleration[0] = (a.x * 2.0 * g) / (16384.0);
    imu_msg.linear_acceleration[1] = (a.y * 2.0 * g) / (16384.0);
    imu_msg.linear_acceleration[2] = (a.z * 2.0 * g) / (16384.0);

    if (now - last_transmit_tick > 33) {
      transmitting_bytes[0] = START_BYTE;
      memcpy(transmitting_bytes + 1, (uint8_t*)&imu_msg, sizeof(imu_msg));
      transmitting_bytes[sizeof(imu_msg) + 1] = crc.get_hash(transmitting_bytes + 1, sizeof(imu_msg));

#ifdef DEBUG
      Serial.println(yaw * 180 / PI);
#else
      Serial.write(transmitting_bytes, sizeof(transmitting_bytes));
#endif
      digitalWrite(TRANSFER_LED_PIN, !digitalRead(TRANSFER_LED_PIN));

      last_transmit_tick = now;
    }
  }
}

void error_handler() {
  for (int i = 0; i < 20; ++i) {
    digitalWrite(EXTERNAL_LED_PIN, !digitalRead(EXTERNAL_LED_PIN));
    delay(100);
  }

#ifdef PICO
  watchdog_reboot(0, 0, 10);
  while (1) {
    continue;
  }
#else
  esp_restart();
#endif
}