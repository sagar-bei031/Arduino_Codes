#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "crc8.hpp"
#include <memory.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 18
#define EXTERNAL_LED_PIN 19
#define TRANSFER_LED_PIN 2
#define RXD2 16
#define TXD2 17

#define START_BYTE 0xA5

// #define DEBUG

#if defined USB_COM && defined UART_COM
#error "Can't implement both usb and uart"
#endif


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
// VectorInt16 a;
// VectorInt16 aa;
// VectorInt16 aaa;
float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
const float g = 9.80665; // ROS standard

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

CRC8 crc(7);
float yaw;
uint8_t transmitting_bytes[6];
unsigned long last_transmit_tick = 0;
unsigned long transfer_led_tick = 0;

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

#ifdef DEBUG
  Serial.begin(115200);
#endif

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);


#ifdef DEBUG
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();
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
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);
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

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // mpu.dmpGetAccel(&a, fifoBuffer);
    // mpu.dmpGetLinearAccel(&aa, &a, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaa, &aa, &q);

    yaw = -ypr[0];

    unsigned long now = millis();

    if (now - last_transmit_tick >= 10) {
      
      transmitting_bytes[0] = START_BYTE;
      memcpy(transmitting_bytes + 1, &yaw, 4);
      transmitting_bytes[5] = crc.get_hash(transmitting_bytes + 1, 4);

        Serial2.write(transmitting_bytes, 6);

#ifdef DEBUG
      Serial.println(yaw * 180 / PI);
#endif

      if (now - transfer_led_tick > 20) {
        digitalWrite(TRANSFER_LED_PIN, !digitalRead(TRANSFER_LED_PIN));
        transfer_led_tick = now;
      }

      last_transmit_tick = now;
    }
  } else if (millis() - last_transmit_tick > 1000) {
    // Serial.println("fifo error");
    // error_handler();
  }
}

void error_handler() {
  for (int i = 0; i < 20; ++i) {
    digitalWrite(EXTERNAL_LED_PIN, !digitalRead(EXTERNAL_LED_PIN));
    delay(100);
  }
  esp_restart();
}