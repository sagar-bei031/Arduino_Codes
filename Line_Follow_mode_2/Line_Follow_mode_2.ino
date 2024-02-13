// Line following diffential driving car
// Prototyping robot code

// - Created on Oct 12, 2024 by Sagar


// #define _DEBUG_


#include <math.h>


// Pin NUMBERS
#define MOT_L_DIR 8
#define MOT_L_PWM 9
#define MOT_R_DIR 10
#define MOT_R_PWM 11

#define UEN 17
#define JP 19

#define LED_PIN 13


// If the line, which is 3cm wide, is in the middle, sensor reading becomes 0x18 i.e. 00001 1000
#define CENTER_VALUE 0x18

// Upper threshold and lower threshold of voltage equivalent to reflected intensity of IR
#define UPPER_THRESHOLD 800
#define LOWER_THRESHOLD 200

// Values to decide the turning condtion on sensor value
#define LEFT_TURN_BYTE 0xF0
#define RIGHT_TURN_BYTE 0x0F


// Constants of PID control
// #define KP 1
// #define KI 1
// #define KD 1


const uint8_t TOP_SPEED = 255;
const uint8_t LOW_SPEED = 150;
const uint8_t INPUT_RANGE = 70;
const uint8_t MID_VALUE = 35;  // setpoint


// reading from sensor which we use in processing
uint8_t sensorValue;
bool isJP;


uint8_t L_PWM;
uint8_t R_PWM;


void setup() {
  // Initialize serial3 for raeding single from sensor
#ifdef _DEBUG_
  Serial.begin(115200);
#endif

  Serial3.begin(115200);
  // Initialize analog pins for sensor
  pinMode(UEN, OUTPUT);
  pinMode(JP, INPUT);

  // Initialize digital pins for motor
  pinMode(MOT_L_DIR, OUTPUT);
  pinMode(MOT_R_DIR, OUTPUT);
  pinMode(MOT_L_PWM, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  // UEN : UART ENABLE is active low
  digitalWrite(UEN, LOW);
}


void loop() {
  if (Serial3.available()) {
    sensorValue = Serial3.read();
    digitalWrite(LED_PIN, BIN);
    isJP = digitalRead(JP);

#ifdef _DEBUG_
    Serial.print("Sensor value:");
    Serial.print(sensorValue);
    Serial.print(" isJP:");
    Serial.print(isJP);
#endif

    Run();
  } else {
    // stop();
    digitalWrite(LED_PIN, HIGH);
  }
}


inline void
Stop() {
  // Disable enable signal of each motor
  analogWrite(MOT_R_PWM, 0);
  analogWrite(MOT_L_PWM, 0);
#ifdef _DEBUG_
  Serial.println(" Stop");
#endif
}


inline void
Back() {
  analogWrite(MOT_R_PWM, LOW_SPEED);
  analogWrite(MOT_L_PWM, LOW_SPEED);
  digitalWrite(MOT_R_DIR, HIGH);
  digitalWrite(MOT_L_DIR, HIGH);
#ifdef _DEBUG_
  Serial.println(" Back");
#endif
}


inline void
Straight() {
  analogWrite(MOT_R_PWM, TOP_SPEED);
  analogWrite(MOT_L_PWM, TOP_SPEED);
  digitalWrite(MOT_R_DIR, LOW);
  digitalWrite(MOT_L_DIR, LOW);
#ifdef _DEBUG_
  Serial.println(" Straight");
#endif
}


inline void
Follow() {
  analogWrite(MOT_R_PWM, R_PWM);
  analogWrite(MOT_L_PWM, L_PWM);
  digitalWrite(MOT_R_DIR, LOW);
  digitalWrite(MOT_L_DIR, LOW);
#ifdef _DEBUG_
  Serial.println(" Follow");
#endif
}


inline void
Error() {
#ifdef _DEBUG_
  Serial.println(" Error");
#endif
}


inline void
Run() {
  if (isJP) {
    Straight();
  } else if ((sensorValue >= 0) && (sensorValue <= 70)) {
    L_PWM = (uint8_t)sensorValue * 255 / INPUT_RANGE;
    R_PWM = TOP_SPEED - (uint8_t)sensorValue * 255 / INPUT_RANGE;
#ifdef _DEBUG_
    Serial.print(" L_PWM:");
    Serial.print(L_PWM);
    Serial.print(" R_PWM:");
    Serial.print(R_PWM);
#endif

    Follow();
  } else if (sensorValue == 255) {
    Back();
  } else {
    Error();
    Stop();
  }
}
