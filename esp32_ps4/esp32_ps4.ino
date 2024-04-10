/**
 ************************************************************************
 * @file    esp_ps4.ino
 * @brief   Connection betweenn ESP32 and PS4 controller
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2023
 ************************************************************************
 * @details
 * Connect PS4 conroller using bluetooth and send joystick data using uart.
 *
 * Packet Format:
 * [0] - Start Byte (0xA5 or 65)
 * [1] - Left Joystick X
 * [2] - Left Joystick Y
 * [3] - Right Joystick X
 * [4] - Right Joystick Y
 * [5] - Left Trigger (L2)
 * [6] - Right Trigger (R2)
 * [7] - Buttons Lower Byte
 * [8] - Buttons Higher Byte
 * [9] - CRC Hash of Joystick Data (1-8) i.e. excluding start byte 
 *
 * The packet format follows little-endian.
 */

#include <PS4Controller.h>
#include "crc8.hpp"

// Define to enable debugging
#define DEBUG

/**! Pins assignments of ESP32 */
#define IN_BUILT_LED_BLUE 2
#define RXD2 16
#define TXD2 17

/**! Start byte of packet */
#define START_BYTE 0b10100101

/**! Define to mask bit at position x */
#define BB(x) (1 << x)

/** @Enum for buttons bit postion */
enum Buttons {
  Cross,
  Circle,
  Square,
  Triangle,
  Share,
  Power,
  Option,
  L3,
  R3,
  L1,
  R1,
  Up,
  Down,
  Left,
  Right,
  Touch
};
/** @Struct for joystick data */
struct Joydata {
  int8_t lx;        /**< Left Joystick X */
  int8_t ly;        /**< Left Joystick Y */
  int8_t rx;        /**< Right Joystick X */
  int8_t ry;        /**< Right Joystick Y */
  uint8_t lt;       /**< Left Trigger (L2) */
  uint8_t rt;       /**< Right Trigger (R2) */
  uint16_t buttons; /**< Buttons */
};

Joydata jdata;                            /*! Global object of Joydata */
uint8_t packet[10];                       /*! Packet store the transmitting data */
CRC8 crc(7);                              /*! CRC8 object to calculate hash for error detection */
unsigned long int last_transmit_tick = 0; /*! Variable to track transmission time */

/** @brief Initialize controller parameters */
void setup() {

  /**< Initialize UART2 of ESP32 for transmitting data received from PS4 to STM32 */
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  //Callbacks
  // PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  /**< Initiates communication between ESP32 and PS4 */
  // c8 ps4:: 28:C1:3C:7D:CC:B6
  PS4.begin();

  /**< Initalize GPIO pins */
  pinMode(IN_BUILT_LED_BLUE, OUTPUT);

  /**< Reset on board Blue LED */
  digitalWrite(IN_BUILT_LED_BLUE, LOW);  // active high

#ifdef DEBUG
  Serial.begin(115200, SERIAL_8N1);
  Serial.println("Ready.");
#endif
}

/** @brief Operation */
void loop() {
  /**< Check connection */
  if (PS4.isConnected()) {

    /**< Control transfer rate */
    unsigned long int current_tick = millis();
    if (current_tick - last_transmit_tick >= 30) {

      /**< Parse PS4 data to our format */
      /**< Parse all axes*/
      jdata.lx = PS4.LStickX();
      jdata.ly = PS4.LStickY();
      jdata.rx = PS4.RStickX();
      jdata.ry = PS4.RStickY();
      jdata.lt = PS4.L2Value();
      jdata.rt = PS4.R2Value();

      // Serial.println(jdata.lt);

      /**< Parse buttons */
      jdata.buttons = 0;
      if (PS4.Cross()) jdata.buttons |= BB(Cross);
      if (PS4.Circle()) jdata.buttons |= BB(Circle);
      if (PS4.Square()) jdata.buttons |= BB(Square);
      if (PS4.Triangle()) jdata.buttons |= BB(Triangle);
      if (PS4.PSButton()) jdata.buttons |= BB(Power);
      if (PS4.Options()) jdata.buttons |= BB(Option);
      if (PS4.L3()) jdata.buttons |= BB(L3);
      if (PS4.R3()) jdata.buttons |= BB(R3);
      if (PS4.L1()) jdata.buttons |= BB(L1);
      if (PS4.R1()) jdata.buttons |= BB(R1);
      if (PS4.Up()) jdata.buttons |= BB(Up);
      if (PS4.Down()) jdata.buttons |= BB(Down);
      if (PS4.Left()) jdata.buttons |= BB(Left);
      if (PS4.Right()) jdata.buttons |= BB(Right);
      if (PS4.Touchpad()) jdata.buttons |= BB(Touch);

      /**< Store into the packet for transmission */
      packet[0] = START_BYTE;
      memcpy((uint8_t*)packet + 1, (uint8_t*)&jdata, sizeof(jdata));
      packet[9] = crc.get_hash(packet + 1, sizeof(jdata));

      /**< Finally send packet through UART2 */
      for (uint8_t i = 0; i < 10; i++) {
        Serial2.write(packet[i]);
      }

      last_transmit_tick = current_tick;

#ifdef DEBUG
      /**< Print to Serial for debug */
      for (uint8_t i = 0; i < 10; i++) {
        Serial.print(packet[i]);
        Serial.print('\t');
      }
#endif

#ifdef DEBUG
      /**< Check battry if DEBUG is defined */
      uint8_t battery = PS4.Battery();
      Serial.print(battery);
      Serial.println();
#endif
    }
  }
}

/** @brief Callback on connect */
void onConnect() {
  /**< Turn on Blue LED */
  digitalWrite(IN_BUILT_LED_BLUE, HIGH);  // active high

#ifdef DEBUG
  Serial.println("Connected to PS4!.");
#endif
}

/** @brief Callback on disconnect */
void onDisConnect() {
  /**< Turn off Blue LED */
  digitalWrite(IN_BUILT_LED_BLUE, LOW);  // active high

#ifdef DEBUG
  Serial.println("PS4 Disconnected!.");
#endif
}

