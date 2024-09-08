/**
 **************************************************************************
 * @file    esp_ps4.ino
 * @brief   Connection betweenn ESP32 and PS4 controller
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2024
 **************************************************************************
 */

// Uncomment the line below to enable debugging
// #define DEBUG

/* Implement CRC or Checksum.
** Must define one of these two and comment another 
*/
#define CRC
// #define CHECKSUM

#include <PS4Controller.h>

#ifdef CRC
#include "crc8.hpp"
#endif

/** @brief Pins assignments of ESP32 */
#define IN_BUILT_LED_BLUE 2
#define RXD2 16
#define TXD2 17

/** @brief Start byte of packet */
#define START_BYTE 0b10100101

#ifdef CRC
/** @brief Make object of CRC_HASH and call constructor with parameter: polynomial = 7
* It generates CRC lookup table */
CRC8 crc(7);
#endif

#define BB(x) (1 << x)
uint32_t last_transmit = 0;

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

struct Joydata {
  int8_t lx;
  int8_t ly;
  int8_t rx;
  int8_t ry;
  uint8_t lt;
  uint8_t rt;
  uint16_t buttons;
};

Joydata jdata;
bool is_flash_set = false;
bool is_braked;

/** @brief Initialize controller parameters */
void setup() {

  Serial.begin(115200, SERIAL_8N1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(IN_BUILT_LED_BLUE, OUTPUT);
  digitalWrite(IN_BUILT_LED_BLUE, LOW);

  //Callbacks
  // PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  PS4.begin();
  Serial.print("ESP32 MAC: ");
  Serial.println(PS4.getAddress());
}

/** @brief Operation */
void loop() {
  if (!PS4.isConnected())
    return;

  // if (PS4.Battery() < 5) {
  //   if (!is_flash_set) {
  //     PS4.setFlashRate(200, 55);
  //     is_flash_set = true;
  //   }
  // } else {
  //   if (is_flash_set) {
  //     PS4.setFlashRate(255, 0);
  //     is_flash_set = false;
  //   }
  // }

  uint32_t now = millis();
  if (now - last_transmit < 10)
    return;

  uint8_t packet[10];
  jdata.lx = PS4.LStickX();
  jdata.ly = -PS4.LStickY();
  jdata.rx = PS4.RStickX();
  jdata.ry = -PS4.RStickY();
  jdata.lt = PS4.L2Value();
  jdata.rt = PS4.R2Value();

  jdata.buttons = 0;

  if (PS4.Cross()) jdata.buttons |= BB(Cross);
  if (PS4.Circle()) jdata.buttons |= BB(Circle);
  if (PS4.Square()) jdata.buttons |= BB(Square);
  if (PS4.Triangle()) jdata.buttons |= BB(Triangle);

  if (PS4.Share()) jdata.buttons |= BB(Share);
  if (PS4.PSButton()) jdata.buttons |= BB(Power);
  if (PS4.Options()) jdata.buttons |= BB(Option);

  if (PS4.L3()) jdata.buttons |= BB(L3);
  if (PS4.R3()) jdata.buttons |= BB(R3);
  if (PS4.L1()) jdata.buttons |= BB(L1);
  if (PS4.R1()) jdata.buttons |= BB(R1);

  if (PS4.Up()) jdata.buttons |= BB(Up);
  if (PS4.UpLeft()) jdata.buttons |= (BB(Up) | BB(Left));
  if (PS4.Left()) jdata.buttons |= BB(Left);
  if (PS4.DownLeft()) jdata.buttons |= (BB(Left) | BB(Down));
  if (PS4.Down()) jdata.buttons |= BB(Down);
  if (PS4.DownRight()) jdata.buttons |= (BB(Down) | BB(Right));
  if (PS4.Right()) jdata.buttons |= BB(Right);
  if (PS4.UpRight()) jdata.buttons |= (BB(Up) | BB(Right));
  
  if (PS4.Touchpad()) jdata.buttons |= BB(Touch);

  packet[0] = START_BYTE;
  memcpy((uint8_t*)packet + 1, (uint8_t*)&jdata, sizeof(jdata));
  packet[9] = crc.get_hash(packet + 1, sizeof(jdata));

  // Finally send packet to STM32 through UART2
  Serial2.write(packet, sizeof(packet));

  if (PS4.PSButton()) {
    if (PS4.L1() && PS4.R1()) {
      if (is_braked) {
        PS4.setLed(0, 255, 0);
        PS4.sendToController();
        // Serial.println("Brake removed");
        is_braked = false;
      }
    } else {
      if (!is_braked) {
        PS4.setLed(255, 0, 255);
        PS4.sendToController();
        // Serial.println("Braked");
        is_braked = true;
      }
    }
  }

#ifdef DEBUG
  Serial.printf("%lu: %hd %hd %hd %hd %hu %hu %04X\n",
                now - last_transmit, jdata.lx, jdata.ly, jdata.rx, jdata.ry, jdata.lt, jdata.rt, jdata.buttons);
#endif

  last_transmit = now;
}

/** @brief Callback on connect */
void onConnect() {
  digitalWrite(IN_BUILT_LED_BLUE, HIGH);
  Serial.printf("Battery: %d\n", PS4.Battery());
  PS4.setLed(0, 255, 0);
  PS4.sendToController();
  Serial.println("PS4 Connected.");
}

/** @brief Callback on disconnect */
void onDisConnect() {
  digitalWrite(IN_BUILT_LED_BLUE, LOW);
  Serial.println("!!PS4 Disconnected!!");
}