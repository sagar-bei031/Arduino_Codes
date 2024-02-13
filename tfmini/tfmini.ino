#define HEADER (uint8_t)0x59

uint8_t packet[9];
uint16_t distance;
uint16_t strength;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  // setFrequency(1000);
}

void loop() {
  if (Serial3.available() >= 9) {
    packet[0] = Serial3.read();

    if (packet[0] == HEADER) {
      packet[1] = Serial3.read();

      if (packet[1] == HEADER) {
        for (int i = 2; i < 9; ++i) {
          packet[i] = Serial3.read();
        }

        uint8_t check = packet[0];
        for (int i = 1; i < 8; ++i) {
          check += packet[i];
        }

        if (check == packet[8]) {
          strength = ((uint16_t)packet[5] << 8) + (uint16_t)packet[4];

          if ((strength >= 200) && (strength != 0xFFFF)) {
            distance = ((uint16_t)packet[3] << 8) + (uint16_t)packet[2];

            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print("cm  Strength: ");
            Serial.println(strength);
          }
          else {
            Serial.println("Unreliable signal strength.");
          }
        }
      }
    }
  }
}

// bool setFrequency(const uint8_t freq) {
//   if (freq > 1000 || freq < 1)
//     return false;

//   uint16_t outputCycle = 1000 / freq;
//   uint8_t LL = (uint8_t)outputCycle;
//   uint8_t HH = (uint8_t)(outputCycle >> 8);
//   uint8_t config[8]{ 0x42, 0x57, 0x02, 0x00, LL, HH, 0X00, 0X07 };

//   for (int i = 0; i < 8; ++i) {
//     Serial.write((config[i]));
//   }

//   int i = 0;
//   while (i < 8) {
//     if (Serial3.available()) {
//       uint8_t d = Serial3.read();
//       Serial.print(d, HEX);
//       Serial.print(' ');
//       ++i;
//     }
//   }

//   delay(100);
//   return true;
// }