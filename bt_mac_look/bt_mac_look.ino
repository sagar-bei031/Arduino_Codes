#include "esp_bt_main.h"

#include "esp_bt_device.h"

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void printDeviceAddress() {

  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
    if (i < 5){
      Serial.print(":");
    }
  }
}
void setup() {
  Serial.begin(115200);
   SerialBT.begin("ESP32 Bluetooth");
  printDeviceAddress();

}

void loop() {}