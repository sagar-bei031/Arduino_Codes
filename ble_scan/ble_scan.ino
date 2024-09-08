#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; // In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning for BLE devices...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // Create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Set active scanning
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // Less or equal setInterval value
}

void loop() {
  // Start scanning
  BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);
  Serial.printf("Devices found: %d\n", foundDevices->getCount());
  Serial.println("Scan done!");
  delay(2000); // Delay between scans
}
