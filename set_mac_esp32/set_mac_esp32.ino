#include <esp_system.h>

// Define your custom MAC address
const uint8_t custom_mac[6] = {0x28, 0xC1, 0x3C, 0x7D, 0xCC, 0xB6};

void setup() {
  // Set the custom MAC address
  esp_base_mac_addr_set(custom_mac);

  // Your setup code here
}

void loop() {
  // Your code here
}
