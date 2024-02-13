#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <memory.h>
#include "crc.hpp"

#define LED_PIN 2  // D4
#define START_BYTE 0xA5

#define uart Serial

struct Odometry {
  float x = 50.0f;
  float y = 35.0f;
  float theta = 0.0f;
};

uint8_t packet[14];

const char* ssid = "simsim";
const char* password = "simsimsim";
const char* host = "10.42.0.1";
const int port = 9990;

WiFiUDP udp;
Odometry odom;
CRC_Hash crc(7);

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connect to Wi-Fi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (uart.available() >= 14) {
    packet[0] = uart.read();
    if (packet[0] == START_BYTE) {
      for (int i=1; i<14; ++i)
      {
        packet[i] = uart.read();
      }

      uint8_t hash = crc.get_Hash(&packet[1], 12);
      if (hash == packet[13]) {

        memcpy(&odom, &packet[1], 12);
        odom.x = round2(odom.x * 100.0f + 50.0f);
        odom.y = round2(odom.y * 100.0f + 35.0f);
        odom.theta = round2(odom.theta * 180 / PI);

        Serial.print("Odom:: ");
        Serial.print(odom.x);
        Serial.print("  ");
        Serial.print(odom.y);
        Serial.print("  ");
        Serial.println(odom.theta);
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);

    // Prepare the JSON object
    DynamicJsonDocument doc(1024);
    doc["x"] = odom.x;
    doc["y"] = odom.y;
    doc["theta"] = odom.theta;

    // Convert the JSON object to a string
    String output;
    serializeJson(doc, output);

    // Send the JSON string to the server
    udp.beginPacket(host, port);
    udp.print(output);
    udp.endPacket();
  } else {
    Serial.println("WiFi not connected");
    digitalWrite(LED_PIN, HIGH);
  }
}

inline float round2(float val) {
  return (float)((int32_t)(val * 100.0f) / 100.0f);
}
