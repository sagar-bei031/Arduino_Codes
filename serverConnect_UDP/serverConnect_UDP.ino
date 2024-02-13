// #include <WiFi.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

uint16_t x = 500;
uint16_t y = 350;
float theta = 0.0;

const char* ssid = "sim";
const char* password = "simsimsim";
const char* host = "192.168.43.139";
const int port = 9990;

WiFiUDP udp;

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
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {

    if (x > 10000)
    {
      x = 1000;
    }
    else {
      x += 10;
    }

    // Prepare the JSON object
    DynamicJsonDocument doc(1024);
    doc["x"] = x;
    doc["y"] = y;
    doc["theta"] = theta;

    // Convert the JSON object to a string
    String output;
    serializeJson(doc, output);

    // Send the JSON string to the server
    udp.beginPacket(host, port);
    udp.print(output);
    udp.endPacket();
  } else {
    Serial.println("WiFi not connected");
  }
  delay(50); // Adjust the delay as needed
}
