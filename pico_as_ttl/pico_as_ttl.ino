#ifdef ARDUINO_ARCH_MBED_RP2040
// For the Arduino MBedOS Core, we must create Serial2
UART Serial2(8, 9, NC, NC);
#endif

uint32_t last_receive = 0;

void blink_led()
{
  if (millis() - last_receive >= 30)
  {
    digitalWrite(25, !(digitalRead(25)));
    last_receive = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(25, OUTPUT);
}

void loop() {
  if (Serial.available())
  {
    uint8_t b = Serial.read();
    Serial2.write(b);
    blink_led();
  }
  if ( Serial2.available())
  {
      uint8_t b = Serial2.read()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          ;
      Serial.write(b);
      blink_led();
  }
  if (millis() - last_receive > 100)
  {
    digitalWrite(25, HIGH);
  }
}
