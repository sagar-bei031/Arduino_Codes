#define S1 26
#define S2 27

uint8_t sensor;
uint32_t last_transmit = 0;
uint32_t last_blink = 0;

void blink_led() {
  uint8_t now = millis();
  if (now - last_blink >= 20) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  sensor = 0;
  if (!digitalRead(S1))
    sensor |= 1;
  if (!digitalRead(S2))
    sensor |= 1 << 1;

  uint8_t now = millis();
  if (now - last_transmit >= 10) {
    // Serial.write(sensor);

    char c = '0';
    if (sensor == 3) {
      c = '3';
    } else if (sensor == 1) {
      c = '1';
    } else if (sensor == 2) {
      c = '2';
    }

    Serial.println(c);

    last_transmit = now;
  }

  if (sensor)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
}
