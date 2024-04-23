uint32_t t1;
uint32_t t2;
size_t n;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial.println("Reset");
  n = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (n == 0) {
    t1 = micros();
  }

  if (Serial3.available()) {
    uint8_t d = Serial3.read();
    ++n;

    if (n == 1000) {
      t2 = micros();

      double bytesPerSec = (double)n / (t2 - t1) * 1000000;
      Serial.print("Bytes per second: ");
      Serial.println(bytesPerSec);
      
      n = 0;
    }
  }
}
