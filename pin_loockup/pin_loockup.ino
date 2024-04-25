void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("\nSPI PINS");
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);

  Serial.println("\nI2C PINS");
  Serial.print("SCL: ");
  Serial.println(SCL);
  Serial.print("SDA: ");
  Serial.println(SDA);

  delay(100);
}
