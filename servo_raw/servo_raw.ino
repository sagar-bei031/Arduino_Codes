uint8_t servoPin = 9;
uint16_t T = 10000;
uint16_t min_duty = 500+200;
uint16_t max_duty = 2500+100;
void setup()
{

  pinMode(servoPin, OUTPUT);
  setPos(0);
  delay(3000);
}

void loop()
{
  setPos(180);
}

void setPos(uint16_t pos)
{
    uint16_t signal = map(pos, 0, 180, min_duty, max_duty);
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(signal);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(T-signal);
}