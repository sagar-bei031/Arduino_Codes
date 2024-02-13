#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC
int throttleInput = 0; // Variable to hold throttle input (0-180)

void setup() {
  esc.attach(9);
  esc.writeMicroseconds(1000);  // Send a minimum throttle signal (1000 microseconds)
  delay(2000);
}

void loop() {
    esc.writeMicroseconds(2000);
    delay(1000);
}
