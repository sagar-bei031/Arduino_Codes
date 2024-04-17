
// TCS230 or TCS3200 pins wiring to Arduino
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define sensorOut 12

// Stores frequency read by the photodiodes
int redFrequency = 0;
int blueFrequency = 0;

// Stores the red. green and blue colors
int redColor = 0;
int blueColor = 0;

int diff = 0;

// diff value compare 
const int DRED = 15;
const int DPURPLE = 10;

void setup() {
  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);

  // Setting frequency scaling to 100%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  // Begins serial communication
  Serial.begin(9600);
}

void loop() {
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);

   
  // Printing the RED (R) value
  Serial.print(" R = ");
  Serial.print (  redFrequency );

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);

  // Printing the BLUE (B) value
  Serial.print(" B = ");
  Serial.print (  blueFrequency );
  Serial.print("  ");
  
  // Checks the current detected color and prints
  // a message in the serial monitor
  if (redFrequency >= 50 && blueFrequency >= 50) {
    Serial.println("Ball not detected");
  } else {

    diff = blueFrequency - redFrequency;
    // Serial.print("  DF = ");
    // Serial.print( diffF );

    if (blueFrequency < redFrequency) {
      Serial.println("Blue ball detected. ");
    } else {
      if (diff > DRED) {
        Serial.print("Red ball detected. Diff:");
        Serial.println(diff);
      } else if (diff < DPURPLE) {
        Serial.print("Purple ball detected: Diff:");
        Serial.println(diff);
      } else {
        Serial.print("Error::cannot find color. Diff:");
        Serial.println(diff);
      }
    }
  }

  delay(100);

}
