const int servoPin = 25;

void setup() {
  pinMode(servoPin, OUTPUT);
  moveServoAngle(90);
}

void loop() {}

void moveServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulsewidth = map(angle, 0, 180, 500, 2400);
  for (int i = 0; i < 50; i++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - pulsewidth);
  }
}
