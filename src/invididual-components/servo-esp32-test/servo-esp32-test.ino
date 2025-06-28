// Program for rotating the steering MG90 servo motor connected to an ESP32 devkit [signal pin = 25]

const int servoPin = 25;
const int servoCenter = 138; // Servo center is the position of the servo at which the steering system is centered

void moveServoAngle(int angle){
  angle = constrain(angle,0,180);
  int pulsewidth = map(angle,0,180,500,2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin, LOW);
  delay(20 - pulsewidth/1000);
}

void setup() {
  pinMode(servoPin, OUTPUT);
}

void loop() {
  moveServoAngle(servoCenter);
}