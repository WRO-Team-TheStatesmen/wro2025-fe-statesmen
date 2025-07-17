// Steering while driving

#define IN1 13
#define IN2 12
#define ENA 14

const int servoPin = 25;
const int servoCenter = 96;

void moveServoAngle(int angle){
  angle = constrain(angle,0,180);
  int pulsewidth = map(angle,0,180,500,2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin, LOW);
  delay(20 - pulsewidth/1000);
}

void steer(int direction)
{
  moveServoAngle(servoCenter + direction);
}

void setup() {
  pinMode(servoPin, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  steer(0);
}

void loop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 100);
  steer(0);
  delay(1000);
  steer(30);
  delay(1000);
  steer(0);
  delay(1000);
  steer(-30);
  delay(1000);
}