#define IN1 13
#define IN2 12
#define ENA 14
const int servoPin = 25;
const int servoCenter = 96;

unsigned long lastServoWrite = 0;
int lastAngle = 0;

void moveServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulsewidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin, LOW);
  delayMicroseconds(20000 - pulsewidth);
}

void steer(int direction) {
  int angle = constrain(servoCenter + direction, 0, 180);
  moveServoAngle(angle);
}

void setup() {
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  steer(0);
  analogWrite(ENA, 0);
}

void loop() {
  static byte data[3];
  static int index = 0;

  while (Serial.available()) {
    data[index++] = Serial.read();
    if (index == 3) {
      int speed = data[0];
      bool direction = data[1];
      int steerRaw = (int8_t)data[2];

      digitalWrite(IN1, direction == 0 ? HIGH : LOW);
      digitalWrite(IN2, direction == 0 ? LOW : HIGH);
      analogWrite(ENA, speed);

      steer(steerRaw);
      lastAngle = steerRaw;

      index = 0;
    }
  }

  if (millis() - lastServoWrite > 20) {
    steer(lastAngle);
    lastServoWrite = millis();
  }
}
