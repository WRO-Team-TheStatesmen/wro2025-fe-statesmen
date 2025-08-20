#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

HardwareSerial tfLuna1(1);
HardwareSerial tfLuna2(2);
// HardwareSerial tfLuna3(3);


TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

volatile int distance1 = 100000;
volatile int distance2 = 100000;
volatile int distance3 = 100000;

#define IN1 13
#define IN2 12
#define ENA 14
const int servoPin = 25;
const int servoCenter = 101;

unsigned long lastServoWrite = 0;
int lastAngle = 0;

#define TRIG_PIN 5
#define ECHO_PIN 18

#define IR_PIN 16

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
  myWire.begin(32, 33);
  Serial.begin(115200);
  delay(500);

  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IR_PIN, INPUT);

  tfLuna1.begin(115200, SERIAL_8N1, 22, 23);
  tfLuna2.begin(115200, SERIAL_8N1, 19, 21); 
  // tfLuna3.begin(115200, SERIAL_8N1, 18, 5);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!bno.begin()) {
    // Serial.println("Error - BNO055 not detected");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  steer(0);
  analogWrite(ENA, 0);

  // Serial.println("Setup Complete");
}

void updateTFLuna(HardwareSerial &sensor, volatile int &distanceVar) {
  while (sensor.available() >= 9) {
    if (sensor.read() == 0x59 && sensor.read() == 0x59) {
      byte low = sensor.read();
      byte high = sensor.read();
      int distance = (high << 8) + low;

      for (int i = 0; i < 5; i++) sensor.read();

      if (distance <= 500)
      {
        distanceVar = distance;
      }
    }
  }
}

void updateUltrasonic(volatile int &distanceVar) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.0343 / 2;

  if (distance > 0 && distance <= 500) {
    distanceVar = distance;
  }
}

void loop() {
  updateTFLuna(tfLuna1, distance1);
  updateTFLuna(tfLuna2, distance2);
  // updateTFLuna(tfLuna3, distance3);
  updateUltrasonic(distance3);

  int IR = !digitalRead(IR_PIN);

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Serial.print((int) orientationData.orientation.x);
  Serial.print(",");
  Serial.print(distance2); // Left
  Serial.print(",");
  Serial.print(distance3); // Front
  Serial.print(",");
  Serial.print(distance1); // Right
  Serial.print(",");
  Serial.println(IR); // IR

  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) {
        int parts[5] = {0};
        int idx = 0;
        char *token = strtok((char*)input.c_str(), ",");
        while (token != NULL && idx < 5) {
          parts[idx++] = atoi(token);
          token = strtok(NULL, ",");
        }

        int speed = parts[0];
        bool direction = parts[1];
        int steerRaw = parts[2];

        digitalWrite(IN1, direction == 0 ? HIGH : LOW);
        digitalWrite(IN2, direction == 0 ? LOW : HIGH);
        analogWrite(ENA, speed);
        steer(steerRaw);
        lastAngle = steerRaw;
      }
      input = "";
    } else {
      input += c;
    }
  }

  if (millis() - lastServoWrite > 20) {
    steer(lastAngle);
    lastServoWrite = millis();
  }

  delay(50);
}
