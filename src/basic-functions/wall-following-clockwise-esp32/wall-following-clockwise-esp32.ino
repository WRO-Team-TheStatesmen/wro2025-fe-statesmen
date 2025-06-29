// Wall Following in Clockwise direction

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define IN1 13
#define IN2 12
#define ENA 14

#define RXD2 22
#define TXD2 23

TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

int counter = 0;

const float kp = -1;
const int servoPin = 25;
const int servoCenter = 138;

float target_direction = 0.0;
bool start_condition = false;

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

void moveServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulsewidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin, LOW);
  delay(20 - pulsewidth / 1000);
}

void steer(int direction) {
  moveServoAngle(servoCenter + direction);
}

void flushLidarBuffer(HardwareSerial &port) {
  while (port.available()) port.read();
}

int readLidar(HardwareSerial &port, const char *label) {
  uint8_t buf[9];
  unsigned long start_time = millis();
  while (millis() - start_time < 50) {
    if (port.available() >= 9) {
      port.readBytes(buf, 9);
      if (buf[0] == 0x59 && buf[1] == 0x59) {
        uint16_t distance = buf[2] | (buf[3] << 8);
        return distance;
      } else {
        flushLidarBuffer(port);
      }
    }
  }
}

void setup(void) {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  steer(0);
  myWire.begin(32, 33);
  if (!bno.begin()) {
    while (1);
  }
  delay(2000);
  bno.setExtCrystalUse(true);
}

void loop(void) {
  if (counter < 12)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);

    int distance_right = readLidar(Serial2, "Forward Check");
    Serial.print("DEBUG: ");
    Serial.println(distance_right);

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float direction = normalizeAngle(orientationData.orientation.x);

    float error = normalizeAngle(direction - target_direction);
    int steer_val = (int)(error * kp);
    steer_val = constrain(steer_val, -40, 40);
    steer(steer_val);

    if (distance_right >= 75) {
      target_direction += 90;
      target_direction = normalizeAngle(target_direction);

      while (true) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        direction = normalizeAngle(orientationData.orientation.x);
        float error = normalizeAngle(direction - target_direction);
        int steer_val = (int)(error * kp);
        steer_val = constrain(steer_val, -40, 40);
        steer(steer_val);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 250);
        if (abs(error) <= 5) break;
      }
      int afterturn_distance_right = 100;
      steer(0);
      if (counter < 11) {
        while (afterturn_distance_right >= 50)
        {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          analogWrite(ENA, 250);
          afterturn_distance_right = readLidar(Serial2, "Afterturn Check");
        }
      }
      
      counter++;
      
      for (int i = 0; i < 10; i++) {
        int flush_distance = readLidar(Serial2, "Start Check");
      }

      while (true) {
        flushLidarBuffer(Serial2);
        int dist = readLidar(Serial2, "Cooldown Check");
        if (dist < 50 && dist > 0) break;
        delay(10);
      }
    }
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
    delay(200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    while (1);
  }
}
