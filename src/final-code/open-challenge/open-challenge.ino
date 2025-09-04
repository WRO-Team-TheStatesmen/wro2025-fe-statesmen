// Including Required Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Defining Pins
#define IN1 13
#define IN2 12
#define ENA 14

#define RXD1 22 // Right
#define TXD1 23
#define RXD2 19 // Left
#define TXD2 21

// Initiating the BNO055
TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

// Defining Constants and Variables
int counter = 0;
int direction = 0; // 1 = clockwise, 2 = anticlockwise

const float kp = -1;
const int servoPin = 25;
const int servoCenter = 88;

float target_direction = 0.0;

volatile int distance_right = 0;
volatile int distance_left = 0;

const int button_pin = 3;

const int SPEED = 125;

// Defining Functions
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

void waitForOK() {
  Serial.println("Waiting for OK to start...");
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        input.trim();
        if (input.equalsIgnoreCase("OK")) {
          Serial.println("OK received. Starting...");
          break;
        }
        input = "";
      } else {
        input += c;
      }
    }
  }
}

void steer(int direction) {
  moveServoAngle(servoCenter + direction);
}

void updateTFLuna(HardwareSerial &sensor, volatile int &distanceVar) {
  while (sensor.available() >= 9) {
    if (sensor.read() == 0x59 && sensor.read() == 0x59) {
      byte low = sensor.read();
      byte high = sensor.read();
      int distance = (high << 8) + low;

      for (int i = 0; i < 5; i++) sensor.read();

      if (distance <= 500) {
        distanceVar = distance;
      }
    }
  }
}

// Setup
void setup(void) {
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1); // Right LiDAR
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Left LiDAR

  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(button_pin, INPUT);

  steer(0);
  myWire.begin(32, 33);

  if (!bno.begin()) {
    while (1); // Halt if sensor fails to initialize
  }

  delay(2000);
  bno.setExtCrystalUse(true);
  Serial.println("DEBUG: SETUP COMPLETE (Detect direction then use one LiDAR)");

  // waitForOK();
  while (digitalRead(button_pin) == HIGH) {}
  Serial.println("Button clicked");
}

// Main Loop
void loop(void) {
  while (direction == 0) {
    updateTFLuna(Serial1, distance_right);
    updateTFLuna(Serial2, distance_left);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, SPEED);

    Serial.print("Left: ");
    Serial.print(distance_left);
    Serial.print(" | Right: ");
    Serial.println(distance_right);

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float heading = normalizeAngle(orientationData.orientation.x);

    float error = normalizeAngle(heading - target_direction);
    int steer_val = (int)(error * kp);
    steer_val = constrain(steer_val, -30, 30);
    steer(steer_val);

    if (distance_right >= 100) {
      direction = 1;
      Serial.println("Direction chosen: CLOCKWISE");
    } else if (distance_left >= 100) {
      direction = 2;
      Serial.println("Direction chosen: ANTICLOCKWISE");
    }
  }

  if (counter < 12) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, SPEED);

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float heading = normalizeAngle(orientationData.orientation.x);

    float error = normalizeAngle(heading - target_direction);
    int steer_val = (int)(error * kp);
    steer_val = constrain(steer_val, -30, 30);
    steer(steer_val);

    if (direction == 1) {
      updateTFLuna(Serial1, distance_right);
      if (distance_right >= 80) {
        target_direction += 90;
        target_direction = normalizeAngle(target_direction);

        while (true) {
          updateTFLuna(Serial1, distance_right);
          bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          heading = normalizeAngle(orientationData.orientation.x);
          error = normalizeAngle(heading - target_direction);
          steer_val = (int)(error * kp);
          steer_val = constrain(steer_val, -30, 30);
          steer(steer_val);
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          analogWrite(ENA,  100);
          if (abs(error) <= 10) break;
        }

        counter++;
        steer(0);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, SPEED);
        delay(200);

        while (true) {
          updateTFLuna(Serial1, distance_right);
          if (distance_right < 50 && distance_right > 0) break;
          delay(10);
        }
      }
    }

    else if (direction == 2) {
      updateTFLuna(Serial2, distance_left);
      if (distance_left >= 80) {
        target_direction -= 90;
        target_direction = normalizeAngle(target_direction);

        while (true) {
          updateTFLuna(Serial2, distance_left);
          bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          heading = normalizeAngle(orientationData.orientation.x);
          error = normalizeAngle(heading - target_direction);
          steer_val = (int)(error * kp);
          steer_val = constrain(steer_val, -30, 30);
          steer(steer_val);
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          analogWrite(ENA,  100);
          if (abs(error) <= 10) break;
        }

        counter++;
        steer(0);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, SPEED);
        delay(200);

        while (true) {
          updateTFLuna(Serial2, distance_left);
          if (distance_left < 50 && distance_left > 0) break;
          delay(10);
        }
      }
    }
  }

  else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, SPEED);
    delay(1000);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 0);
    while (1);
  }
}
