#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

HardwareSerial tfLuna1(1);
HardwareSerial tfLuna2(2);


TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

volatile int distance1 = 100000;
volatile int distance2 = 100000;

#define IN1 13
#define IN2 12
#define ENA 14

const int servoPin = 25;
const int servoCenter = 88;

unsigned long lastServoWrite = 0;
int lastAngle = 0;

const int button_pin = 26;

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

void waitForOK() {
  bool okReceived = false;
  String input = "";
  while (!okReceived) {
    Serial.println("OK");
    delay(500);

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        input.trim();
        if (input.equals("OK")) {
          okReceived = true;
          break;
        }
        input = "";
      } else {
        input += c;
      }
    }
  }
}

void setup() {
  myWire.begin(32, 33);
  Serial.begin(115200);
  delay(500);

  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  tfLuna1.begin(115200, SERIAL_8N1, 22, 23);
  tfLuna2.begin(115200, SERIAL_8N1, 19, 21); 

  if (!bno.begin()) {
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  steer(0);
  analogWrite(ENA, 0);

  // while (true)
  // {
  //   Serial.println(digitalRead(button_pin));
  // }
  Serial.println("DEBUG: INITIALIZED");
  bool flag = true;
  while (flag) {
    if (digitalRead(button_pin) == LOW) {
      Serial.println("DEBUG: BUTTON CLICK");
      unsigned long startTime = millis();

      // stay in loop while button is HIGH
      while (digitalRead(button_pin) == LOW) {
        Serial.println(millis() - startTime);
        if (millis() - startTime >= 2000) {
          Serial.println("Button held for 2 seconds. Continuing...");
          flag = false;
          break;
        }
      }
    }
  }

  Serial.println("Waiting for ok");
  waitForOK();
  Serial.println("Handshake Complete");
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

void loop() {
  updateTFLuna(tfLuna1, distance1);
  updateTFLuna(tfLuna2, distance2);

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Serial.print((int) orientationData.orientation.x);
  Serial.print(",");
  Serial.print(distance2); // Left
  Serial.print(",");
  Serial.println(distance1); // Right

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
