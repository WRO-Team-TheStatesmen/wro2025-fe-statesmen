// Driving in a straight line

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define IN1 13
#define IN2 12
#define ENA 14

TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

const float target_direction = 0;
const float kp = -1;

const int servoPin = 25;
const int servoCenter = 138;

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

void setup(void) 
{
  Serial.begin(115200);

  pinMode(servoPin, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  steer(0);

  myWire.begin(32, 33);

  if(!bno.begin())
  {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(2000); 
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float direction = orientationData.orientation.x;
  if (direction > 180)
  {
    direction -= 360;
  }

  float error = direction - target_direction;
  int steer_val = (int) error*kp;

  steer_val = constrain(steer_val, -40, 40);

  Serial.print("Steer: ");
  Serial.println(steer_val);
  steer(steer_val);
}
