// Program for running the BNO055 IMU sensor connected to an ESP32 devkit [on SDA pin 32 and SCL pin 33]

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &myWire);

void setup() {
  Serial.begin(115200);
  myWire.begin(32, 33);

  if (!bno.begin()) {
    Serial.println("Error - BNO055 not detected");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized");
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Yaw: ");
  Serial.print(orientationData.orientation.x);
  Serial.print(" | Roll: ");
  Serial.print(orientationData.orientation.y);
  Serial.print(" | Pitch: ");
  Serial.println(orientationData.orientation.z);

  delay(500);
}
