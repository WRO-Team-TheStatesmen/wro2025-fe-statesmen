// Program to get data from 2 TF-Luna LiDAR sensors connected to the ESP32 via UART connections

HardwareSerial tfLuna1(1);
HardwareSerial tfLuna2(2);

void setup() {
  Serial.begin(115200);
  delay(500);

  tfLuna1.begin(115200, SERIAL_8N1, 22, 23);
  tfLuna2.begin(115200, SERIAL_8N1, 19, 21);

  Serial.println("Setup Complete");
}

void readTFLuna(HardwareSerial &sensor, const char* label) {
  if (sensor.available()) {
    if (sensor.read() == 0x59) {
      if (sensor.read() == 0x59) {
        byte low = sensor.read();
        byte high = sensor.read();
        int distance = (high << 8) + low;

        sensor.read();
        sensor.read();
        sensor.read();
        sensor.read();
        sensor.read();

        Serial.print(label);
        Serial.print(" Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      }
    }
  }
}

void loop() {
  readTFLuna(tfLuna1, "Sensor 1");
  readTFLuna(tfLuna2, "Sensor 2");
}