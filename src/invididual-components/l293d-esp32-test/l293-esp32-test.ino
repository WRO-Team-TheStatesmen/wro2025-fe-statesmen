// Program for running the motors via the L293D motor driver IC on an ESP32 devkit

#define IN1 13
#define IN2 12
#define ENA 14

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  delay(1000);

  // Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  delay(2000);

  // Stop
  analogWrite(ENA, 0);
  delay(1000);

  // Backwards
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  delay(2000);

  // Stop
  analogWrite(ENA, 0);
}

void loop() {
  
}
