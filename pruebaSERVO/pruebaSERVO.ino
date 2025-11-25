const int motorPin = 3;

void setup() {
  pinMode(motorPin, OUTPUT);

  analogWrite(motorPin, 60); // velocidad media
  delay(500);

  analogWrite(motorPin, 0);   // apagado
}

void loop() {
  // vacío
}
