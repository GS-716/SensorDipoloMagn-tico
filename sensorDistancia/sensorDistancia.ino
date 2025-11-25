#include <Wire.h>
#include <VL53L1X.h>

VL53L1X tof;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  tof.setTimeout(500);

  if (!tof.init()) {
    Serial.println("No se detecta VL53L1X");
    while (1) {}
  }

  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(30000);
  tof.startContinuous(30);
}

void loop() {
  int d = tof.read();
  if (!tof.timeoutOccurred()) {
    Serial.println(d);
  }
  delay(50);
}
