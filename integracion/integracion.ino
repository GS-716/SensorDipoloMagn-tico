#include <Wire.h>
#include "DFRobot_BMM150.h"
#include <VL53L1X.h>
#include <Servo.h>
#include <math.h>

// ================== BMM150 ==================
#define BMM150_I2C_ADDR 0x13
DFRobot_BMM150_I2C bmm150(&Wire, BMM150_I2C_ADDR);

// ================== VL53L1X ==================
VL53L1X tof;

// ================== SERVO ==================
Servo servo;
const int pinServo = 9;
const int pinGate  = A3;
bool servoEnabled = false;

// ================== FONDO MAGNÉTICO ==================
float B0x = 0, B0y = 0, B0z = 0;
const int N_FONDO = 200;

// ================== TIMINGS ==================
const unsigned long SERVO_REFRESH_MS = 20;       // refresco típico servo (~50 Hz)
const unsigned long MOVE_SENSOR_PERIOD_MS = 60; // lectura ToF ~>=50ms (Long mode)

// Micro-pasos (modo pulsos con pausa)
const int BURST_MS = 20;     // micro-avance (>=20ms para que reciba PWM real)
const int PAUSE_MS = 120;    // pausa para medir tranquilo (>=60ms)
const int STEP_DEG = 1;      // grados por micro-avance

// ================== SERIAL NO BLOQUEANTE ==================
String cmdBuffer = "";

// ---------- medir fondo sin imán ----------
void medirFondo() {
  Serial.println("#Calibrando fondo magnetico...");
  Serial.println("#Retira el iman y no lo acerques hasta que termine.");

  B0x = B0y = B0z = 0;

  for (int i = 0; i < N_FONDO; i++) {
    sBmm150MagData_t m = bmm150.getGeomagneticData();
    B0x += m.x; B0y += m.y; B0z += m.z;
    delay(10);
  }

  B0x /= N_FONDO;
  B0y /= N_FONDO;
  B0z /= N_FONDO;

  Serial.println("#Fondo magnetico medido:");
  Serial.print("#B0x = "); Serial.print(B0x); Serial.println(" uT");
  Serial.print("#B0y = "); Serial.print(B0y); Serial.println(" uT");
  Serial.print("#B0z = "); Serial.print(B0z); Serial.println(" uT");
  Serial.println("#Listo.");
}

// ---------- leer sensores e imprimir UNA línea de datos ----------
void leerSensoresYPrint() {
  uint16_t distancia_mm = tof.read();
  if (tof.timeoutOccurred()) {
    Serial.println("#Timeout VL53L1X");
    return;
  }

  sBmm150MagData_t mag = bmm150.getGeomagneticData();
  if (isnan(mag.x) || isnan(mag.y) || isnan(mag.z)) {
    Serial.println("#Overflow BMM150: iman demasiado cerca.");
    return;
  }

  float dBx = mag.x - B0x;
  float dBy = mag.y - B0y;
  float dBz = mag.z - B0z;
  float dB_total = sqrt(dBx*dBx + dBy*dBy + dBz*dBz);

  // *** línea de datos (NO empieza con #) ***
  Serial.print("Distancia = ");
  Serial.print(distancia_mm);
  Serial.print(" mm\t|dB| (sin fondo) = ");
  Serial.print(dB_total, 2);
  Serial.println(" uT");
}

// ---------- chequeo rápido de OFF mientras se mueve ----------
bool offSolicitado() {
  // Si entra OFF durante el MOVE, lo detenemos al toque.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String tmp = cmdBuffer;
      cmdBuffer = "";
      tmp.trim();
      if (tmp == "OFF") return true;
      // Si llegó otro comando mientras movías, lo ignoramos por ahora
    } else {
      cmdBuffer += c;
    }
  }
  return false;
}

// ---------- mover por pulsos + medir SOLO en pausas ----------
void moverPorPulsos(int anguloObjetivo) {
  anguloObjetivo = constrain(anguloObjetivo, 0, 180);

  int actual = servo.read();
  int delta = anguloObjetivo - actual;

  // FIX delta==0: igual START/DONE
  if (delta == 0) {
    Serial.println("#MOVE_START");
    leerSensoresYPrint();
    Serial.println("#MOVE_DONE");
    return;
  }

  int dir = (delta > 0) ? 1 : -1;
  int pasosTotales = abs(delta);
  int nPulsos = (pasosTotales + STEP_DEG - 1) / STEP_DEG; // ceil

  Serial.println("#MOVE_START");

  for (int k = 0; k < nPulsos; k++) {

    // --- si pidieron OFF, abortar YA ---
    if (offSolicitado()) {
      servo.detach();
      digitalWrite(pinGate, LOW);
      servoEnabled = false;
      Serial.println("#MOVE_DONE");
      Serial.println("#Servo deshabilitado");
      return;
    }

    // ===== micro-avance (servo ON) =====
    if (!servo.attached()) servo.attach(pinServo);
    digitalWrite(pinGate, HIGH);
    servoEnabled = true;

    int nextAngle = actual + dir * STEP_DEG;
    if ((dir > 0 && nextAngle > anguloObjetivo) ||
        (dir < 0 && nextAngle < anguloObjetivo)) {
      nextAngle = anguloObjetivo;
    }

    servo.write(nextAngle);
    actual = nextAngle;

    delay(BURST_MS);

    // ===== STOP real (servo OFF) =====
    servo.detach();
    digitalWrite(pinGate, LOW);
    servoEnabled = false;

    // ===== pausa de medición (servo OFF, sí mide) =====
    unsigned long tPause0 = millis();
    while (millis() - tPause0 < (unsigned long)PAUSE_MS) {

      if (offSolicitado()) {
        servo.detach();
        digitalWrite(pinGate, LOW);
        servoEnabled = false;
        Serial.println("#MOVE_DONE");
        Serial.println("#Servo deshabilitado");
        return;
      }

      leerSensoresYPrint();
      delay(MOVE_SENSOR_PERIOD_MS);
    }

    if (actual == anguloObjetivo) break;
  }

  leerSensoresYPrint();
  Serial.println("#MOVE_DONE");

  servo.detach();
  digitalWrite(pinGate, LOW);
  servoEnabled = false;
}

// ---------- procesa un comando completo ----------
void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "ON") {
    if (!servo.attached()) servo.attach(pinServo);
    digitalWrite(pinGate, HIGH);
    servoEnabled = true;
    Serial.println("#Servo habilitado");
    return;
  }

  if (cmd == "OFF") {
    servo.detach();
    digitalWrite(pinGate, LOW);
    servoEnabled = false;
    Serial.println("#Servo deshabilitado");
    return;
  }

  if (!servoEnabled) {
    Serial.println("#IGNORADO: servo OFF");
    return;
  }

  if (cmd.startsWith("MOVE")) {
    int e1 = cmd.indexOf(' ');
    int e2 = cmd.indexOf(' ', e1 + 1);
    if (e1 < 0 || e2 < 0) {
      Serial.println("#Formato MOVE incorrecto. Usa: MOVE angulo tiempo_ms");
      return;
    }

    int angulo = cmd.substring(e1 + 1, e2).toInt();
    // tiempo se ignora en este modo pulsos controlado internamente

    moverPorPulsos(angulo);
    return;
  }

  Serial.print("#Comando desconocido: ");
  Serial.println(cmd);
}

// ---------- lectura serial NO bloqueante ----------
void leerComandosNoBloqueante() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      String cmd = cmdBuffer;
      cmdBuffer = "";
      procesarComando(cmd);
    } else {
      cmdBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // SUPER importante: evitar bloqueos largos
  Serial.setTimeout(5);   // <- por si algo usa readString en libs

  Wire.begin();

  // --- BMM150 ---
  Serial.println("#Inicializando BMM150...");
  while (bmm150.begin() != 0) {
    Serial.println("#ERROR BMM150. Revisa 3.3V / addr 0x13.");
    delay(1000);
  }
  bmm150.setPresetMode(BMM150_PRESETMODE_REGULAR);
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  Serial.println("#BMM150 OK.");

  // --- VL53L1X ---
  Serial.println("#Inicializando VL53L1X...");
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("#ERROR VL53L1X.");
    while (1) {}
  }
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);
  Serial.println("#VL53L1X OK.");

  // --- Servo ---
  servo.attach(pinServo);
  pinMode(pinGate, OUTPUT);
  digitalWrite(pinGate, LOW);
  servoEnabled = false;
  Serial.println("#Servo listo (OFF).");

  medirFondo();
}

void loop() {
  leerComandosNoBloqueante();
  // NO medimos aquí: solo dentro de MOVE / pausas
}
