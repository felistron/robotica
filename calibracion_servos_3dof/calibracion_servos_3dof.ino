#include <Servo.h>

/*
  Calibrador manual de servos
  Envía por Serial:
    "0 90"  → servo 0 (base)   a 90°
    "1 45"  → servo 1 (hombro) a 45°
    "2 120" → servo 2 (codo)   a 120°
    "3 80" → servo 3 (pinza)   a 80°
*/

static const uint8_t SERVO_PINS[4] = {3, 5, 6, 9};
Servo servos[4];
int currentPos[4] = {90, 90, 90, 30};

bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { out = buf; buf = ""; return true; }
    if (c != '\r') buf += c;
  }
  return false;
}

void printStatus() {
  Serial.print(F("Posiciones actuales → base:"));
  Serial.print(currentPos[0]);
  Serial.print(F("  hombro:"));
  Serial.print(currentPos[1]);
  Serial.print(F("  codo:"));
  Serial.print(currentPos[2]);
  Serial.print(F("  pinza:"));
  Serial.println(currentPos[3]);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(currentPos[i]);
  }
  Serial.println(F("=== Calibrador manual ==="));
  Serial.println(F("Formato: <servo> <grados>   (ej: '1 45')"));
  Serial.println(F("Servos: 0=base  1=hombro  2=codo  3=pinza"));
  Serial.println(F("Rango: 0-180"));
  printStatus();
}

void loop() {
  String line;
  if (!readLine(line)) return;
  line.trim();
  if (!line.length()) return;

  int spaceIdx = line.indexOf(' ');
  if (spaceIdx < 0) {
    Serial.println(F("Formato invalido. Usa: <servo> <grados>  ej: '1 90'"));
    return;
  }

  int idx = line.substring(0, spaceIdx).toInt();
  int deg = line.substring(spaceIdx + 1).toInt();

  if (idx < 0 || idx > 3) {
    Serial.println(F("Servo invalido. Usa 0, 1, 2 o 3."));
    return;
  }
  if (deg < 0 || deg > 180) {
    Serial.println(F("Grados fuera de rango [0, 180]."));
    return;
  }

  servos[idx].write(deg);
  currentPos[idx] = deg;

  Serial.print(F("Servo "));
  Serial.print(idx);
  Serial.print(F(" → "));
  Serial.print(deg);
  Serial.println(F("°"));
  printStatus();
}