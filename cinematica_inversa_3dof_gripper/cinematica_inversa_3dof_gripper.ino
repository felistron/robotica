#include <Servo.h>

/*
  IK 3GDL (base + hombro + codo) con selección de rama CONTINUA,
  preferencia "codo hacia arriba" (elbowUp) siempre que sea posible.
  + Control de pinza en pin 9.
  + Movimiento interpolado (suavizado de velocidad).

  Comandos Serial:
    "x y z"       → mover TCP a coordenadas en mm
    "g <30-80>"   → mover pinza (0=cerrada, 180=abierta)
    "go"          → abrir pinza
    "gc"          → cerrar pinza
    "spd <1-50>"  → cambiar velocidad (ms por paso, default 20)

  TODO: Conectar con programa Python que envíe comandos Serial.
*/

static const uint8_t SERVO_PINS[3] = {3, 5, 6};
static const uint8_t GRIPPER_PIN   = 9;

Servo servos[3];
Servo gripper;

// ---- Calibración brazo ----
const int SERVO_ZERO[3] = {0, 30, 90};
const int SERVO_SIGN[3] = {+1, +1, -1};
const int SERVO_MIN[3]  = {0, 0, 0};
const int SERVO_MAX[3]  = {180, 180, 180};

// ---- Calibración pinza ----
const int GRIPPER_OPEN   = 30;
const int GRIPPER_CLOSED = 80;

// ---- Geometría (mm) ----
const float d1 = 90.0f;
const float L2 = 67.0f;
const float Le = 93.0f;

// ---- Interpolación ----
// Posiciones actuales de cada servo (se actualizan tras cada movimiento)
int currentCmd[3]    = {0, 0, 0};
int currentGripper   = 30;
int msPerStep        = 10;  // ms entre cada paso de 1°. Más alto = más lento.

static inline float rad2deg(float r) { return r * 57.295779513082320876f; }

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int modelToServoCmd(int i, float qModelDeg) {
  float raw = (float)SERVO_ZERO[i] + (float)SERVO_SIGN[i] * qModelDeg;
  float cmd = clampf(raw, SERVO_MIN[i], SERVO_MAX[i]);
  return (int)lround(cmd);
}

bool cmdsInRangeRaw(const float qDeg[3]) {
  for (int i = 0; i < 3; i++) {
    float raw = (float)SERVO_ZERO[i] + (float)SERVO_SIGN[i] * qDeg[i];
    if (raw < SERVO_MIN[i] || raw > SERVO_MAX[i]) return false;
  }
  return true;
}

bool solveIKBranch(float x, float y, float z, bool elbowUp, float qDeg[3]) {
  float q1 = atan2(y, x);
  float r  = sqrt(x * x + y * y);
  float zp = z - d1;

  float D = (r * r + zp * zp - L2 * L2 - Le * Le) / (2.0f * L2 * Le);
  if (D < -1.0f || D > 1.0f) return false;

  float s  = sqrt(1.0f - D * D);
  float q3 = elbowUp ? atan2(+s, D) : atan2(-s, D);

  float k1 = L2 + Le * cos(q3);
  float k2 = Le * sin(q3);
  float q2 = atan2(zp, r) - atan2(k2, k1);

  qDeg[0] = rad2deg(q1);
  qDeg[1] = rad2deg(q2);
  qDeg[2] = rad2deg(q3);
  return true;
}

bool inverseKinematicsPreferElbowUp(float x, float y, float z, float qOut[3], bool &usedElbowUp) {
  float qUp[3], qDown[3];

  bool okUp  = solveIKBranch(x, y, z, true,  qUp);
  bool upOK  = okUp && cmdsInRangeRaw(qUp);

  if (upOK) {
    usedElbowUp = true;
    qOut[0] = qUp[0]; qOut[1] = qUp[1]; qOut[2] = qUp[2];
    return true;
  }

  bool okDown = solveIKBranch(x, y, z, false, qDown);
  bool downOK = okDown && cmdsInRangeRaw(qDown);

  if (downOK) {
    usedElbowUp = false;
    qOut[0] = qDown[0]; qOut[1] = qDown[1]; qOut[2] = qDown[2];
    return true;
  }

  return false;
}

// ---------- Interpolación ----------
/*
  Mueve los 3 servos del brazo juntos desde currentCmd[] hasta target[].
  Todos terminan al mismo tiempo (el que más recorre marca el ritmo).
*/
void moveArmSmooth(int target[3]) {
  int delta[3];
  int maxDelta = 0;
  for (int i = 0; i < 3; i++) {
    delta[i] = target[i] - currentCmd[i];
    if (abs(delta[i]) > maxDelta) maxDelta = abs(delta[i]);
  }
  if (maxDelta == 0) return;

  for (int step = 1; step <= maxDelta; step++) {
    for (int i = 0; i < 3; i++) {
      int pos = currentCmd[i] + (int)round((float)delta[i] * step / maxDelta);
      servos[i].write(pos);
    }
    delay(msPerStep);
  }

  for (int i = 0; i < 3; i++) currentCmd[i] = target[i];
}

void moveGripperSmooth(int target) {
  target = (int)clampf(target, 0, 180);
  int delta = target - currentGripper;
  int steps = abs(delta);
  for (int step = 1; step <= steps; step++) {
    int pos = currentGripper + (int)round((float)delta * step / steps);
    gripper.write(pos);
    delay(msPerStep);
  }
  currentGripper = target;
  Serial.print(F("Pinza → "));
  Serial.print(target);
  Serial.println(F("°"));
}

// ---------- Serial ----------
bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { out = buf; buf = ""; return true; }
    if (c != '\r') buf += c;
  }
  return false;
}

bool parse3Floats(const String &line, float out[3]) {
  int idx = 0, start = 0;
  for (int i = 0; i < (int)line.length(); i++) {
    if (line[i] == ' ' || line[i] == '\t') {
      if (i > start) out[idx++] = line.substring(start, i).toFloat();
      while (i + 1 < (int)line.length() && (line[i+1] == ' ' || line[i+1] == '\t')) i++;
      start = i + 1;
      if (idx > 3) return false;
    }
  }
  if (start < (int)line.length()) out[idx++] = line.substring(start).toFloat();
  return idx == 3;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(SERVO_ZERO[i]);
  }
  gripper.attach(GRIPPER_PIN);
  gripper.write(GRIPPER_OPEN);

  Serial.println(F("IK 3GDL + pinza lista"));
  Serial.println(F("  Brazo:  x y z (mm)     ej: '100 0 150'"));
  Serial.println(F("  Pinza:  g <0-180>       ej: 'g 90'"));
  Serial.println(F("          go              abrir"));
  Serial.println(F("          gc              cerrar"));
  Serial.print  (F("  Vel:    spd <1-50>      actual: "));
  Serial.print  (msPerStep);
  Serial.println(F(" ms/paso"));
}

void loop() {
  String line;
  if (!readLine(line)) return;
  line.trim();
  if (!line.length()) return;

  // --- Velocidad ---
  if (line.startsWith("spd ")) {
    int v = line.substring(4).toInt();
    if (v < 1 || v > 50) {
      Serial.println(F("Rango valido: 1 (rapido) a 50 (lento)"));
    } else {
      msPerStep = v;
      Serial.print(F("Velocidad → "));
      Serial.print(msPerStep);
      Serial.println(F(" ms/paso"));
    }
    return;
  }

  // --- Pinza ---
  if (line.equalsIgnoreCase("go")) { moveGripperSmooth(GRIPPER_OPEN);   return; }
  if (line.equalsIgnoreCase("gc")) { moveGripperSmooth(GRIPPER_CLOSED); return; }
  if (line.length() >= 3 && line.charAt(0) == 'g' && line.charAt(1) == ' ') {
    moveGripperSmooth(line.substring(2).toInt());
    return;
  }

  // --- IK ---
  float xyz[3];
  if (!parse3Floats(line, xyz)) {
    Serial.println(F("Formato invalido. Usa: 'x y z'  |  'g <deg>'  |  'go'/'gc'  |  'spd <1-50>'"));
    return;
  }

  float q[3];
  bool elbowUpUsed = true;
  if (!inverseKinematicsPreferElbowUp(xyz[0], xyz[1], xyz[2], q, elbowUpUsed)) {
    Serial.println(F("No hay solucion (inalcanzable o fuera de limites)."));
    return;
  }

  int target[3] = {
    modelToServoCmd(0, q[0]),
    modelToServoCmd(1, q[1]),
    modelToServoCmd(2, q[2])
  };

  moveArmSmooth(target);

  Serial.print(F("xyz(mm)= "));
  Serial.print(xyz[0], 1); Serial.print(F(", "));
  Serial.print(xyz[1], 1); Serial.print(F(", "));
  Serial.print(xyz[2], 1);
  Serial.print(F(" | q(deg)= "));
  Serial.print(q[0], 2); Serial.print(F(", "));
  Serial.print(q[1], 2); Serial.print(F(", "));
  Serial.print(q[2], 2);
  Serial.print(F(" | cmd= "));
  Serial.print(target[0]); Serial.print(F(", "));
  Serial.print(target[1]); Serial.print(F(", "));
  Serial.print(target[2]);
  Serial.println(elbowUpUsed ? F(" | elbowUp") : F(" | elbowDown"));
}