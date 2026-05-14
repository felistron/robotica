#include <Servo.h>

/*
  IK 3GDL (base + hombro + codo) con selección de rama CONTINUA,
  preferencia "codo hacia arriba" (elbowUp) siempre que sea posible.
  + Control de pinza en pin 9.
  + Movimiento interpolado (suavizado de velocidad).
  + Protocolo HANDSHAKE para definir contenedores y pick & place automático.

  Comandos Serial:
    CFG,BEGIN                                          → inicia configuración
    CFG,CONT,{color},{idx},{x_cm},{y_cm},{z_cm}        → define contenedor
    CFG,END                                            → finaliza configuración
    OBJ,{x_cm},{y_cm},{z_cm},{container_idx}           → pick & place (Python)
    "x y z"                                            → mover TCP a coordenadas en mm (manual)
    "g <100-180>"                                      → mover pinza (100=cerrada, 180=abierta)
    "go" / "gc"                                        → abrir / cerrar pinza
    "spd <1-50>"                                       → cambiar velocidad (ms por paso, default 10)
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
const int GRIPPER_OPEN   = 100;
const int GRIPPER_CLOSED = 180;

// ---- Contenedores recibidos por CFG ----
struct Contenedor {
  float x, y, z;  // coordenadas en cm (convertidas a mm para IK)
  bool valido;
};

const int MAX_CONT = 8;
Contenedor contenedores[MAX_CONT];
int numContenedores = 0;
bool cfgLista       = false;
bool leyendoCfg     = false;

// ---- Constantes de timing ----
const unsigned long MOVE_HOLD   = 1000;  // ms entre movimientos de brazo
const unsigned long GRIPPER_HOLD = 500;  // ms para cerrar/abrir pinza
const float SAFE_APPROACH_OFFSET_MM = 50.0f;  // altura extra para evitar choques al acercarse

// ---- Geometría (mm) ----
const float d1 = 100.0f;
const float L2 = 67.0f;
const float Le = 93.0f;

// ---- Interpolación ----
// Posiciones actuales de cada servo (se actualizan tras cada movimiento)
int currentCmd[3]    = {0, 0, 0};
int currentGripper   = 100;  // posición actual de la pinza (100=cerrada, 180=abierta)
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
  target = (int)clampf(target, GRIPPER_OPEN, GRIPPER_CLOSED);
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

// ---------- Helpers para HOME y Flush --------
void goHome() {
  int homeCmd[3] = {
    modelToServoCmd(0, 0.0f),
    modelToServoCmd(1, 0.0f),
    modelToServoCmd(2, 0.0f)
  };
  moveArmSmooth(homeCmd);
  Serial.println(F("Home: regresando a posición inicial"));
}

void flushSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// ---------- CSV Parsing --------
// Devuelve el campo n (base 0) de una cadena separada por comas
String csvField(const String &s, int n) {
  int start = 0, commas = 0;
  for (int i = 0; i <= (int)s.length(); i++) {
    if (i == (int)s.length() || s[i] == ',') {
      if (commas == n) return s.substring(start, i);
      commas++;
      start = i + 1;
    }
  }
  return "";
}

// Cuenta el número de campos separados por coma
int csvCount(const String &s) {
  int c = 1;
  for (int i = 0; i < (int)s.length(); i++)
    if (s[i] == ',') c++;
  return c;
}

// ---------- Configuración de Contenedores --------
void procesarCFG(const String &line) {
  // CFG,BEGIN
  if (line == "CFG,BEGIN") {
    leyendoCfg     = true;
    cfgLista       = false;
    numContenedores = 0;
    for (int i = 0; i < MAX_CONT; i++) contenedores[i].valido = false;
    Serial.println(F("CFG: inicio de configuración"));
    return;
  }

  // CFG,END
  if (line == "CFG,END") {
    if (!leyendoCfg) { 
      Serial.println(F("WARN: CFG,END sin CFG,BEGIN")); 
      return; 
    }
    leyendoCfg = false;
    cfgLista   = true;
    Serial.print(F("CFG: listo con "));
    Serial.print(numContenedores);
    Serial.println(F(" contenedor(es)"));
    return;
  }

  // CFG,CONT,{color},{índice},{x_cm},{y_cm},{z_cm}
  if (line.startsWith("CFG,CONT,") && leyendoCfg) {
    if (csvCount(line) != 7) {
      Serial.println(F("WARN: CFG,CONT mal formado"));
      return;
    }
    String color  = csvField(line, 2);  color.toLowerCase(); color.trim();
    int    idx    = csvField(line, 3).toInt();
    float  px_cm  = csvField(line, 4).toFloat();
    float  py_cm  = csvField(line, 5).toFloat();
    float  pz_cm  = csvField(line, 6).toFloat();
    if (idx < 0 || idx >= MAX_CONT) {
      Serial.println(F("WARN: índice contenedor fuera de rango"));
      return;
    }
    
    contenedores[idx].x      = px_cm;    // Store in cm, convert to mm during use
    contenedores[idx].y      = py_cm;
    contenedores[idx].z      = pz_cm;
    contenedores[idx].valido = true;
    if (idx >= numContenedores) numContenedores = idx + 1;

    Serial.print(F("CFG: cont "));
    Serial.print(idx);
    Serial.print(F(" ("));
    Serial.print(color);
    Serial.print(F(") @ ("));
    Serial.print(px_cm);
    Serial.print(F(","));
    Serial.print(py_cm);
    Serial.print(F(","));
    Serial.print(pz_cm);
    Serial.println(F(") cm"));
    return;
  }

  Serial.print(F("WARN: trama CFG desconocida → "));
  Serial.println(line);
}

// Busca un contenedor por índice
bool buscarContenedor(int idx, float &x_mm, float &y_mm, float &z_mm) {
  if (idx < 0 || idx >= MAX_CONT || !contenedores[idx].valido) {
    return false;
  }
  x_mm = contenedores[idx].x * 10.0f;  // Convert cm to mm
  y_mm = contenedores[idx].y * 10.0f;
  z_mm = contenedores[idx].z * 10.0f;
  return true;
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

// Parse CSV format: OBJ,x_cm,y_cm,z_cm,container_id
// Returns true if successful, sets x_mm, y_mm, z_mm, container_id
bool parseOBJCommand(const String &line, float &x_mm, float &y_mm, float &z_mm, int &container_id) {
  if (!line.startsWith("OBJ,")) return false;
  
  String csv = line.substring(4);
  int idx = 0, start = 0;
  float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int cont_id = -1;
  
  // Parse: x_cm,y_cm,z_cm,container_id
  for (int i = 0; i <= (int)csv.length(); i++) {
    if (i == (int)csv.length() || csv[i] == ',') {
      if (i > start) {
        String token = csv.substring(start, i);
        if (idx < 3) {
          values[idx] = token.toFloat();
        } else if (idx == 3) {
          cont_id = token.toInt();
        }
        idx++;
      }
      start = i + 1;
    }
  }
  
  if (idx != 4) return false;  // Must have exactly 4 comma-separated values
  
  x_mm = values[0] * 10.0f;     // Convert cm to mm
  y_mm = values[1] * 10.0f;     // Convert cm to mm
  z_mm = values[2] * 10.0f;     // Convert cm to mm
  container_id = cont_id;
  
  return true;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(SERVO_ZERO[i]);
  }
  gripper.attach(GRIPPER_PIN);
  gripper.write(GRIPPER_OPEN);
  currentGripper = GRIPPER_OPEN;

  Serial.println(F("=== Brazo 3DOF + Pinza ==="));
  Serial.println(F("Esperando configuración (CFG,BEGIN/END)"));
  delay(100);
  Serial.println(F("READY"));  // Signal to Python that we're ready for config
}

void loop() {
  String line;
  if (!readLine(line)) return;
  line.trim();
  if (!line.length()) return;

  // ---- Tramas de configuración ----
  if (line.startsWith("CFG,")) {
    procesarCFG(line);
    return;
  }

  // ---- Velocidad ----
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

  // ---- Pinza (manual) ----
  if (line.equalsIgnoreCase("go")) { 
    moveGripperSmooth(GRIPPER_OPEN); 
    return; 
  }
  if (line.equalsIgnoreCase("gc")) { 
    moveGripperSmooth(GRIPPER_CLOSED); 
    return; 
  }
  if (line.length() >= 3 && line.charAt(0) == 'g' && line.charAt(1) == ' ') {
    moveGripperSmooth(line.substring(2).toInt());
    return;
  }

  // ---- OBJ command from Python (pick & place) ----
  float x_mm, y_mm, z_mm;
  int container_id;
  if (parseOBJCommand(line, x_mm, y_mm, z_mm, container_id)) {
    if (!cfgLista) {
      Serial.println(F("OBJ: WARN - configuración no completada, ignorado"));
      return;
    }

    // Validate workspace bounds (mm)
    const float X_MIN = -400.0f, X_MAX = 400.0f;
    const float Y_MIN = -400.0f, Y_MAX = 400.0f;
    const float Z_MIN = 0.0f, Z_MAX = 300.0f;
    
    if (x_mm < X_MIN || x_mm > X_MAX || y_mm < Y_MIN || y_mm > Y_MAX || z_mm < Z_MIN || z_mm > Z_MAX) {
      Serial.print(F("OBJ: WARN - fuera de limites: x="));
      Serial.print(x_mm, 1); Serial.print(F(" y="));
      Serial.print(y_mm, 1); Serial.print(F(" z="));
      Serial.print(z_mm, 1); Serial.print(F(" cont="));
      Serial.println(container_id);
      return;
    }

    float approach_z_mm = z_mm + SAFE_APPROACH_OFFSET_MM;
    if (approach_z_mm > Z_MAX) {
      approach_z_mm = Z_MAX;
    }

    // 1. Move to a safe approach above the object
    float q[3];
    bool elbowUpUsed = true;
    if (!inverseKinematicsPreferElbowUp(x_mm, y_mm, approach_z_mm, q, elbowUpUsed)) {
      Serial.print(F("OBJ: WARN - punto de aproximación inalcanzable en ("));
      Serial.print(x_mm, 1); Serial.print(F(",");
      Serial.print(y_mm, 1); Serial.print(F(",");
      Serial.print(approach_z_mm, 1); Serial.println(F(")"));
      return;
    }

    int target[3] = {
      modelToServoCmd(0, q[0]),
      modelToServoCmd(1, q[1]),
      modelToServoCmd(2, q[2])
    };

    moveArmSmooth(target);
    Serial.print(F("OBJ: movido a aproximación en ("));
    Serial.print(x_mm, 1); Serial.print(F(","));
    Serial.print(y_mm, 1); Serial.print(F(","));
    Serial.print(approach_z_mm, 1); Serial.println(F(")"));
    delay(MOVE_HOLD);

    // 1b. Descend to object and grasp
    if (!inverseKinematicsPreferElbowUp(x_mm, y_mm, z_mm, q, elbowUpUsed)) {
      Serial.print(F("OBJ: WARN - objeto inalcanzable al descender en ("));
      Serial.print(x_mm, 1); Serial.print(F(","));
      Serial.print(y_mm, 1); Serial.print(F(","));
      Serial.print(z_mm, 1); Serial.println(F(")"));
      goHome();
      flushSerial();
      return;
    }

    target[0] = modelToServoCmd(0, q[0]);
    target[1] = modelToServoCmd(1, q[1]);
    target[2] = modelToServoCmd(2, q[2]);

    moveArmSmooth(target);
    Serial.print(F("OBJ: movido a objeto en ("));
    Serial.print(x_mm, 1); Serial.print(F(","));
    Serial.print(y_mm, 1); Serial.print(F(","));
    Serial.print(z_mm, 1); Serial.println(F(")"));
    delay(MOVE_HOLD);

    // 2. Close gripper
    moveGripperSmooth(GRIPPER_CLOSED);
    delay(GRIPPER_HOLD);

    // 2b. Move back up before traveling to container
    if (!inverseKinematicsPreferElbowUp(x_mm, y_mm, approach_z_mm, q, elbowUpUsed)) {
      Serial.println(F("OBJ: WARN - no se pudo volver a altura segura, retornando a home"));
      goHome();
      moveGripperSmooth(GRIPPER_OPEN);
      delay(GRIPPER_HOLD);
      flushSerial();
      return;
    }

    target[0] = modelToServoCmd(0, q[0]);
    target[1] = modelToServoCmd(1, q[1]);
    target[2] = modelToServoCmd(2, q[2]);

    moveArmSmooth(target);
    Serial.print(F("OBJ: retirado a altura segura en ("));
    Serial.print(x_mm, 1); Serial.print(F(","));
    Serial.print(y_mm, 1); Serial.print(F(","));
    Serial.print(approach_z_mm, 1); Serial.println(F(")"));
    delay(MOVE_HOLD);

    // 3. Find container
    float dest_x_mm, dest_y_mm, dest_z_mm;
    if (!buscarContenedor(container_id, dest_x_mm, dest_y_mm, dest_z_mm)) {
      Serial.print(F("OBJ: WARN - contenedor "));
      Serial.print(container_id);
      Serial.println(F(" no configurado, retornando a home"));
      goHome();
      moveGripperSmooth(GRIPPER_OPEN);
      delay(GRIPPER_HOLD);
      flushSerial();
      return;
    }

    float container_approach_z_mm = dest_z_mm + SAFE_APPROACH_OFFSET_MM;
    if (container_approach_z_mm > Z_MAX) {
      container_approach_z_mm = Z_MAX;
    }

    // 4. Move to container above target
    if (!inverseKinematicsPreferElbowUp(dest_x_mm, dest_y_mm, container_approach_z_mm, q, elbowUpUsed)) {
      Serial.print(F("OBJ: WARN - contenedor "));
      Serial.print(container_id);
      Serial.print(F(" inalcanzable, retornando a home"));
      goHome();
      moveGripperSmooth(GRIPPER_OPEN);
      delay(GRIPPER_HOLD);
      flushSerial();
      return;
    }

    target[0] = modelToServoCmd(0, q[0]);
    target[1] = modelToServoCmd(1, q[1]);
    target[2] = modelToServoCmd(2, q[2]);

    moveArmSmooth(target);
    Serial.print(F("OBJ: movido a aproximación de contenedor "));
    Serial.print(container_id);
    Serial.print(F(" en ("));
    Serial.print(dest_x_mm / 10.0f, 1); Serial.print(F(","));
    Serial.print(dest_y_mm / 10.0f, 1); Serial.print(F(","));
    Serial.print(container_approach_z_mm / 10.0f, 1); Serial.println(F(")"));
    delay(MOVE_HOLD);

    // 4b. Descend to container and release
    if (!inverseKinematicsPreferElbowUp(dest_x_mm, dest_y_mm, dest_z_mm, q, elbowUpUsed)) {
      Serial.print(F("OBJ: WARN - contenedor "));
      Serial.print(container_id);
      Serial.println(F(" inalcanzable al descender, retornando a home"));
      goHome();
      moveGripperSmooth(GRIPPER_OPEN);
      delay(GRIPPER_HOLD);
      flushSerial();
      return;
    }

    target[0] = modelToServoCmd(0, q[0]);
    target[1] = modelToServoCmd(1, q[1]);
    target[2] = modelToServoCmd(2, q[2]);

    moveArmSmooth(target);
    Serial.print(F("OBJ: depositado en contenedor "));
    Serial.print(container_id);
    Serial.print(F(" en ("));
    Serial.print(dest_x_mm / 10.0f, 1); Serial.print(F(","));
    Serial.print(dest_y_mm / 10.0f, 1); Serial.print(F(","));
    Serial.print(dest_z_mm / 10.0f, 1); Serial.println(F(")"));
    delay(MOVE_HOLD);

    // 5. Open gripper
    moveGripperSmooth(GRIPPER_OPEN);
    delay(GRIPPER_HOLD);

    // 6. Return to home
    goHome();
    delay(MOVE_HOLD);
    flushSerial();

    Serial.println(F("OBJ: ciclo completo, listo"));
    return;
  }

  // ---- Manual IK command (x y z) ----
  float xyz[3];
  if (!parse3Floats(line, xyz)) {
    Serial.println(F("WARN: formato inválido"));
    Serial.println(F("  Uso: 'x y z' (mm manual)"));
    Serial.println(F("       'OBJ,x_cm,y_cm,z_mm,cont' (desde Python)"));
    Serial.println(F("       'g <0-180>' (pinza)"));
    Serial.println(F("       'go'/'gc' (abrir/cerrar)"));
    Serial.println(F("       'spd <1-50>' (velocidad)"));
    return;
  }

  float q[3];
  bool elbowUpUsed = true;
  if (!inverseKinematicsPreferElbowUp(xyz[0], xyz[1], xyz[2], q, elbowUpUsed)) {
    Serial.println(F("Manual: punto inalcanzable o fuera de limites"));
    return;
  }

  int target[3] = {
    modelToServoCmd(0, q[0]),
    modelToServoCmd(1, q[1]),
    modelToServoCmd(2, q[2])
  };

  moveArmSmooth(target);

  Serial.print(F("Manual: xyz(mm)="));
  Serial.print(xyz[0], 1); Serial.print(F(",");
  Serial.print(xyz[1], 1); Serial.print(F(",");
  Serial.print(xyz[2], 1);
  Serial.print(F(" | q(deg)="));
  Serial.print(q[0], 2); Serial.print(F(",");
  Serial.print(q[1], 2); Serial.print(F(",");
  Serial.print(q[2], 2);
  Serial.print(F(" | "));
  Serial.println(elbowUpUsed ? F("elbowUp") : F("elbowDown"));
}