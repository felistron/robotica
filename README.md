# Robotica - Sistema de Detección y Control

Sistema de detección de objetos en tiempo real con comunicación serial para control de un brazo robótico de 3 grados de libertad (3DOF) equipado con pinza.

## Descripción General

El proyecto integra:
- **Backend Flask**: Servidor web con WebSocket para comunicación en tiempo real
- **Detección visual**: Identificación de objetos por color utilizando OpenCV (espacio HSV)
- **Calibración ArUco**: Calibración del plano de trabajo mediante marcadores ArUco
- **Control serial**: Comunicación UART con microcontrolador Arduino para control del brazo y pinza
- **Interfaz web**: Dashboard interactivo en tiempo real con visualización de detecciones

## Estructura del Proyecto

```
robotica/
├── app.py                          # Servidor Flask principal con endpoints y WebSocket
├── serial_handler.py               # Manejador de comunicación serial con Arduino
├── detector.py                     # Módulo de detección de objetos y calibración
├── calibration_config.json         # Configuración de calibración (generado dinámicamente)
├── container_config.json           # Configuración de contenedores de destino
├── requirements.txt                # Dependencias Python
├── templates/
│   └── index.html                  # Interfaz web del dashboard
├── cinematica_inversa_3dof_gripper/
│   └── cinematica_inversa_3dof_gripper.ino  # Código principal del brazo robótico con IK
└── calibracion_servos_3dof/
    └── calibracion_servos_3dof.ino  # Calibrador manual de ángulos de servos
```

## Dependencias

```
Flask>=3.0,<4.0
Flask-SocketIO>=5.4,<6.0
opencv-contrib-python>=4.10,<5.0
numpy>=1.26,<3.0
simple-websocket>=1.0,<2.0
pyserial>=3.5,<4.0
```

## Comunicación Serial

### Protocolo de Comunicación

La comunicación con Arduino se realiza a través de UART a **115200 baud**, utilizando líneas de texto terminadas en `\n`.

#### Comandos de Prueba (directos al Arduino - no implementados aún desde Python)

Estos comandos se envían directamente al Arduino mediante herramientas seriales externas y están en fase de prueba. Serán reemplazados por comunicación serial desde Python en futuras versiones:

1. **Comando de movimiento TCP**
   ```
   x y z
   ```
   - `x, y, z`: Coordenadas en milímetros del punto TCP (Tool Center Point)
   - Ej: `150 100 200`

2. **Control de pinza**
   ```
   g <valor>
   ```
   - `valor`: Ángulo servo pinza (30=cerrada, 80=abierta)
   - Ejemplos: `go` (abrir), `gc` (cerrar), `g 55` (posición intermedia)

3. **Control de velocidad**
   ```
   spd <1-50>
   ```
   - `valor`: Milisegundos por paso (default: 20ms)

#### Mensajes desde Python → Arduino

1. **Handshake de Contenedores**
   ```
   CFG,BEGIN
   CFG,CONT,<color>,<indice>,<x_cm>,<y_cm>
   CFG,END
   ```
   - Configura ubicaciones de contenedores destino
   - Se envía cuando Arduino envía "READY" o se actualiza la configuración

2. **Envío de Detecciones**
   ```
   OBJ,<x_cm>,<y_cm>,<contenedor_id>
   ```
   - `x_cm, y_cm`: Coordenadas del objeto detectado en cm
   - `contenedor_id`: Índice del contenedor destino

#### Mensajes desde Arduino → Python

- **READY**: Arduino señala que está listo para recibir comandos de configuración
- Otros mensajes se imprimen en la consola para debugging

### Características de `SerialHandler`

- **Thread-safe**: Utiliza locks para operaciones concurrentes
- **Cola de salida**: Encola mensajes y los envía de forma sincrónica
- **Lectura asincrónica**: Lee datos disponibles en el puerto sin bloquear
- **Callbacks**: Permite registrar callbacks para procesar líneas entrantes
- **Auto-reconexión**: Reinenta conexión automáticamente si se desconecta

### Métodos Principales

```python
connect(port: str, baud: int = 115200)  # Conectar al puerto serial
disconnect()                             # Desconectar
is_ready() -> bool                       # Estado del handler (conectado y listo)
enqueue_line(line: str)                  # Encolar línea para enviar
enqueue_lines(lines: Iterable[str])      # Encolar múltiples líneas
send_detection(track_id, figura, color, x_cm, y_cm) -> bool  # Enviar detección
set_line_callback(callback)              # Registrar callback para líneas entrantes
get_status() -> dict                     # Obtener estado de conexión y cola
list_ports() -> List[str]                # Listar puertos seriales disponibles
```

## TODOs Identificados

### Arduino (cinematica_inversa_3dof_gripper.ino)
- **Línea 16**: `TODO: Conectar con programa Python que envíe comandos Serial.`
  - Estado: El código Python ya está implementado y envía comandos via serial
  - Acción pendiente: Validar y documentar el protocolo completo en Arduino

## Configuración

### calibration_config.json
Almacena parámetros de calibración del plano de trabajo:
- Dimensiones del plano (`ancho`, `alto` en cm)
- Origen del sistema de coordenadas (`x`, `y` en cm)
- Rotación de ejes (`invertir_eje_y`)
- IDs de marcadores ArUco por esquina

### container_config.json
Almacena ubicaciones de contenedores destino:
```json
{
  "contenedores": {
    "rojo": {"indice": 0, "x_cm": 12.0, "y_cm": 8.0},
    "azul": {"indice": 1, "x_cm": 28.0, "y_cm": 8.0}
  }
}
```

## Instalación y Ejecución

```bash
# Instalar dependencias
pip install -r requirements.txt

# Ejecutar servidor
python app.py
```

El servidor inicia en `http://localhost:5000`

## API Endpoints

- `GET /` - Interfaz web
- `POST /procesar_frame` - Procesar frame de imagen (base64)
- `GET /config/calibracion` - Obtener configuración de calibración
- `POST /config/calibracion` - Actualizar configuración de calibración
- `GET /config/contenedores` - Obtener configuración de contenedores
- `POST /config/contenedores` - Actualizar configuración de contenedores

## WebSocket Events

La conexión WebSocket permite comunicación bidireccional en tiempo real con el dashboard.

## Notas de Desarrollo

- El módulo `detector.py` maneja calibración con marcadores ArUco (DICT_4X4_50)
- Los rangos de color en HSV están predefinidos en `detector.py` (rojo, azul, verde, etc.)
- La interpolación suave de movimiento ocurre en el microcontrolador Arduino
- Thread-safety: Múltiples locks protegen el acceso a variables compartidas

## Calibración de Servos

El proyecto incluye un **calibrador manual de servos** (`calibracion_servos_3dof.ino`) para ajustar los ángulos de posicionamiento del brazo robótico.

### Uso del Calibrador

1. Cargar el sketch `calibracion_servos_3dof/calibracion_servos_3dof.ino` en el Arduino
2. Abrir el Monitor Serial a **115200 baud**
3. Enviar comandos en formato: `<servo> <grados>`

**Comandos disponibles:**
- `0 <ángulo>` - Servo base (rotación horizontal)
- `1 <ángulo>` - Servo hombro (articulación superior)
- `2 <ángulo>` - Servo codo (articulación inferior)
- `3 <ángulo>` - Servo pinza (apertura/cierre)

**Rango:** 0-180°

**Ejemplo:** `1 45` posiciona el servo del hombro a 45°

El calibrador mostrará continuamente las posiciones actuales de todos los servos.
