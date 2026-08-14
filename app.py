import base64
import json
import time
import threading
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit

from serial_handler import SerialHandler

from detector import (
    actualizar_configuracion_calibracion,
    obtener_configuracion_calibracion,
    procesar_frame,
)

import logging

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Serial and send-once tracking
serialer = SerialHandler()
CONTAINER_CONFIG_FILE = Path(__file__).with_name('container_config.json')
CONTAINER_CONFIG_LOCK = threading.Lock()
PENDING_CONTAINER_HANDSHAKE = None
PENDING_CONTAINER_HANDSHAKE_LOCK = threading.Lock()

# Gripper height configuration (cm) - configurable height for pick & place
DEFAULT_GRIPPER_HEIGHT_CM = 0.0

_TRACK_LAST_SEEN = {}
_SENT_TRACKS = set()
_LAST_PRUNE = time.time()
_PRUNE_TIMEOUT = 2.0  # seconds without seeing track -> allow resend


def _configuracion_contenedores_por_defecto():
    return {
        'contenedores': {
            'rojo': {'indice': 0, 'x_cm': 12.0, 'y_cm': 8.0, 'z_cm': 0.0},
            'azul': {'indice': 1, 'x_cm': 28.0, 'y_cm': 8.0, 'z_cm': 0.0},
        }
    }


def obtener_configuracion_contenedores():
    with CONTAINER_CONFIG_LOCK:
        return json.loads(json.dumps(_contenedores_actuales))


def _guardar_configuracion_contenedores():
    CONTAINER_CONFIG_FILE.write_text(
        json.dumps(obtener_configuracion_contenedores(), ensure_ascii=False, indent=2),
        encoding='utf-8',
    )


def cargar_configuracion_contenedores():
    if not CONTAINER_CONFIG_FILE.exists():
        return obtener_configuracion_contenedores()

    try:
        configuracion = json.loads(CONTAINER_CONFIG_FILE.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return obtener_configuracion_contenedores()

    return actualizar_configuracion_contenedores(configuracion, persistir=False)


def actualizar_configuracion_contenedores(configuracion, persistir=True):
    global _contenedores_actuales

    configuracion = configuracion or {}
    contenedores = configuracion.get('contenedores') or {}

    if 'rojo' not in contenedores or 'azul' not in contenedores:
        raise ValueError('Debes configurar los contenedores rojo y azul')

    nuevos = {}
    for color, indice_default in (('rojo', 0), ('azul', 1)):
        datos = contenedores.get(color) or {}
        try:
            indice = int(datos.get('indice', indice_default))
            x_cm = float(datos.get('x_cm'))
            y_cm = float(datos.get('y_cm'))
            z_cm = float(datos.get('z_cm', 0.0))
        except (TypeError, ValueError):
            raise ValueError(f'Contenedor inválido para {color}')

        if indice < 0:
            raise ValueError('El índice del contenedor debe ser mayor o igual a 0')

        nuevos[color] = {
            'indice': indice,
            'x_cm': round(x_cm, 2),
            'y_cm': round(y_cm, 2),
            'z_cm': round(z_cm, 2),
        }

    with CONTAINER_CONFIG_LOCK:
        _contenedores_actuales = {'contenedores': nuevos}

    if persistir:
        _guardar_configuracion_contenedores()

    return obtener_configuracion_contenedores()


def _lineas_handshake_contenedores(configuracion=None):
    config = configuracion or obtener_configuracion_contenedores()
    contenedores = config.get('contenedores') or {}
    lineas = ['CFG,BEGIN']
    for color in ('rojo', 'azul'):
        datos = contenedores.get(color)
        if not datos:
            continue
        lineas.append(
            f"CFG,CONT,{color},{int(datos['indice'])},{float(datos['x_cm']):.2f},{float(datos['y_cm']):.2f},{float(datos.get('z_cm', 0.0)):.2f}"
        )
    lineas.append('CFG,END')
    return lineas


def _set_pending_container_handshake(configuracion):
    global PENDING_CONTAINER_HANDSHAKE
    with PENDING_CONTAINER_HANDSHAKE_LOCK:
        PENDING_CONTAINER_HANDSHAKE = json.loads(json.dumps(configuracion))


def _get_pending_container_handshake():
    with PENDING_CONTAINER_HANDSHAKE_LOCK:
        if PENDING_CONTAINER_HANDSHAKE is None:
            return None
        return json.loads(json.dumps(PENDING_CONTAINER_HANDSHAKE))


def _enviar_handshake_contenedores_si_listo():
    if not serialer.get_status().get('connected'):
        return False

    configuracion = _get_pending_container_handshake() or obtener_configuracion_contenedores()
    serialer.clear_output_queue()
    serialer.enqueue_lines(_lineas_handshake_contenedores(configuracion))
    serialer.set_ready(True)
    return True


def _manejar_linea_serial(texto):
    if (texto or '').strip().upper() == 'READY':
        _enviar_handshake_contenedores_si_listo()


def _indice_contenedor_para_color(color):
    config = obtener_configuracion_contenedores()
    datos = (config.get('contenedores') or {}).get((color or '').lower())
    if not datos:
        return None
    return int(datos['indice'])


_contenedores_actuales = _configuracion_contenedores_por_defecto()
try:
    cargar_configuracion_contenedores()
except Exception:
    pass
_set_pending_container_handshake(obtener_configuracion_contenedores())
serialer.set_line_callback(_manejar_linea_serial)


def _decodificar_imagen(imagen_codificada):
    if not imagen_codificada:
        raise ValueError("No se recibió imagen")

    if "," in imagen_codificada:
        imagen_codificada = imagen_codificada.split(",", 1)[1]

    img_bytes = base64.b64decode(imagen_codificada)
    np_arr = np.frombuffer(img_bytes, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if frame is None:
        raise ValueError("No se pudo decodificar el frame")

    return frame


def _procesar_imagen_codificada(imagen_codificada):
    frame = _decodificar_imagen(imagen_codificada)
    resultado, detecciones, calibracion = procesar_frame(frame, return_metadata=True)

    exito, buffer = cv2.imencode(
        ".jpg",
        resultado,
        [cv2.IMWRITE_JPEG_QUALITY, 80],
    )
    if not exito:
        raise ValueError("No se pudo codificar el resultado")

    img_b64 = base64.b64encode(buffer).decode("utf-8")
    return {
        "imagen": f"data:image/jpeg;base64,{img_b64}",
        "detecciones": detecciones,
        "calibracion": calibracion,
    }


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/procesar_frame', methods=['POST'])
def procesar_frame_endpoint():
    payload = request.get_json(silent=True) or {}
    resultado = _procesar_imagen_codificada(payload.get('imagen'))
    return jsonify(resultado)


@app.route('/config/calibracion', methods=['GET'])
def obtener_config_calibracion_endpoint():
    return jsonify(obtener_configuracion_calibracion())


@app.route('/config/calibracion', methods=['POST'])
def actualizar_config_calibracion_endpoint():
    payload = request.get_json(silent=True) or {}
    try:
        config = actualizar_configuracion_calibracion(payload)
    except ValueError as error:
        return jsonify({"error": str(error)}), 400
    return jsonify(config)


@app.route('/config/contenedores', methods=['GET'])
def obtener_config_contenedores_endpoint():
    return jsonify(obtener_configuracion_contenedores())


@app.route('/config/contenedores', methods=['POST'])
def actualizar_config_contenedores_endpoint():
    payload = request.get_json(silent=True) or {}
    try:
        config = actualizar_configuracion_contenedores(payload)
        _set_pending_container_handshake(config)
        serialer.set_ready(False)
    except ValueError as error:
        return jsonify({'error': str(error)}), 400
    return jsonify(config)


@socketio.on('frame')
def manejar_frame(datos):
    try:
        resultado = _procesar_imagen_codificada(datos.get('imagen'))
        emit('resultado', resultado)
        # Non-blocking: enqueue detections for serial sending following rules:
        try:
            ahora = time.time()
            detecciones = resultado.get('detecciones', []) or []
            for d in detecciones:
                track = d.get('track_id')
                if track is None:
                    continue
                _TRACK_LAST_SEEN[track] = ahora
                color = (d.get('color') or '').lower()
                en_plano = bool(d.get('en_plano', False))
                # send only once per stable detection, only rojo/azul and en_plano
                indice_contenedor = _indice_contenedor_para_color(color) if en_plano else None
                if indice_contenedor is not None and track not in _SENT_TRACKS:
                    centro = d.get('centroide_cm') or {}
                    x = centro.get('x') - 3.0  # ajuste de calibración para alinear con centro del objeto
                    y = centro.get('y')
                    try:
                        if serialer.send_detection(indice_contenedor, d.get('figura', ''), color, x, y, DEFAULT_GRIPPER_HEIGHT_CM):
                            _SENT_TRACKS.add(track)
                    except Exception:
                        pass

            # prune old tracks to allow future re-sends
            global _LAST_PRUNE
            if ahora - _LAST_PRUNE > 1.0:
                expirados = [t for t, ts in _TRACK_LAST_SEEN.items() if ahora - ts > _PRUNE_TIMEOUT]
                for t in expirados:
                    _TRACK_LAST_SEEN.pop(t, None)
                    _SENT_TRACKS.discard(t)
                _LAST_PRUNE = ahora
        except Exception:
            # keep frame processing robust
            pass
    except (ValueError, KeyError) as error:
        emit('error', {'mensaje': str(error)})


@app.route('/serial/ports', methods=['GET'])
def serial_ports():
    try:
        ports = serialer.list_ports()
        return jsonify({'ports': ports})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/serial/connect', methods=['POST'])
def serial_connect():
    payload = request.get_json(silent=True) or {}
    port = payload.get('port')
    baud = int(payload.get('baud', 115200))
    if not port:
        return jsonify({'error': 'Se requiere puerto'}), 400
    try:
        serialer.connect(port, baud)
        serialer.set_ready(False)
        return jsonify(serialer.get_status())
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/serial/disconnect', methods=['POST'])
def serial_disconnect():
    try:
        serialer.disconnect()
        return jsonify(serialer.get_status())
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/serial/status', methods=['GET'])
def serial_status():
    try:
        return jsonify(serialer.get_status())
    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

