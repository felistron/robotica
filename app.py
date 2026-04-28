import base64

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit
import time

from serial_handler import SerialHandler

from detector import (
    actualizar_configuracion_calibracion,
    obtener_configuracion_calibracion,
    procesar_frame,
)


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Serial and send-once tracking
serialer = SerialHandler()
_TRACK_LAST_SEEN = {}
_SENT_TRACKS = set()
_LAST_PRUNE = time.time()
_PRUNE_TIMEOUT = 2.0  # seconds without seeing track -> allow resend


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
                if en_plano and color in ('rojo', 'azul') and track not in _SENT_TRACKS:
                    centro = d.get('centroide_cm') or {}
                    x = centro.get('x')
                    y = centro.get('y')
                    try:
                        serialer.send_detection(track, d.get('figura', ''), color, x, y)
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

