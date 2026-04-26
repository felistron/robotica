import base64

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit

from detector import (
    actualizar_configuracion_calibracion,
    obtener_configuracion_calibracion,
    procesar_frame,
)


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


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
    except (ValueError, KeyError) as error:
        emit('error', {'mensaje': str(error)})


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

