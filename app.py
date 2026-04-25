import base64

from flask import Flask, render_template, jsonify, request
import cv2
import threading
from detector import procesar_frame

import numpy as np

app = Flask(__name__)

# Estado global de la cámara
camara = None
lock = threading.Lock()


def obtener_camara():
    global camara
    if camara is None or not camara.isOpened():
        camara = cv2.VideoCapture(0)
        camara.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return camara


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/procesar_frame', methods=['POST'])
def procesar_frame_endpoint():
    data = request.json['imagen']  # base64
    img_bytes = base64.b64decode(data.split(',')[1])
    np_arr = np.frombuffer(img_bytes, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    resultado = procesar_frame(frame)

    _, buffer = cv2.imencode('.jpg', resultado, [cv2.IMWRITE_JPEG_QUALITY, 80])
    img_b64 = base64.b64encode(buffer).decode('utf-8')
    return jsonify({"imagen": f"data:image/jpeg;base64,{img_b64}"})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

