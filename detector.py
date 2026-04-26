import cv2
import numpy as np

# Rangos de color en espacio HSV
COLORES_HSV = {
    "rojo":     [(0, 100, 100), (10, 255, 255)],
    "rojo2":    [(160, 100, 100), (180, 255, 255)],
    "naranja":  [(11, 100, 100), (25, 255, 255)],
    "amarillo": [(26, 100, 100), (35, 255, 255)],
    "verde":    [(36, 50, 50),  (85, 255, 255)],
    "azul":     [(86, 50, 50),  (130, 255, 255)],
    "morado":   [(131, 50, 50), (160, 255, 255)],
    "blanco":   [(0, 0, 200),   (180, 30, 255)],
    "negro":    [(0, 0, 0),     (180, 255, 50)],
}

# Configuración del plano físico y marcadores ArUco para calibración.
PLANO_ANCHO_CM = 40.0
PLANO_ALTO_CM = 30.0
MARCADORES_PLANO = {
    0: "top_left",
    1: "top_right",
    2: "bottom_right",
    3: "bottom_left",
}

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def detectar_color(region_hsv):
    """Detecta el color dominante en una región HSV."""
    mejor_color = "desconocido"
    max_pixeles = 0

    for nombre, (bajo, alto) in COLORES_HSV.items():
        mascara = cv2.inRange(region_hsv, np.array(bajo), np.array(alto))
        pixeles = cv2.countNonZero(mascara)
        if pixeles > max_pixeles:
            max_pixeles = pixeles
            mejor_color = nombre if nombre != "rojo2" else "rojo"

    return mejor_color if max_pixeles > 500 else "desconocido"


def clasificar_figura(vertices):
    """Clasifica la figura según el número de vértices del polígono aproximado."""
    n = len(vertices)
    if n == 3:
        return "triángulo"
    elif n == 4:
        # Distinguir cuadrado de rectángulo por relación de aspecto
        x, y, w, h = cv2.boundingRect(vertices)
        ratio = w / float(h)
        return "cuadrado" if 0.9 <= ratio <= 1.1 else "rectángulo"
    elif n == 5:
        return "pentágono"
    elif n == 6:
        return "hexágono"
    elif n >= 8:
        return "círculo"
    else:
        return f"polígono ({n} lados)"


def _detectar_calibracion_aruco(frame, resultado):
    """Calcula homografía de píxeles a centímetros usando 4 marcadores ArUco."""
    corners, ids, _ = ARUCO_DETECTOR.detectMarkers(frame)

    calibracion = {
        "activa": False,
        "marcadores_detectados": [],
        "marcadores_requeridos": sorted(MARCADORES_PLANO.keys()),
        "plano_cm": {
            "ancho": PLANO_ANCHO_CM,
            "alto": PLANO_ALTO_CM,
        },
        "motivo": "No se detectaron marcadores",
    }

    mascara_marcadores = np.zeros(frame.shape[:2], dtype=np.uint8)

    if ids is None or len(ids) == 0:
        return calibracion, None, mascara_marcadores

    cv2.aruco.drawDetectedMarkers(resultado, corners, ids)

    ids_lista = [int(valor) for valor in ids.flatten()]
    calibracion["marcadores_detectados"] = sorted(ids_lista)

    centros = {}
    for marker_id, marker_corners in zip(ids.flatten(), corners):
        marker_id = int(marker_id)
        puntos = marker_corners[0]
        cv2.fillConvexPoly(mascara_marcadores, np.int32(puntos), 255)
        centro_x = float(np.mean(puntos[:, 0]))
        centro_y = float(np.mean(puntos[:, 1]))
        centros[marker_id] = (centro_x, centro_y)

    faltantes = [marker_id for marker_id in MARCADORES_PLANO if marker_id not in centros]
    if faltantes:
        calibracion["motivo"] = f"Faltan marcadores requeridos: {faltantes}"
        return calibracion, None, mascara_marcadores

    puntos_imagen = np.float32([
        centros[0],
        centros[1],
        centros[2],
        centros[3],
    ])
    puntos_plano_cm = np.float32([
        [0.0, 0.0],
        [PLANO_ANCHO_CM, 0.0],
        [PLANO_ANCHO_CM, PLANO_ALTO_CM],
        [0.0, PLANO_ALTO_CM],
    ])

    homografia = cv2.getPerspectiveTransform(puntos_imagen, puntos_plano_cm)
    if homografia is None or not np.isfinite(homografia).all():
        calibracion["motivo"] = "No se pudo calcular la homografía"
        return calibracion, None, mascara_marcadores

    poligono = np.array(puntos_imagen, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(resultado, [poligono], True, (255, 120, 0), 2)

    calibracion["activa"] = True
    calibracion["motivo"] = "Calibración ArUco activa"
    calibracion["esquinas_px"] = {
        "top_left": {"x": float(puntos_imagen[0][0]), "y": float(puntos_imagen[0][1])},
        "top_right": {"x": float(puntos_imagen[1][0]), "y": float(puntos_imagen[1][1])},
        "bottom_right": {"x": float(puntos_imagen[2][0]), "y": float(puntos_imagen[2][1])},
        "bottom_left": {"x": float(puntos_imagen[3][0]), "y": float(puntos_imagen[3][1])},
    }

    return calibracion, homografia, mascara_marcadores


def _pixel_a_cm(homografia, x_px, y_px):
    punto_px = np.array([[[float(x_px), float(y_px)]]], dtype=np.float32)
    punto_cm = cv2.perspectiveTransform(punto_px, homografia)[0][0]
    return float(punto_cm[0]), float(punto_cm[1])


def procesar_frame(frame, return_metadata=False):
    """
    Detecta figuras geométricas y colores en un frame.
    Devuelve el frame anotado con etiquetas.
    """
    resultado = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    calibracion, homografia, mascara_marcadores = _detectar_calibracion_aruco(frame, resultado)

    # Suavizado y detección de bordes
    blur = cv2.GaussianBlur(gris, (5, 5), 0)
    bordes = cv2.Canny(blur, 50, 150)

    # Dilatación para cerrar bordes abiertos
    kernel = np.ones((3, 3), np.uint8)
    bordes = cv2.dilate(bordes, kernel, iterations=1)

    # Encontrar contornos
    contornos, _ = cv2.findContours(bordes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detecciones = []

    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area < 1500:  # Ignorar figuras muy pequeñas
            continue

        if cv2.countNonZero(mascara_marcadores) > 0:
            mascara_contorno = np.zeros(gris.shape, dtype=np.uint8)
            cv2.drawContours(mascara_contorno, [contorno], -1, 255, -1)
            superposicion = cv2.countNonZero(cv2.bitwise_and(mascara_contorno, mascara_marcadores))
            if superposicion / max(area, 1.0) > 0.35:
                continue

        # Aproximar polígono
        perimetro = cv2.arcLength(contorno, True)
        epsilon = 0.03 * perimetro
        aprox = cv2.approxPolyDP(contorno, epsilon, True)

        # Clasificar figura
        figura = clasificar_figura(aprox)

        # Detectar color en la región de interés
        x, y, w, h = cv2.boundingRect(aprox)
        roi_hsv = hsv[y:y+h, x:x+w]
        color = detectar_color(roi_hsv)

        # Dibujar contorno
        cv2.drawContours(resultado, [aprox], -1, (0, 255, 0), 2)

        # Etiqueta centrada
        M = cv2.moments(contorno)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2

        centroide_cm = None
        en_plano = False
        if homografia is not None:
            x_cm, y_cm = _pixel_a_cm(homografia, cx, cy)
            en_plano = 0.0 <= x_cm <= PLANO_ANCHO_CM and 0.0 <= y_cm <= PLANO_ALTO_CM
            centroide_cm = {"x": round(x_cm, 2), "y": round(y_cm, 2)}

        detecciones.append({
            "figura": figura,
            "color": color,
            "area_px": float(area),
            "centroide_px": {"x": cx, "y": cy},
            "centroide_cm": centroide_cm,
            "en_plano": en_plano,
            "bbox_px": {"x": x, "y": y, "w": w, "h": h},
        })

        etiqueta = f"{figura} ({color})"
        if centroide_cm is not None:
            etiqueta += f" {centroide_cm['x']:.1f}cm,{centroide_cm['y']:.1f}cm"
        (tw, th), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)

        # Fondo semitransparente para el texto
        overlay = resultado.copy()
        cv2.rectangle(overlay, (cx - tw//2 - 4, cy - th - 6),
                      (cx + tw//2 + 4, cy + 4), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, resultado, 0.5, 0, resultado)

        cv2.putText(resultado, etiqueta, (cx - tw//2, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    if return_metadata:
        return resultado, detecciones, calibracion

    return resultado