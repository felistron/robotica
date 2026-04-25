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


def procesar_frame(frame):
    """
    Detecta figuras geométricas y colores en un frame.
    Devuelve el frame anotado con etiquetas.
    """
    resultado = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Suavizado y detección de bordes
    blur = cv2.GaussianBlur(gris, (5, 5), 0)
    bordes = cv2.Canny(blur, 50, 150)

    # Dilatación para cerrar bordes abiertos
    kernel = np.ones((3, 3), np.uint8)
    bordes = cv2.dilate(bordes, kernel, iterations=1)

    # Encontrar contornos
    contornos, _ = cv2.findContours(bordes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area < 1500:  # Ignorar figuras muy pequeñas
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

        etiqueta = f"{figura} ({color})"
        (tw, th), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)

        # Fondo semitransparente para el texto
        overlay = resultado.copy()
        cv2.rectangle(overlay, (cx - tw//2 - 4, cy - th - 6),
                      (cx + tw//2 + 4, cy + 4), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, resultado, 0.5, 0, resultado)

        cv2.putText(resultado, etiqueta, (cx - tw//2, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    return resultado