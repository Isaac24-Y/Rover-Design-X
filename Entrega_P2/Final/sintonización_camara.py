import cv2
import numpy as np

# ================================================================
#               CLASIFICACIÓN DE FIGURA POR HU MOMENTS
# ================================================================
RANGOS = {
    "X": {
        1: (0.68,0.75),
        2: (2.7, 5.1),
        3: (0,6.5),
        4: (5,7.5),
        5: (-14, 15),
        6: (-12,11),
        7: (-13.5,13)
    },
    "T": {
        1: (0.3,0.7),
        2: (0.5,4),
        3: (1, 2),
        4: (1,4),
        5: (-10,7),
        6: (-5,6),
        7: (-8,7 )
    },
    "Circulo": {
        1: (0.72, 0.81),
        2: (3.35, 5.1),
        3: (4.5, 6.5),
        4: (5.2,8.4),
        5: (-17,18),
        6: (-14,13),
        7: (-17,17.5)
    },
    "Triangulo": {
        1: (0.6,1),
        2: (2, 4),
        3: (2,4),
        4: (3.5,6),
        5: (6.5,10),
        6: (4.8,10),
        7: (-10,10)
    },
    "Cuadrado": {
        1: (0.70, 0.89),
        2: (2, 6.5),
        3: (3, 7),
        4: (4, 7.9),
        5: (-15, 15),
        6: (-11, 11),
        7: (-15.7, 14)
    }
}

def coincide_rango(valor, rango):
    return rango[0] <= valor <= rango[1]


def clasificar_por_hu_preciso(hu_log):
    for figura, reglas in RANGOS.items():
        ok = True
        for idx, rango in reglas.items():
            if not coincide_rango(hu_log[idx-1], rango):
                ok = False
                break
        if ok:
            return figura
    return "Desconocido"


# ================================================================
#           DETECTOR NARANJA + VERDE + AMARILLO FOSFO
# ================================================================
cap = cv2.VideoCapture(0)

# ---------- NARANJA FOSFO ----------
lower_orange = np.array([8, 150, 180])
upper_orange = np.array([18, 255, 255])

# ---------- VERDE FOSFO ----------
lower_green = np.array([35, 120, 150])
upper_green = np.array([85, 255, 255])

# ---------- AMARILLO FOSFO ----------
lower_yellow = np.array([22, 140, 150])
upper_yellow = np.array([32, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Máscaras por color
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_green  = cv2.inRange(hsv, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Combinar colores
    mask = cv2.bitwise_or(mask_orange, mask_green)
    mask = cv2.bitwise_or(mask, mask_yellow)

    # Suavizar máscara
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if cnts:
        c = max(cnts, key=cv2.contourArea)

        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x,y),(x+w, y+h),(0,255,0), 2)

        m = cv2.moments(c)
        if m["m00"] > 0:
            hu = cv2.HuMoments(m).flatten()
            hu_log = -np.sign(hu)*np.log10(np.abs(hu)+1e-30)

            figura = clasificar_por_hu_preciso(hu_log)

            cv2.putText(frame, figura, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            print("\nFigura:", figura)
            for i in range(7):
                print(f"hu_log[{i+1}] = {hu_log[i]:.6f}")

    cv2.imshow("Mask", mask)
    cv2.imshow("Detección", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
