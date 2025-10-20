import numpy as np 
import cv2
#import serial
import time
import matplotlib.pyplot as plt
from datetime import datetime
import subprocess
import os

# parámetros a ajustar
estamosHaciendoPruebas = True # para que no guarde fotos en drive al hacer pruebas
webcam = cv2.VideoCapture(0)
#arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Conexión al Arduino (ajusta si tu puerto es diferente)

# auxiliares
time.sleep(2)  # Esperar a que Arduino reinicie
prev_color = None # Variable temporal para guardar el frame anterior

# definir dónde guardar imágenes 
dirTimestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') # las minusculas se usan para la fecha y las mayusculas para horas!
dirTitle = f'exp {dirTimestamp}'
imgDir = os.path.join(os.getcwd(), #asumiendo que root dir es el repo.
                         'pruebas',
                         'vision',
                         'optimizaciones-a-code-p1',
                         'output',
                         f"{dirTitle}")
os.makedirs(imgDir, exist_ok=True)


# creación de límites de colores HSV (no se ocupan recalcular en loop principal)
# Red can wrap around the hue, so we use two ranges

red_lower1 = np.array([0, 120, 70], np.uint8)
red_upper1 = np.array([8, 255, 255], np.uint8) # ajuste fino
red_lower2 = np.array([170, 120, 70], np.uint8)
red_upper2 = np.array([180, 255, 255], np.uint8)
green_lower = np.array([36, 100, 50], np.uint8)
green_upper = np.array([86, 255, 255], np.uint8)
blue_lower = np.array([100, 150, 100], np.uint8) # prev: [94, 80, 2]
blue_upper = np.array([130, 255, 255], np.uint8)
white_lower = np.array([0, 0, 210], np.uint8) # prev: [0, 0, 210]
white_upper = np.array([180, 15, 255], np.uint8)

"""
processing stuff:

    beforeProcessing = time.time()
    afterProcessing = time.time()
    processingTime = afterProcessing - beforeProcessing
    print(f"Processing time before changing mask stuff: {processingTime}")
"""

# Operaciones morfológicas
MORPH_KERNEL = np.ones((5, 5), "uint8") 
RED_IT = 8; GREEN_IT = 13; BLUE_IT = 4; WHITE_IT = 5 # número de veces que se hace CIERRE por cada máscara

# Store contours and their metadata
max_area = 1000  # Minimum area threshold
max_contour = None; max_color = None; 
max_color_name = None; max_bounding_rect = None

numImgsGuardadas = 0; captured_image = None

while True: 
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _, imageFrame = webcam.read()
    
    # Convert to HSV color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    
    # crear máscaras binarias con el frame
    red_mask1 = cv2.inRange(hsvFrame, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2)
    red_mask = cv2.addWeighted(red_mask1, 1.0, red_mask2, 1.0, 0.0)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    white_mask = cv2.inRange(hsvFrame, white_lower, white_upper)
    
    # Morphological Transform (apertura y dilatación)
    # se hace erosión, luego dilatación, y luego dilatación. 
    
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=RED_IT)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=GREEN_IT)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=BLUE_IT)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=WHITE_IT)
    # Process contours for each color
    color_masks = [
        (red_mask, "rojo", (0, 0, 255)), # atras
        (green_mask, "verde", (0, 255, 0)), # derecha
        (blue_mask, "azul", (255, 0, 0)), # izquierda
        (white_mask, "blanco", (255, 255, 255)) #adelante
    ]
    
    comando = '5'  # Default command for no color detected
    for mask, color_name, color_bgr in color_masks: # O(1)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        for contour in contours: # O(n)
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
                max_color = color_bgr
                max_color_name = color_name
                max_bounding_rect = cv2.boundingRect(contour)
                if max_color_name == "rojo":
                    comando = '2'
                elif max_color_name == "verde":
                    comando = '3'
                elif max_color_name == "blanco":
                    comando = '1'
                elif max_color_name == "azul":
                    comando = '4'
                else:
                    comando = '5'


    # Draw only the largest contour, if found
    rectFrame = imageFrame.copy()
    if max_contour is not None:
        x, y, w, h = max_bounding_rect
        rectFrame = cv2.rectangle(rectFrame, (x, y), (x + w, y + h), max_color, 2)
        cv2.putText(rectFrame, max_color_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, max_color, 2)

    """arduino.write(comando.encode())  # Envía el comando
    print("Arduino:", max_color_name)"""


    """if prev_color is not max_color_name and max_color_name is not None and estamosHaciendoPruebas is False:

        print("Color diferente detectado!")

        # Aquí puedes hacer lo que quieras (ej: guardar imagen, mandar señal, etc.)
        filename = f"{max_color_name}_{timestamp}.jpg"
        cv2.imwrite(filename, imageFrame)

        subprocess.Popen(["rclone", "copy", filename, "vc:/imgs_colores/"])
        print("Subido a Drive:", filename)"""

    prev_color = max_color_name

    # Mostrar las máscaras finales en ventanas separadas
    cv2.imshow("Original", rectFrame)
    cv2.imshow("Mascara Rojo", red_mask)
    cv2.imshow("Mascara Verde", green_mask)
    cv2.imshow("Mascara Azul", blue_mask)
    cv2.imshow("Mascara Blanco", white_mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        if len(os.listdir(imgDir)) == 0:
            os.rmdir(imgDir)  # eliminar carpeta si está vacía
        webcam.release()
    elif key == ord('t'):
        
        # Generar canales histograma
        captured_image = imageFrame.copy()
        processedImage = cv2.cvtColor(captured_image, cv2.COLOR_BGR2HSV_FULL)
        h, s, v = cv2.split(processedImage)
        hHist = cv2.calcHist([h], [0], None, [180], [0, 180])
        sHist = cv2.calcHist([s], [0], None, [256], [0, 256])
        lHist = cv2.calcHist([v], [0], None, [256], [0, 256])
        
        # Mostrar histograma con matplotlib
        plt.figure('Histograma Matplotlib', figsize=(12, 6))
        plt.plot(hHist, color='black', label='H')
        plt.plot(sHist, color='green', label='S')
        plt.plot(lHist, color='yellow', label='V')
        plt.legend()
        plt.title('Histograma HSV')
        plt.xlabel('Valor de pixel')
        plt.ylabel('Frecuencia')
        plt.xticks(np.arange(0, 256, 10))
        plt.grid()
        
        # guardar imagenes de frame actual e histograma en carpeta correspondiente
        numImgsGuardadas += 1
        cv2.imwrite(os.path.join(imgDir, f'img-{numImgsGuardadas}.jpg'), captured_image)
        plt.savefig(os.path.join(imgDir, f'hist-hsv-{numImgsGuardadas}.png'))


        # se ocupa hacer todo ANTES del plt.show()
        plt.show()
