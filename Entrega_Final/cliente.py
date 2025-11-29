# Cliente Rover - Interfaz Completa con Detección de Marcadores
import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk
import time
import os
import csv
from datetime import datetime

# ========== CONFIGURACIÓN ==========
SERVER_IP = "192.168.1.10"  # IP de la Raspberry Pi
UDP_PORT = 50000
TCP_PORT_VIDEO1 = 50001  # Cámara frontal
TCP_PORT_VIDEO2 = 50002  # Cámara superior
BUFFER_SIZE = 4096

# ========== VARIABLES GLOBALES ==========
running = True
pause_video = False
modo_autonomo = False
marcador_clave = "Circulo"  # Marcador objetivo (configurable)

# Frames de video
frame_frontal = None
frame_superior = None
frame_frontal_tk = None
frame_superior_tk = None

# Datos de sensores
datos_sensores = {
    "temperatura": "-- °C",
    "luz": "-- lux",
    "humedad_suelo": "-- %",
    "humedad_aire": "-- %",
    "distancia": "-- mm",
    "imu": "-- -- --"
}

# Contador de marcadores
marcadores_detectados = []
total_marcadores = 0

# CSV para guardar datos
CSV_FILE = "datos_exploracion.csv"
csv_lock = threading.Lock()

# ========== DETECCIÓN DE MARCADORES ==========
def detectar_marcador(frame):
    """
    Detecta marcadores en la imagen usando OpenCV.
    Retorna: tipo de marcador detectado o None
    """
    if frame is None:
        return None
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:  # Filtrar contornos pequeños
            continue
        
        # Aproximar forma
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)
        
        # Clasificar por número de vértices
        if vertices == 3:
            return "Triangulo"
        elif vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w) / h
            if 0.95 <= aspectRatio <= 1.05:
                return "Cuadrado"
            else:
                return "Cruz"
        elif vertices > 8:
            return "Circulo"
    
    return None

def calcular_pixelsUV(frame):
    """Retorna dimensiones de la imagen"""
    if frame is None:
        return 0, 0
    h, w = frame.shape[:2]
    return w, h  # U=ancho, V=alto

# ========== GESTIÓN CSV ==========
def inicializar_csv():
    """Crea el archivo CSV con encabezados si no existe"""
    if not os.path.exists(CSV_FILE):
        with csv_lock:
            with open(CSV_FILE, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Timestamp", "PIXELSU", "PIXELSV", "TIPO_MARCADOR",
                    "INTENSIDAD_LUZ", "TEMPERATURA", "HUMEDAD_SUELO", "MODO"
                ])

def guardar_dato_csv(pixelsU, pixelsV, marcador, luz, temp, humedad, modo="MANUAL"):
    """Guarda un registro en el CSV"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with csv_lock:
        with open(CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, pixelsU, pixelsV, marcador, luz, temp, humedad, modo])
    agregar_log(f"[CSV] Datos guardados: {marcador} @ {timestamp}")

# ========== COMUNICACIÓN UDP ==========
def enviar_comando(comando, espera_respuesta=True):
    """Envía comando por UDP al servidor"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3)
        sock.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        
        if espera_respuesta:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            respuesta = data.decode('utf-8', errors='ignore')
        else:
            respuesta = "Enviado"
        
        sock.close()
        agregar_log(f"[CMD] {comando} → {respuesta}")
        return respuesta
    
    except socket.timeout:
        agregar_log(f"[ERROR] Timeout: {comando}")
        return "TIMEOUT"
    except Exception as e:
        agregar_log(f"[ERROR] {comando}: {e}")
        return f"ERROR: {e}"

# ========== RECEPCIÓN DE VIDEO ==========
def recibir_video(puerto, callback):
    """Thread genérico para recibir video por TCP"""
    global running
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SERVER_IP, puerto))
        agregar_log(f"[VIDEO] Conectado al puerto {puerto}")
    except Exception as e:
        agregar_log(f"[ERROR] No se pudo conectar al video {puerto}: {e}")
        return
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    while running:
        try:
            # Leer tamaño del frame
            while len(data) < payload_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    running = False
                    break
                data += packet
            
            if not running:
                break
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_size)[0]
            
            # Leer frame completo
            while len(data) < msg_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    running = False
                    break
                data += packet
            
            if not running:
                break
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            # Decodificar frame
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is not None:
                callback(frame)
        
        except Exception as e:
            agregar_log(f"[ERROR VIDEO {puerto}] {e}")
            break
    
    sock.close()
    agregar_log(f"[VIDEO] Desconectado del puerto {puerto}")

def actualizar_frame_frontal(frame):
    """Callback para actualizar video frontal"""
    global frame_frontal, frame_frontal_tk
    frame_frontal = frame.copy()
    
    if not pause_video:
        # Detectar marcadores si modo autónomo está activo
        if modo_autonomo:
            marcador = detectar_marcador(frame)
            if marcador:
                cv2.putText(frame, f"MARCADOR: {marcador}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Actualizar GUI
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = img.resize((640, 480))
        frame_frontal_tk = ImageTk.PhotoImage(img)
        label_video_frontal.config(image=frame_frontal_tk)
        label_video_frontal.image = frame_frontal_tk

def actualizar_frame_superior(frame):
    """Callback para actualizar video superior"""
    global frame_superior, frame_superior_tk
    frame_superior = frame.copy()
    
    if not pause_video:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = img.resize((320, 240))
        frame_superior_tk = ImageTk.PhotoImage(img)
        label_video_superior.config(image=frame_superior_tk)
        label_video_superior.image = frame_superior_tk

# ========== ACTUALIZACIÓN DE SENSORES ==========
def actualizar_sensores():
    """Lee sensores cada segundo"""
    if running:
        threading.Thread(target=lambda: enviar_comando("sensores"), daemon=True).start()
        ventana.after(1000, actualizar_sensores)

def procesar_datos_sensores(respuesta):
    """Parsea respuesta de sensores y actualiza GUI"""
    # Formato esperado: "SENSORES:temp,hum_aire,luz,distancia"
    if respuesta.startswith("SENSORES:"):
        valores = respuesta.split(":")[1].split(",")
        if len(valores) >= 4:
            datos_sensores["temperatura"] = f"{valores[0]} °C"
            datos_sensores["humedad_aire"] = f"{valores[1]} %"
            datos_sensores["luz"] = f"{valores[2]} lux"
            datos_sensores["distancia"] = f"{valores[3]} mm"
            
            # Actualizar labels
            label_temp.config(text=datos_sensores["temperatura"])
            label_luz.config(text=datos_sensores["luz"])
            label_humedad.config(text=datos_sensores["humedad_aire"])
            label_distancia.config(text=datos_sensores["distancia"])

# ========== FUNCIONES DE CONTROL ==========
def mover_adelante():
    enviar_comando("adelante", False)

def mover_atras():
    enviar_comando("atras", False)

def girar_izquierda():
    enviar_comando("izquierda", False)

def girar_derecha():
    enviar_comando("derecha", False)

def detener():
    enviar_comando("stop", False)

def mover_camara(angulo):
    enviar_comando(f"camara:{angulo}", False)

def toggle_modo_autonomo():
    global modo_autonomo
    modo_autonomo = not modo_autonomo
    
    if modo_autonomo:
        enviar_comando("modo_auto")
        btn_modo.config(text="🤖 Modo: AUTÓNOMO", bg="#28a745")
        agregar_log("[SISTEMA] Modo autónomo activado")
    else:
        enviar_comando("modo_manual")
        btn_modo.config(text="🎮 Modo: MANUAL", bg="#ffc107")
        agregar_log("[SISTEMA] Modo manual activado")

def capturar_marcador():
    """Captura el frame actual y procesa el marcador"""
    global total_marcadores
    
    if frame_frontal is None:
        messagebox.showwarning("Captura", "No hay video disponible")
        return
    
    # Detectar marcador
    marcador = detectar_marcador(frame_frontal)
    if not marcador:
        messagebox.showwarning("Detección", "No se detectó ningún marcador en la imagen")
        return
    
    # Obtener dimensiones
    pixelsU, pixelsV = calcular_pixelsUV(frame_frontal)
    
    # Obtener datos de sensores actuales
    luz = datos_sensores.get("luz", "0")
    temp = datos_sensores.get("temperatura", "0")
    humedad = "0"  # Manual siempre 0
    
    # Extraer valores numéricos
    luz_val = ''.join(filter(lambda x: x.isdigit() or x == '.', luz))
    temp_val = ''.join(filter(lambda x: x.isdigit() or x == '.', temp))
    
    # Guardar en CSV
    guardar_dato_csv(pixelsU, pixelsV, marcador, luz_val, temp_val, humedad, "MANUAL")
    
    # Actualizar contador
    total_marcadores += 1
    marcadores_detectados.append(marcador)
    label_contador.config(text=f"Marcadores: {total_marcadores}")
    
    # Verificar si es el marcador clave
    if marcador.lower() == marcador_clave.lower():
        messagebox.showinfo("¡MARCADOR CLAVE!", 
                           f"Se detectó el marcador clave: {marcador}\n"
                           "Activando modo autónomo...")
        activar_secuencia_autonoma(pixelsU, pixelsV, marcador, luz_val, temp_val)
    else:
        messagebox.showinfo("Marcador Capturado", 
                           f"Tipo: {marcador}\nPixelsU: {pixelsU}\nPixelsV: {pixelsV}")

def activar_secuencia_autonoma(pixelsU, pixelsV, marcador, luz, temp):
    """Secuencia autónoma cuando se detecta el marcador clave"""
    agregar_log(f"[AUTÓNOMO] Iniciando secuencia para {marcador}")
    
    # Detener motores
    enviar_comando("stop")
    time.sleep(0.5)
    
    # Activar brazo y tomar muestra
    agregar_log("[AUTÓNOMO] Tomando muestra de humedad...")
    respuesta = enviar_comando("brazo_muestra")
    
    # Esperar respuesta con valor de humedad
    time.sleep(5)  # Tiempo estimado de la secuencia
    
    # Obtener humedad del suelo
    resp_hum = enviar_comando("humedad_suelo")
    humedad_val = ''.join(filter(lambda x: x.isdigit() or x == '.', resp_hum))
    
    # Guardar en CSV con humedad real
    guardar_dato_csv(pixelsU, pixelsV, marcador, luz, temp, humedad_val, "AUTONOMO")
    
    agregar_log(f"[AUTÓNOMO] Muestra completada. Humedad: {humedad_val}%")
    messagebox.showinfo("Secuencia Completada", 
                       f"Muestra de humedad tomada: {humedad_val}%\n"
                       "Datos guardados en CSV")

def agregar_log(mensaje):
    """Agrega mensaje al área de logs"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    texto_log.insert(tk.END, f"[{timestamp}] {mensaje}\n")
    texto_log.see(tk.END)

def cerrar_aplicacion():
    global running
    running = False
    enviar_comando("stop")
    ventana.destroy()

# ========== INTERFAZ GRÁFICA ==========
ventana = tk.Tk()
ventana.title("🤖 ROVER DE EXPLORACIÓN - Control Center")
ventana.geometry("1400x900")
ventana.configure(bg="#1e1e1e")

# ========== PANEL SUPERIOR - Videos ==========
frame_videos = tk.Frame(ventana, bg="#1e1e1e")
frame_videos.pack(side="top", fill="both", expand=True, padx=10, pady=10)

# Video frontal
frame_frontal_container = tk.Frame(frame_videos, bg="#2d2d2d", relief="ridge", bd=2)
frame_frontal_container.pack(side="left", padx=5)
tk.Label(frame_frontal_container, text="📹 CÁMARA FRONTAL", bg="#2d2d2d", 
         fg="white", font=("Arial", 12, "bold")).pack(pady=5)
label_video_frontal = tk.Label(frame_frontal_container, bg="black", width=640, height=480)
label_video_frontal.pack(padx=5, pady=5)

# Video superior
frame_superior_container = tk.Frame(frame_videos, bg="#2d2d2d", relief="ridge", bd=2)
frame_superior_container.pack(side="left", padx=5)
tk.Label(frame_superior_container, text="📹 CÁMARA SUPERIOR", bg="#2d2d2d", 
         fg="white", font=("Arial", 12, "bold")).pack(pady=5)
label_video_superior = tk.Label(frame_superior_container, bg="black", width=320, height=240)
label_video_superior.pack(padx=5, pady=5)

# Control cámara superior
tk.Label(frame_superior_container, text="Rotación", bg="#2d2d2d", fg="white").pack()
slider_camara = tk.Scale(frame_superior_container, from_=0, to=180, orient="horizontal",
                        command=lambda v: mover_camara(int(v)), bg="#3d3d3d", fg="white")
slider_camara.set(90)
slider_camara.pack(pady=5)

# ========== PANEL LATERAL - Controles ==========
frame_controles = tk.Frame(ventana, bg="#2d2d2d", width=350, relief="ridge", bd=2)
frame_controles.pack(side="right", fill="y", padx=10, pady=10)

# Título
tk.Label(frame_controles, text="🎮 PANEL DE CONTROL", bg="#2d2d2d", 
         fg="white", font=("Arial", 14, "bold")).pack(pady=10)

# Modo de operación
btn_modo = tk.Button(frame_controles, text="🎮 Modo: MANUAL", command=toggle_modo_autonomo,
                    bg="#ffc107", fg="black", font=("Arial", 12, "bold"), height=2)
btn_modo.pack(fill="x", padx=10, pady=5)

# Marcador clave
frame_marcador = tk.Frame(frame_controles, bg="#2d2d2d")
frame_marcador.pack(fill="x", padx=10, pady=5)
tk.Label(frame_marcador, text="Marcador Clave:", bg="#2d2d2d", fg="white").pack(side="left")
combo_marcador = ttk.Combobox(frame_marcador, values=["Cruz", "T", "Circulo", "Cuadrado", "Triangulo"],
                              state="readonly", width=12)
combo_marcador.set(marcador_clave)
combo_marcador.pack(side="left", padx=5)

def cambiar_marcador(event):
    global marcador_clave
    marcador_clave = combo_marcador.get()
    agregar_log(f"[CONFIG] Marcador clave: {marcador_clave}")

combo_marcador.bind("<<ComboboxSelected>>", cambiar_marcador)

# Contador de marcadores
label_contador = tk.Label(frame_controles, text="Marcadores: 0", bg="#2d2d2d",
                         fg="#00ff00", font=("Arial", 14, "bold"))
label_contador.pack(pady=10)

# Botón capturar
tk.Button(frame_controles, text="📸 CAPTURAR MARCADOR", command=capturar_marcador,
         bg="#17a2b8", fg="white", font=("Arial", 12, "bold"), height=2).pack(fill="x", padx=10, pady=5)

# Separador
ttk.Separator(frame_controles, orient='horizontal').pack(fill='x', pady=10)

# Controles de movimiento
tk.Label(frame_controles, text="MOVIMIENTO", bg="#2d2d2d", 
         fg="white", font=("Arial", 12, "bold")).pack(pady=5)

frame_mov = tk.Frame(frame_controles, bg="#2d2d2d")
frame_mov.pack(pady=5)

tk.Button(frame_mov, text="⬆", command=mover_adelante, width=5, height=2,
         bg="#007bff", fg="white", font=("Arial", 14, "bold")).grid(row=0, column=1, padx=2, pady=2)
tk.Button(frame_mov, text="⬅", command=girar_izquierda, width=5, height=2,
         bg="#007bff", fg="white", font=("Arial", 14, "bold")).grid(row=1, column=0, padx=2, pady=2)
tk.Button(frame_mov, text="⏹", command=detener, width=5, height=2,
         bg="#dc3545", fg="white", font=("Arial", 14, "bold")).grid(row=1, column=1, padx=2, pady=2)
tk.Button(frame_mov, text="➡", command=girar_derecha, width=5, height=2,
         bg="#007bff", fg="white", font=("Arial", 14, "bold")).grid(row=1, column=2, padx=2, pady=2)
tk.Button(frame_mov, text="⬇", command=mover_atras, width=5, height=2,
         bg="#007bff", fg="white", font=("Arial", 14, "bold")).grid(row=2, column=1, padx=2, pady=2)

# Separador
ttk.Separator(frame_controles, orient='horizontal').pack(fill='x', pady=10)

# Sensores
tk.Label(frame_controles, text="📊 SENSORES", bg="#2d2d2d", 
         fg="white", font=("Arial", 12, "bold")).pack(pady=5)

frame_sensores = tk.Frame(frame_controles, bg="#2d2d2d")
frame_sensores.pack(fill="x", padx=10)

tk.Label(frame_sensores, text="🌡 Temperatura:", bg="#2d2d2d", fg="white").grid(row=0, column=0, sticky="w", pady=3)
label_temp = tk.Label(frame_sensores, text="-- °C", bg="#2d2d2d", fg="#ffc107", font=("Arial", 11, "bold"))
label_temp.grid(row=0, column=1, sticky="e")

tk.Label(frame_sensores, text="💡 Luz:", bg="#2d2d2d", fg="white").grid(row=1, column=0, sticky="w", pady=3)
label_luz = tk.Label(frame_sensores, text="-- lux", bg="#2d2d2d", fg="#ffc107", font=("Arial", 11, "bold"))
label_luz.grid(row=1, column=1, sticky="e")

tk.Label(frame_sensores, text="💧 Humedad Aire:", bg="#2d2d2d", fg="white").grid(row=2, column=0, sticky="w", pady=3)
label_humedad = tk.Label(frame_sensores, text="-- %", bg="#2d2d2d", fg="#ffc107", font=("Arial", 11, "bold"))
label_humedad.grid(row=2, column=1, sticky="e")

tk.Label(frame_sensores, text="📏 Distancia:", bg="#2d2d2d", fg="white").grid(row=3, column=0, sticky="w", pady=3)
label_distancia = tk.Label(frame_sensores, text="-- mm", bg="#2d2d2d", fg="#ffc107", font=("Arial", 11, "bold"))
label_distancia.grid(row=3, column=1, sticky="e")

# Botón salir
tk.Button(frame_controles, text="❌ SALIR", command=cerrar_aplicacion,
         bg="#6c757d", fg="white", font=("Arial", 12, "bold")).pack(side="bottom", fill="x", padx=10, pady=10)

# ========== PANEL INFERIOR - Logs ==========
frame_logs = tk.Frame(ventana, bg="#2d2d2d", height=150, relief="ridge", bd=2)
frame_logs.pack(side="bottom", fill="both", padx=10, pady=10)

tk.Label(frame_logs, text="📋 REGISTRO DE EVENTOS", bg="#2d2d2d", 
         fg="white", font=("Arial", 12, "bold")).pack(pady=5)

texto_log = scrolledtext.ScrolledText(frame_logs, height=8, bg="#1e1e1e", fg="#00ff00",
                                      font=("Courier", 9), wrap=tk.WORD)
texto_log.pack(fill="both", expand=True, padx=5, pady=5)

# ========== INICIALIZACIÓN ==========
inicializar_csv()
agregar_log("[SISTEMA] Cliente iniciado")
agregar_log(f"[CONFIG] Servidor: {SERVER_IP}")
agregar_log(f"[CONFIG] Marcador clave: {marcador_clave}")

# Iniciar threads de video
threading.Thread(target=lambda: recibir_video(TCP_PORT_VIDEO1, actualizar_frame_frontal), daemon=True).start()
threading.Thread(target=lambda: recibir_video(TCP_PORT_VIDEO2, actualizar_frame_superior), daemon=True).start()

# Iniciar actualización de sensores
ventana.after(1000, actualizar_sensores)

# Configurar cierre
ventana.protocol("WM_DELETE_WINDOW", cerrar_aplicacion)

# Iniciar GUI
ventana.mainloop()