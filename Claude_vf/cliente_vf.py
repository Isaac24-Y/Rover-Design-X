# cliente_con_odometria.py - Cliente con mapa de odometría
import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import time
import json
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math

# ==================== CONFIGURACIÓN ====================
SERVER_IP = "192.168.1.8"  # IP de Raspberry Pi
UDP_PORT = 50000
TCP_PORT_VIDEO_FRONTAL = 50001
TCP_PORT_VIDEO_SUPERIOR = 50002
TCP_PORT_STATUS = 50003  # ✅ CORREGIDO
BUFFER_SIZE = 4096

# ==================== DETECCIÓN DE MARCADORES ====================
RANGOS_HU = {
    "Cruz": {
        1: (0.68, 0.75), 2: (2.7, 5.1), 3: (0, 6.5), 4: (5, 7.5),
        5: (-14, 15), 6: (-12, 11), 7: (-13.5, 13)
    },
    "T": {
        1: (0.3, 0.7), 2: (0.5, 4), 3: (1, 2), 4: (1, 4),
        5: (-10, 7), 6: (-5, 6), 7: (-8, 7)
    },
    "Circulo": {
        1: (0.72, 0.81), 2: (3.35, 5.1), 3: (4.5, 6.5), 4: (5.2, 8.4),
        5: (-17, 18), 6: (-14, 13), 7: (-17, 17.5)
    },
    "Triangulo": {
        1: (0.6, 1), 2: (2, 4), 3: (2, 4), 4: (3.5, 6),
        5: (6.5, 10), 6: (4.8, 10), 7: (-10, 10)
    },
    "Cuadrado": {
        1: (0.70, 0.89), 2: (2, 6.5), 3: (3, 7), 4: (4, 7.9),
        5: (-15, 15), 6: (-11, 11), 7: (-15.7, 14)
    }
}

# Rangos HSV para colores fosforescentes
LOWER_ORANGE = np.array([8, 150, 180])
UPPER_ORANGE = np.array([18, 255, 255])
LOWER_GREEN = np.array([35, 120, 150])
UPPER_GREEN = np.array([85, 255, 255])
LOWER_YELLOW = np.array([22, 140, 150])
UPPER_YELLOW = np.array([32, 255, 255])

# ==================== VARIABLES GLOBALES ====================
running = True
frame_frontal = None
frame_superior = None
estado_rover = {}
estado_lock = threading.Lock()

# Odometría
class Odometria:
    def __init__(self):
        self.x = 0.0  # metros
        self.y = 0.0  # metros
        self.theta = 0.0  # radianes
        self.historial_x = deque(maxlen=200)
        self.historial_y = deque(maxlen=200)
        self.historial_theta = deque(maxlen=50)
        self.distancias = deque(maxlen=100)  # Historial de distancias
        self.ultimo_tiempo = time.time()
        self.velocidad = 0.0  # m/s estimada
        self.lock = threading.Lock()
        
    def actualizar_desde_mpu(self, gyro_z, dt):
        """Actualiza orientación desde giroscopio"""
        with self.lock:
            # Convertir de unidades MPU a radianes/s (aproximado)
            gyro_z_rad = gyro_z * (math.pi / 180.0) / 131.0  # Escala MPU6050
            self.theta += gyro_z_rad * dt
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi  # Normalizar
            self.historial_theta.append(self.theta)
    
    def actualizar_posicion(self, comando, dt):
        """Actualiza posición basándose en comandos"""
        with self.lock:
            if comando == "avanzar":
                velocidad_estimada = 0.3  # m/s aproximado
                dx = velocidad_estimada * dt * math.cos(self.theta)
                dy = velocidad_estimada * dt * math.sin(self.theta)
                self.x += dx
                self.y += dy
            elif comando == "retroceder":
                velocidad_estimada = 0.3
                dx = -velocidad_estimada * dt * math.cos(self.theta)
                dy = -velocidad_estimada * dt * math.sin(self.theta)
                self.x += dx
                self.y += dy
            
            self.historial_x.append(self.x)
            self.historial_y.append(self.y)
    
    def agregar_distancia(self, distancia):
        """Agrega medición de distancia"""
        if distancia > 0 and distancia < 2000:  # Entre 0 y 2 metros
            with self.lock:
                self.distancias.append((self.x, self.y, self.theta, distancia / 1000.0))
    
    def obtener_estado(self):
        """Retorna estado actual"""
        with self.lock:
            return {
                'x': self.x,
                'y': self.y,
                'theta': self.theta,
                'historial_x': list(self.historial_x),
                'historial_y': list(self.historial_y),
                'distancias': list(self.distancias)
            }
    
    def reset(self):
        """Reinicia odometría"""
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.historial_x.clear()
            self.historial_y.clear()
            self.historial_theta.clear()
            self.distancias.clear()

odometria = Odometria()
ultimo_comando = "stop"
marcador_clave = None

# ==================== FUNCIONES DE DETECCIÓN ====================
def coincide_rango(valor, rango):
    return rango[0] <= valor <= rango[1]

def clasificar_marcador(hu_log):
    """Clasifica figura usando Hu Moments"""
    for figura, reglas in RANGOS_HU.items():
        ok = True
        for idx, rango in reglas.items():
            if not coincide_rango(hu_log[idx-1], rango):
                ok = False
                break
        if ok:
            return figura
    return None

def detectar_marcador(frame):
    """Detecta marcador en frame y retorna tipo, coordenadas y frame anotado"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_orange = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
    mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    
    mask = cv2.bitwise_or(mask_orange, mask_green)
    mask = cv2.bitwise_or(mask, mask_yellow)
    
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not cnts:
        return None, None, None, frame
    
    c = max(cnts, key=cv2.contourArea)
    
    if cv2.contourArea(c) < 500:
        return None, None, None, frame
    
    x, y, w, h = cv2.boundingRect(c)
    
    m = cv2.moments(c)
    if m["m00"] > 0:
        hu = cv2.HuMoments(m).flatten()
        hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-30)
        
        tipo = clasificar_marcador(hu_log)
        
        if tipo:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, tipo, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            
            return tipo, cx, cy, frame
    
    return None, None, None, frame

# ==================== COMUNICACIÓN UDP ====================
def enviar_comando(comando, espera_respuesta=True):
    """Envía comando por UDP al servidor"""
    global ultimo_comando
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        
        # Actualizar comando actual para odometría
        if comando in ["avanzar", "retroceder", "izquierda", "derecha", "stop"]:
            ultimo_comando = comando
        
        if espera_respuesta:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            respuesta = data.decode('utf-8')
        else:
            respuesta = "Enviado"
        
        sock.close()
        return respuesta
    except Exception as e:
        return f"Error: {e}"

# ==================== RECEPCIÓN DE VIDEO ====================
def recibir_video_frontal():
    """Recibe video de cámara frontal"""
    global frame_frontal, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_VIDEO_FRONTAL))
            print("[CLIENTE] ✓ Conectado a video frontal")
            break
        except Exception as e:
            print(f"[ERROR] Video frontal (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                return
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    while running:
        try:
            while len(data) < payload_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_size)[0]
            
            while len(data) < msg_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is not None:
                frame_frontal = frame.copy()
            
        except Exception as e:
            print(f"[ERROR] Loop video frontal: {e}")
            break
    
    sock.close()

def recibir_video_superior():
    """Recibe video de cámara superior"""
    global frame_superior, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_VIDEO_SUPERIOR))
            print("[CLIENTE] ✓ Conectado a video superior")
            break
        except Exception as e:
            print(f"[ERROR] Video superior (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                return
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    while running:
        try:
            while len(data) < payload_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_size)[0]
            
            while len(data) < msg_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is not None:
                frame_superior = frame.copy()
            
        except Exception as e:
            print(f"[ERROR] Loop video superior: {e}")
            break
    
    sock.close()

# ==================== RECEPCIÓN DE ESTADO ====================
def recibir_estado():
    """Recibe estado del rover periódicamente"""
    global estado_rover, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_STATUS))
            print("[CLIENTE] ✓ Conectado a status")
            break
        except Exception as e:
            print(f"[ERROR] Status (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                return
    
    data = b""
    
    while running:
        try:
            while len(data) < 4:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            size = struct.unpack("I", data[:4])[0]
            data = data[4:]
            
            while len(data) < size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            json_data = data[:size]
            data = data[size:]
            
            estado = json.loads(json_data.decode('utf-8'))
            
            with estado_lock:
                estado_rover.update(estado)
            
        except Exception as e:
            print(f"[ERROR] Loop status: {e}")
            break
    
    sock.close()

# ==================== ACTUALIZACIÓN ODOMETRÍA ====================
def actualizar_odometria_thread():
    """Thread que actualiza odometría continuamente"""
    global ultimo_comando
    ultimo_tiempo = time.time()
    
    while running:
        try:
            tiempo_actual = time.time()
            dt = tiempo_actual - ultimo_tiempo
            ultimo_tiempo = tiempo_actual
            
            # Obtener datos MPU si están disponibles
            with estado_lock:
                if 'gyro' in estado_rover and 'z' in estado_rover['gyro']:
                    try:
                        gyro_z = float(estado_rover['gyro']['z'])
                        odometria.actualizar_desde_mpu(gyro_z, dt)
                    except:
                        pass
                
                # Agregar medición de distancia
                if 'distancia' in estado_rover:
                    try:
                        dist_str = estado_rover['distancia']
                        if 'mm' in dist_str:
                            dist = float(dist_str.replace('mm', '').strip())
                            odometria.agregar_distancia(dist)
                    except:
                        pass
            
            # Actualizar posición según comando actual
            if ultimo_comando in ["avanzar", "retroceder"]:
                odometria.actualizar_posicion(ultimo_comando, dt)
            
            time.sleep(0.05)  # 20 Hz
            
        except Exception as e:
            print(f"[ERROR] Odometría: {e}")
            time.sleep(0.1)

# ==================== INTERFAZ GRÁFICA ====================
class RoverClienteUI:
    def __init__(self, root):
        global marcador_clave
        
        self.root = root
        self.root.title("🤖 Control de Rover - Exploración con Odometría")
        self.root.geometry("1600x900")
        self.root.configure(bg="#1e1e1e")
        
        marcador_clave = tk.StringVar(value="Circulo")
        
        self.root.bind('<KeyPress>', self.tecla_presionada)
        self.root.bind('<KeyRelease>', self.tecla_liberada)
        
        self.teclas_activas = set()
        
        self.crear_interfaz()
        self.actualizar_gui()
        
    def crear_interfaz(self):
        # ===== FRAME PRINCIPAL =====
        main_container = tk.Frame(self.root, bg="#1e1e1e")
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ===== COLUMNA IZQUIERDA: VIDEOS =====
        left_frame = tk.Frame(main_container, bg="#1e1e1e")
        left_frame.pack(side="left", fill="both", expand=True)
        
        # Video Frontal
        frontal_label = tk.Label(left_frame, text="📹 CÁMARA FRONTAL", 
                                 bg="#2d2d2d", fg="white", font=("Arial", 12, "bold"))
        frontal_label.pack(pady=(0, 5))
        
        self.video_frontal = tk.Label(left_frame, bg="black", width=640, height=360)
        self.video_frontal.pack()
        
        # Video Superior
        superior_label = tk.Label(left_frame, text="📹 CÁMARA SUPERIOR", 
                                  bg="#2d2d2d", fg="white", font=("Arial", 12, "bold"))
        superior_label.pack(pady=(10, 5))
        
        self.video_superior = tk.Label(left_frame, bg="black", width=640, height=360)
        self.video_superior.pack()
        
        # ===== COLUMNA CENTRAL: MAPA =====
        center_frame = tk.Frame(main_container, bg="#2d2d2d", width=400)
        center_frame.pack(side="left", fill="both", expand=True, padx=(10, 0))
        
        map_label = tk.Label(center_frame, text="🗺️ MAPA DE ODOMETRÍA", 
                            bg="#2d2d2d", fg="#00ff00", font=("Arial", 14, "bold"))
        map_label.pack(pady=10)
        
        # Crear figura de matplotlib
        self.fig = Figure(figsize=(5, 5), facecolor='#1e1e1e')
        self.ax = self.fig.add_subplot(111, facecolor='#2d2d2d')
        self.ax.set_xlabel('X (metros)', color='white')
        self.ax.set_ylabel('Y (metros)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.grid(True, alpha=0.3)
        
        # Integrar en tkinter
        self.canvas_mapa = FigureCanvasTkAgg(self.fig, center_frame)
        self.canvas_mapa.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
        
        # Información de odometría
        odom_info_frame = tk.Frame(center_frame, bg="#3d3d3d")
        odom_info_frame.pack(fill="x", padx=10, pady=5)
        
        self.odom_label = tk.Label(odom_info_frame, 
                                   text="Posición: (0.0, 0.0) m | Ángulo: 0.0°", 
                                   bg="#3d3d3d", fg="#00ff00", 
                                   font=("Arial", 10, "bold"))
        self.odom_label.pack(pady=5)
        
        # Botón reset odometría
        tk.Button(center_frame, text="🔄 Reset Odometría", 
                 command=self.reset_odometria,
                 bg="#ff9800", fg="white", font=("Arial", 10, "bold")).pack(pady=5)
        
        # ===== COLUMNA DERECHA: CONTROLES =====
        right_frame = tk.Frame(main_container, bg="#2d2d2d", width=350)
        right_frame.pack(side="right", fill="y", padx=(10, 0))
        right_frame.pack_propagate(False)
        
        title = tk.Label(right_frame, text="🎮 PANEL DE CONTROL", 
                        bg="#2d2d2d", fg="#00ff00", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # ===== SENSORES =====
        sensores_frame = tk.LabelFrame(right_frame, text="📊 Sensores", 
                                       bg="#3d3d3d", fg="white", font=("Arial", 11, "bold"))
        sensores_frame.pack(fill="x", padx=10, pady=5)
        
        self.temp_label = tk.Label(sensores_frame, text="🌡️ Temp: -- °C", 
                                   bg="#3d3d3d", fg="#ff6b6b", font=("Arial", 10, "bold"))
        self.temp_label.pack(anchor="w", padx=10, pady=2)
        
        self.luz_label = tk.Label(sensores_frame, text="💡 Luz: -- V", 
                                  bg="#3d3d3d", fg="#ffd93d", font=("Arial", 10, "bold"))
        self.luz_label.pack(anchor="w", padx=10, pady=2)
        
        self.humedad_label = tk.Label(sensores_frame, text="💧 Humedad: -- %", 
                                      bg="#3d3d3d", fg="#6bcfff", font=("Arial", 10, "bold"))
        self.humedad_label.pack(anchor="w", padx=10, pady=2)
        
        self.dist_label = tk.Label(sensores_frame, text="📏 Distancia: -- mm", 
                                   bg="#3d3d3d", fg="#95e1d3", font=("Arial", 10, "bold"))
        self.dist_label.pack(anchor="w", padx=10, pady=2)
        
        # ===== ESTADO =====
        estado_frame = tk.LabelFrame(right_frame, text="🤖 Estado", 
                                     bg="#3d3d3d", fg="white", font=("Arial", 11, "bold"))
        estado_frame.pack(fill="x", padx=10, pady=5)
        
        self.modo_label = tk.Label(estado_frame, text="Modo: MANUAL", 
                                   bg="#3d3d3d", fg="#00ff00", font=("Arial", 10, "bold"))
        self.modo_label.pack(anchor="w", padx=10, pady=2)
        
        self.marcadores_label = tk.Label(estado_frame, text="🎯 Marcadores: 0", 
                                         bg="#3d3d3d", fg="#ff9ff3", font=("Arial", 10, "bold"))
        self.marcadores_label.pack(anchor="w", padx=10, pady=2)
        
        self.log_label = tk.Label(estado_frame, text="Esperando...", 
                                  bg="#3d3d3d", fg="#c7ecee", font=("Arial", 9), 
                                  wraplength=300, justify="left")
        self.log_label.pack(anchor="w", padx=10, pady=5)
        
        # ===== SERVOS =====
        servo_frame = tk.LabelFrame(right_frame, text="🔧 Servos", 
                                    bg="#3d3d3d", fg="white", font=("Arial", 11, "bold"))
        servo_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(servo_frame, text="Cámara Frontal (0-30°)", 
                bg="#3d3d3d", fg="white").pack(anchor="w", padx=10, pady=(5, 0))
        self.servo1_slider = tk.Scale(servo_frame, from_=0, to=30, orient="horizontal",
                                      bg="#3d3d3d", fg="white", troughcolor="#1e1e1e",
                                      command=self.servo1_changed, length=250)
        self.servo1_slider.set(15)
        self.servo1_slider.pack(padx=10, pady=2)
        
        tk.Label(servo_frame, text="Cámara Superior (0-180°)", 
                bg="#3d3d3d", fg="white").pack(anchor="w", padx=10, pady=(5, 0))
        self.servo4_slider = tk.Scale(servo_frame, from_=0, to=180, orient="horizontal",
                                      bg="#3d3d3d", fg="white", troughcolor="#1e1e1e",
                                      command=self.servo4_changed, length=250)
        self.servo4_slider.set(90)
        self.servo4_slider.pack(padx=10, pady=2)
        
        # ===== MOVIMIENTO =====
        mov_frame = tk.LabelFrame(right_frame, text="🕹️ Movimiento (WASD)", 
                                  bg="#3d3d3d", fg="white", font=("Arial", 11, "bold"))
        mov_frame.pack(fill="x", padx=10, pady=5)
        
        btn_frame = tk.Frame(mov_frame, bg="#3d3d3d")
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="▲", width=4, height=2, 
                 command=lambda: self.comando_movimiento("avanzar")).grid(row=0, column=1, padx=2, pady=2)
        
        tk.Button(btn_frame, text="◀", width=4, height=2,
                 command=lambda: self.comando_movimiento("izquierda")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(btn_frame, text="■", width=4, height=2, bg="#ff4444",
                 command=lambda: self.comando_movimiento("stop")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(btn_frame, text="▶", width=4, height=2,
                 command=lambda: self.comando_movimiento("derecha")).grid(row=1, column=2, padx=2, pady=2)
        
        tk.Button(btn_frame, text="▼", width=4, height=2,
                 command=lambda: self.comando_movimiento("retroceder")).grid(row=2, column=1, padx=2, pady=2)
        
        # ===== MARCADORES =====
        marcador_frame = tk.LabelFrame(right_frame, text="🎯 Detección", 
                                       bg="#3d3d3d", fg="white", font=("Arial", 11, "bold"))
        marcador_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(marcador_frame, text="Marcador Clave:", 
                bg="#3d3d3d", fg="white").pack(anchor="w", padx=10, pady=(5, 0))
        
        opciones = ["Cruz", "T", "Circulo", "Triangulo", "Cuadrado"]
        dropdown = ttk.Combobox(marcador_frame, textvariable=marcador_clave, 
                               values=opciones, state="readonly", width=15)
        dropdown.pack(padx=10, pady=5)
        dropdown.current(2)
        
        tk.Button(marcador_frame, text="📸 Capturar Marcador", 
                 command=self.capturar_marcador, bg="#4CAF50", fg="white",
                 font=("Arial", 10, "bold")).pack(padx=10, pady=10, fill="x")
        
        # ===== SALIR =====
        tk.Button(right_frame, text="✖ SALIR", command=self.cerrar,
                 bg="#d32f2f", fg="white", font=("Arial", 12, "bold"),
                 height=2).pack(side="bottom", fill="x", padx=10, pady=10)
    
    # ===== CALLBACKS =====
    def servo1_changed(self, val):
        angulo = int(float(val))
        threading.Thread(target=lambda: enviar_comando(f"servo1 {angulo}"), daemon=True).start()
    
    def servo4_changed(self, val):
        angulo = int(float(val))
        threading.Thread(target=lambda: enviar_comando(f"servo4 {angulo}"), daemon=True).start()
    
    def comando_movimiento(self, comando):
        threading.Thread(target=lambda: enviar_comando(comando), daemon=True).start()
    
    def tecla_presionada(self, event):
        tecla = event.keysym.lower()
        
        if tecla in self.teclas_activas:
            return
        
        self.teclas_activas.add(tecla)
        
        if tecla in ['w', 'up']:
            self.comando_movimiento("avanzar")
        elif tecla in ['s', 'down']:
            self.comando_movimiento("retroceder")
        elif tecla in ['a', 'left']:
            self.comando_movimiento("izquierda")
        elif tecla in ['d', 'right']:
            self.comando_movimiento("derecha")
    
    def tecla_liberada(self, event):
        tecla = event.keysym.lower()
        
        if tecla in self.teclas_activas:
            self.teclas_activas.remove(tecla)
        
        if not self.teclas_activas:
            self.comando_movimiento("stop")
    
    def capturar_marcador(self):
        """Captura marcador de cámara frontal"""
        global frame_frontal
        
        if frame_frontal is None:
            messagebox.showwarning("Captura", "No hay frame disponible")
            return
        
        tipo, cx, cy, frame_anotado = detectar_marcador(frame_frontal.copy())
        
        if tipo is None:
            messagebox.showinfo("Detección", "No se detectó ningún marcador válido")
            return
        
        h, w = frame_frontal.shape[:2]
        es_clave = (tipo == marcador_clave.get())
        
        comando = f"marcador:{tipo}:{w}:{h}:{'true' if es_clave else 'false'}"
        respuesta = enviar_comando(comando)
        
        if es_clave:
            messagebox.showinfo("🎯 Marcador Clave", 
                              f"Marcador clave '{tipo}' detectado!\n"
                              f"Iniciando secuencia autónoma...")
        else:
            messagebox.showinfo("Marcador Detectado", 
                              f"Marcador '{tipo}' guardado\n{respuesta}")
    
    def reset_odometria(self):
        """Reinicia odometría"""
        odometria.reset()
        messagebox.showinfo("Reset", "Odometría reiniciada")
    
    def actualizar_mapa(self):
        """Actualiza mapa de odometría"""
        try:
            estado_odom = odometria.obtener_estado()
            
            self.ax.clear()
            self.ax.set_facecolor('#2d2d2d')
            self.ax.grid(True, alpha=0.3, color='white')
            
            # Dibujar trayectoria
            if len(estado_odom['historial_x']) > 1:
                self.ax.plot(estado_odom['historial_x'], 
                           estado_odom['historial_y'], 
                           'cyan', linewidth=2, label='Trayectoria')
            
            # Dibujar posición actual
            x, y, theta = estado_odom['x'], estado_odom['y'], estado_odom['theta']
            
            # Rover como triángulo
            arrow_len = 0.3
            dx = arrow_len * math.cos(theta)
            dy = arrow_len * math.sin(theta)
            self.ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.1,
                         fc='lime', ec='lime', linewidth=2)
            
            # Dibujar obstáculos detectados
            for ox, oy, otheta, dist in estado_odom['distancias'][-20:]:
                obs_x = ox + dist * math.cos(otheta)
                obs_y = oy + dist * math.sin(otheta)
                self.ax.plot(obs_x, obs_y, 'ro', markersize=4, alpha=0.6)
            
            # Configurar ejes
            max_range = max(3.0, 
                          max(abs(x), abs(y)) + 1 if x != 0 or y != 0 else 3.0)
            self.ax.set_xlim(-max_range, max_range)
            self.ax.set_ylim(-max_range, max_range)
            self.ax.set_xlabel('X (metros)', color='white')
            self.ax.set_ylabel('Y (metros)', color='white')
            self.ax.tick_params(colors='white')
            self.ax.legend(loc='upper right', facecolor='#3d3d3d', 
                          edgecolor='white', labelcolor='white')
            
            self.canvas_mapa.draw()
            
            # Actualizar info
            angulo_grados = math.degrees(theta)
            self.odom_label.config(
                text=f"Posición: ({x:.2f}, {y:.2f}) m | Ángulo: {angulo_grados:.1f}°"
            )
            
        except Exception as e:
            print(f"[ERROR] Actualizar mapa: {e}")
    
    def actualizar_gui(self):
        """Actualiza GUI con frames y estado"""
        global frame_frontal, frame_superior, estado_rover
        
        # Actualizar video frontal
        if frame_frontal is not None:
            frame = frame_frontal.copy()
            tipo, cx, cy, frame_anotado = detectar_marcador(frame)
            
            frame_rgb = cv2.cvtColor(frame_anotado, cv2.COLOR_BGR2RGB)
            frame_rgb = cv2.resize(frame_rgb, (640, 360))
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_frontal.imgtk = imgtk
            self.video_frontal.configure(image=imgtk)
        
        # Actualizar video superior
        if frame_superior is not None:
            frame_rgb = cv2.cvtColor(frame_superior, cv2.COLOR_BGR2RGB)
            frame_rgb = cv2.resize(frame_rgb, (640, 360))
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_superior.imgtk = imgtk
            self.video_superior.configure(image=imgtk)
        
        # Actualizar estado
        with estado_lock:
            if estado_rover:
                self.temp_label.config(text=f"🌡️ Temp: {estado_rover.get('temperatura', 'N/A')}")
                self.luz_label.config(text=f"💡 Luz: {estado_rover.get('luz', 'N/A')}")
                self.humedad_label.config(text=f"💧 Humedad: {estado_rover.get('humedad', 'N/A')}")
                self.dist_label.config(text=f"📏 Distancia: {estado_rover.get('distancia', 'N/A')}")
                
                modo = estado_rover.get('modo', 'manual').upper()
                color = "#ff9900" if modo == "AUTONOMO" else "#00ff00"
                self.modo_label.config(text=f"Modo: {modo}", fg=color)
                
                marcadores = estado_rover.get('marcadores_detectados', 0)
                self.marcadores_label.config(text=f"🎯 Marcadores: {marcadores}")
                
                log = estado_rover.get('ultimo_log', '')
                self.log_label.config(text=log)
        
        # Actualizar mapa
        self.actualizar_mapa()
        
        self.root.after(33, self.actualizar_gui)
    
    def cerrar(self):
        global running
        running = False
        self.root.destroy()

# ==================== MAIN ====================
if __name__ == "__main__":
    print("=" * 60)
    print("   🤖 CLIENTE ROVER CON ODOMETRÍA")
    print("=" * 60)
    print(f"\n🔗 Conectando a servidor: {SERVER_IP}")
    
    # Iniciar threads
    threading.Thread(target=recibir_video_frontal, daemon=True).start()
    threading.Thread(target=recibir_video_superior, daemon=True).start()
    threading.Thread(target=recibir_estado, daemon=True).start()
    threading.Thread(target=actualizar_odometria_thread, daemon=True).start()
    
    time.sleep(1)
    
    # Iniciar GUI
    root = tk.Tk()
    app = RoverClienteUI(root)
    
    root.protocol("WM_DELETE_WINDOW", app.cerrar)
    root.mainloop()
    
    print("\n[CLIENTE] Desconectado")