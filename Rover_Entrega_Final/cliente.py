#!/usr/bin/env python3
"""
ROVER DE EXPLORACIÓN - CLIENTE GUI
Interfaz gráfica completa para control del rover

Características:
- Vista dual de cámaras (frontal y superior)
- Control de motores (botones + teclado WASD/flechas)
- Control de servos con sliders
- Visualización de sensores en tiempo real
- Modo manual/autónomo
- Guardado de datos CSV
- Botón de emergencia
- Mapa 2D con IMU y distancia
- Contador de marcadores

Autor: Sistema Rover
Versión: 2.0
"""

import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import struct
import cv2
import numpy as np
from PIL import Image, ImageTk
import time
from datetime import datetime
import json

# =========================
# CONFIGURACIÓN
# =========================
class Config:
    # Servidor
    SERVER_IP = '192.168.1.100'  # Cambiar a IP de Raspberry Pi
    UDP_PORT = 50000
    TCP_PORT = 50001
    
    # UI
    WINDOW_TITLE = "Control Rover de Exploración"
    WINDOW_WIDTH = 1400
    WINDOW_HEIGHT = 900
    
    # Cámaras
    CAMERA_WIDTH = 480
    CAMERA_HEIGHT = 360
    
    # Control
    DEFAULT_SPEED = 80
    SPEED_MIN = 0
    SPEED_MAX = 100
    
    # Telemetría
    TELEMETRY_UPDATE_MS = 500

# =========================
# CLASE COMUNICACIÓN UDP
# =========================
class UDPClient:
    def __init__(self, server_ip, port):
        self.server_ip = server_ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)
    
    def send_command(self, command):
        """Enviar comando UDP y recibir respuesta"""
        try:
            self.sock.sendto(command.encode('utf-8'), (self.server_ip, self.port))
            data, _ = self.sock.recvfrom(4096)
            return data.decode('utf-8')
        except socket.timeout:
            return "ERR:TIMEOUT"
        except Exception as e:
            return f"ERR:{str(e)}"

# =========================
# CLASE STREAMING VIDEO TCP
# =========================
class VideoStream:
    def __init__(self, server_ip, port, camera_id):
        self.server_ip = server_ip
        self.port = port
        self.camera_id = camera_id
        self.sock = None
        self.running = False
        self.current_frame = None
        self.lock = threading.Lock()
    
    def connect(self):
        """Conectar al servidor TCP"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.server_ip, self.port))
            self.running = True
            threading.Thread(target=self._receive_frames, daemon=True).start()
            return True
        except Exception as e:
            print(f"Error conectando video: {e}")
            return False
    
    def _receive_frames(self):
        """Recibir frames continuamente"""
        try:
            while self.running:
                # Leer tamaño del frame
                size_data = self._recv_all(8)
                if not size_data:
                    break
                
                frame_size = struct.unpack("Q", size_data)[0]
                
                # Leer frame
                frame_data = self._recv_all(frame_size)
                if not frame_data:
                    break
                
                # Decodificar
                frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                
                with self.lock:
                    self.current_frame = frame
        
        except Exception as e:
            print(f"Error en recepción de video: {e}")
        finally:
            self.running = False
    
    def _recv_all(self, size):
        """Recibir cantidad exacta de bytes"""
        data = b''
        while len(data) < size:
            packet = self.sock.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def get_frame(self):
        """Obtener frame actual"""
        with self.lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def disconnect(self):
        """Desconectar stream"""
        self.running = False
        if self.sock:
            self.sock.close()

# =========================
# CLASE PRINCIPAL GUI
# =========================
class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.root.title(Config.WINDOW_TITLE)
        self.root.geometry(f"{Config.WINDOW_WIDTH}x{Config.WINDOW_HEIGHT}")
        self.root.resizable(True, True)
        
        # Cliente UDP
        self.udp_client = UDPClient(Config.SERVER_IP, Config.UDP_PORT)
        
        # Streams de video
        self.video_front = None
        self.video_top = None
        
        # Variables de estado
        self.connected = False
        self.autonomous_mode = tk.BooleanVar(value=False)
        self.saving_data = tk.BooleanVar(value=False)
        self.current_speed = tk.IntVar(value=Config.DEFAULT_SPEED)
        self.emergency_active = False
        
        # Variables de servos
        self.servo1_var = tk.IntVar(value=15)
        self.servo2_var = tk.IntVar(value=90)
        self.servo3_var = tk.IntVar(value=90)
        self.servo4_var = tk.IntVar(value=45)
        
        # Variables de sensores
        self.temp_var = tk.StringVar(value="--")
        self.light_var = tk.StringVar(value="--")
        self.dist_var = tk.StringVar(value="--")
        self.hum_var = tk.StringVar(value="--")
        self.marker_count_var = tk.StringVar(value="0")
        
        # Datos para mapa 2D
        self.map_points = []
        
        # Crear interfaz
        self.create_widgets()
        
        # Bind de teclado
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
        # Iniciar actualización de telemetría
        self.update_telemetry()
        self.update_video_feeds()
    
    def create_widgets(self):
        """Crear todos los widgets de la interfaz"""
        
        # ========== PANEL SUPERIOR: CONEXIÓN Y ESTADO ==========
        top_frame = tk.Frame(self.root, bg='#2c3e50', height=60)
        top_frame.pack(side=tk.TOP, fill=tk.X)
        top_frame.pack_propagate(False)
        
        # Título
        title_label = tk.Label(top_frame, text="🤖 ROVER DE EXPLORACIÓN", 
                              font=('Arial', 18, 'bold'), fg='white', bg='#2c3e50')
        title_label.pack(side=tk.LEFT, padx=20, pady=10)
        
        # Botón conectar
        self.btn_connect = tk.Button(top_frame, text="🔌 CONECTAR", 
                                     command=self.toggle_connection,
                                     font=('Arial', 12, 'bold'),
                                     bg='#27ae60', fg='white',
                                     width=12, height=1)
        self.btn_connect.pack(side=tk.LEFT, padx=10)
        
        # Indicador de estado
        self.status_label = tk.Label(top_frame, text="● DESCONECTADO", 
                                     font=('Arial', 12), fg='#e74c3c', bg='#2c3e50')
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # Contador de marcadores
        marker_frame = tk.Frame(top_frame, bg='#34495e', relief=tk.RIDGE, bd=2)
        marker_frame.pack(side=tk.RIGHT, padx=20, pady=10)
        
        tk.Label(marker_frame, text="Marcadores:", font=('Arial', 10), 
                bg='#34495e', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Label(marker_frame, textvariable=self.marker_count_var, 
                font=('Arial', 16, 'bold'), bg='#34495e', fg='#f39c12').pack(side=tk.LEFT, padx=5)
        
        # ========== FRAME PRINCIPAL ==========
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # ========== COLUMNA IZQUIERDA: CÁMARAS ==========
        left_column = tk.Frame(main_frame, bg='#ecf0f1')
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Cámara frontal
        cam_front_frame = tk.LabelFrame(left_column, text="📹 CÁMARA FRONTAL", 
                                       font=('Arial', 11, 'bold'), bg='#ecf0f1')
        cam_front_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        self.canvas_front = tk.Canvas(cam_front_frame, width=Config.CAMERA_WIDTH, 
                                     height=Config.CAMERA_HEIGHT, bg='black')
        self.canvas_front.pack(padx=5, pady=5)
        
        # Cámara superior
        cam_top_frame = tk.LabelFrame(left_column, text="📹 CÁMARA SUPERIOR", 
                                     font=('Arial', 11, 'bold'), bg='#ecf0f1')
        cam_top_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        
        self.canvas_top = tk.Canvas(cam_top_frame, width=Config.CAMERA_WIDTH, 
                                   height=Config.CAMERA_HEIGHT, bg='black')
        self.canvas_top.pack(padx=5, pady=5)
        
        # ========== COLUMNA CENTRAL: CONTROLES ==========
        center_column = tk.Frame(main_frame, bg='#ecf0f1', width=400)
        center_column.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        center_column.pack_propagate(False)
        
        # === MODOS DE OPERACIÓN ===
        mode_frame = tk.LabelFrame(center_column, text="⚙️ MODO DE OPERACIÓN", 
                                  font=('Arial', 10, 'bold'), bg='#ecf0f1')
        mode_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Radiobutton(mode_frame, text="Manual", variable=self.autonomous_mode, 
                      value=False, font=('Arial', 10), bg='#ecf0f1',
                      command=self.change_mode).pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Radiobutton(mode_frame, text="Autónomo", variable=self.autonomous_mode, 
                      value=True, font=('Arial', 10), bg='#ecf0f1',
                      command=self.change_mode).pack(side=tk.LEFT, padx=10)
        
        # === CONTROL DE MOVIMIENTO ===
        move_frame = tk.LabelFrame(center_column, text="🕹️ CONTROL DE MOVIMIENTO", 
                                  font=('Arial', 10, 'bold'), bg='#ecf0f1')
        move_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Velocidad
        speed_frame = tk.Frame(move_frame, bg='#ecf0f1')
        speed_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(speed_frame, text="Velocidad:", font=('Arial', 9), 
                bg='#ecf0f1').pack(side=tk.LEFT)
        
        speed_slider = tk.Scale(speed_frame, from_=Config.SPEED_MIN, to=Config.SPEED_MAX,
                               orient=tk.HORIZONTAL, variable=self.current_speed,
                               bg='#ecf0f1', length=200)
        speed_slider.pack(side=tk.LEFT, padx=5)
        
        tk.Label(speed_frame, textvariable=self.current_speed, 
                font=('Arial', 9, 'bold'), bg='#ecf0f1', width=3).pack(side=tk.LEFT)
        
        # Botones de dirección
        btn_frame = tk.Frame(move_frame, bg='#ecf0f1')
        btn_frame.pack(pady=10)
        
        # Fila 1: Adelante
        btn_forward = tk.Button(btn_frame, text="▲", command=lambda: self.move('forward'),
                               font=('Arial', 14, 'bold'), width=4, height=1,
                               bg='#3498db', fg='white')
        btn_forward.grid(row=0, column=1, padx=2, pady=2)
        
        # Fila 2: Izquierda, Stop, Derecha
        btn_left = tk.Button(btn_frame, text="◄", command=lambda: self.move('left'),
                            font=('Arial', 14, 'bold'), width=4, height=1,
                            bg='#3498db', fg='white')
        btn_left.grid(row=1, column=0, padx=2, pady=2)
        
        btn_stop = tk.Button(btn_frame, text="■", command=lambda: self.move('stop'),
                            font=('Arial', 14, 'bold'), width=4, height=1,
                            bg='#e74c3c', fg='white')
        btn_stop.grid(row=1, column=1, padx=2, pady=2)
        
        btn_right = tk.Button(btn_frame, text="►", command=lambda: self.move('right'),
                             font=('Arial', 14, 'bold'), width=4, height=1,
                             bg='#3498db', fg='white')
        btn_right.grid(row=1, column=2, padx=2, pady=2)
        
        # Fila 3: Atrás
        btn_back = tk.Button(btn_frame, text="▼", command=lambda: self.move('back'),
                            font=('Arial', 14, 'bold'), width=4, height=1,
                            bg='#3498db', fg='white')
        btn_back.grid(row=2, column=1, padx=2, pady=2)
        
        # Texto de ayuda
        tk.Label(move_frame, text="Usar teclas: W/A/S/D o Flechas", 
                font=('Arial', 8), bg='#ecf0f1', fg='#7f8c8d').pack(pady=2)
        
        # === CONTROL DE SERVOS ===
        servo_frame = tk.LabelFrame(center_column, text="🔧 CONTROL DE SERVOS", 
                                   font=('Arial', 10, 'bold'), bg='#ecf0f1')
        servo_frame.pack(fill=tk.X, padx=5, pady=5)
        
        servos_data = [
            ("Servo 1 (Cámara 0-30°)", self.servo1_var, 0, 30),
            ("Servo 2 (Brazo hombro)", self.servo2_var, 0, 180),
            ("Servo 3 (Brazo codo)", self.servo3_var, 0, 180),
            ("Servo 4 (Sensor hum. 0-90°)", self.servo4_var, 0, 90)
        ]
        
        for i, (label, var, min_val, max_val) in enumerate(servos_data, 1):
            frame = tk.Frame(servo_frame, bg='#ecf0f1')
            frame.pack(fill=tk.X, padx=10, pady=3)
            
            tk.Label(frame, text=label, font=('Arial', 8), 
                    bg='#ecf0f1', width=20, anchor='w').pack(side=tk.LEFT)
            
            slider = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                            variable=var, bg='#ecf0f1', length=150,
                            command=lambda v, sid=i: self.set_servo(sid, v))
            slider.pack(side=tk.LEFT, padx=5)
            
            tk.Label(frame, textvariable=var, font=('Arial', 8, 'bold'), 
                    bg='#ecf0f1', width=3).pack(side=tk.LEFT)
        
        # === ACCIONES ===
        action_frame = tk.LabelFrame(center_column, text="⚡ ACCIONES", 
                                    font=('Arial', 10, 'bold'), bg='#ecf0f1')
        action_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Botón de guardado
        save_check = tk.Checkbutton(action_frame, text="Guardado continuo", 
                                   variable=self.saving_data, font=('Arial', 9),
                                   bg='#ecf0f1', command=self.toggle_saving)
        save_check.pack(pady=5)
        
        btn_save_point = tk.Button(action_frame, text="💾 Guardar Punto", 
                                   command=self.save_point,
                                   font=('Arial', 9), bg='#16a085', fg='white')
        btn_save_point.pack(fill=tk.X, padx=10, pady=2)
        
        btn_take_sample = tk.Button(action_frame, text="🔬 Tomar Muestra", 
                                    command=self.take_sample,
                                    font=('Arial', 9), bg='#8e44ad', fg='white')
        btn_take_sample.pack(fill=tk.X, padx=10, pady=2)
        
        # === EMERGENCIA ===
        emergency_frame = tk.Frame(center_column, bg='#e74c3c', relief=tk.RAISED, bd=3)
        emergency_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.btn_emergency = tk.Button(emergency_frame, text="🚨 PARADA DE EMERGENCIA", 
                                       command=self.emergency_stop,
                                       font=('Arial', 12, 'bold'),
                                       bg='#c0392b', fg='white', height=2)
        self.btn_emergency.pack(fill=tk.X, padx=5, pady=5)
        
        # ========== COLUMNA DERECHA: TELEMETRÍA Y MAPA ==========
        right_column = tk.Frame(main_frame, bg='#ecf0f1', width=350)
        right_column.pack(side=tk.LEFT, fill=tk.BOTH, padx=(5, 0))
        right_column.pack_propagate(False)
        
        # === SENSORES ===
        sensor_frame = tk.LabelFrame(right_column, text="📊 TELEMETRÍA", 
                                    font=('Arial', 10, 'bold'), bg='#ecf0f1')
        sensor_frame.pack(fill=tk.X, padx=5, pady=5)
        
        sensors = [
            ("🌡️ Temperatura:", self.temp_var, "°C"),
            ("💡 Luz:", self.light_var, "V"),
            ("📏 Distancia:", self.dist_var, "cm"),
            ("💧 Humedad:", self.hum_var, "%")
        ]
        
        for label, var, unit in sensors:
            frame = tk.Frame(sensor_frame, bg='#ecf0f1')
            frame.pack(fill=tk.X, padx=10, pady=3)
            
            tk.Label(frame, text=label, font=('Arial', 9), 
                    bg='#ecf0f1', width=15, anchor='w').pack(side=tk.LEFT)
            
            value_label = tk.Label(frame, textvariable=var, font=('Arial', 11, 'bold'), 
                                  bg='white', fg='#2c3e50', width=8, anchor='e',
                                  relief=tk.SUNKEN)
            value_label.pack(side=tk.LEFT, padx=5)
            
            tk.Label(frame, text=unit, font=('Arial', 9), 
                    bg='#ecf0f1').pack(side=tk.LEFT)
        
        # === MAPA 2D ===
        map_frame = tk.LabelFrame(right_column, text="🗺️ MAPA 2D", 
                                 font=('Arial', 10, 'bold'), bg='#ecf0f1')
        map_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.canvas_map = tk.Canvas(map_frame, bg='white', width=320, height=320)
        self.canvas_map.pack(padx=5, pady=5)
        
        # Botón limpiar mapa
        tk.Button(map_frame, text="🧹 Limpiar Mapa", command=self.clear_map,
                 font=('Arial', 8), bg='#95a5a6', fg='white').pack(pady=2)
        
        # === LOG DE EVENTOS ===
        log_frame = tk.LabelFrame(right_column, text="📝 LOG DE EVENTOS", 
                                 font=('Arial', 10, 'bold'), bg='#ecf0f1')
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Scrollbar y Text
        scroll = tk.Scrollbar(log_frame)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log_text = tk.Text(log_frame, height=10, font=('Courier', 8),
                               bg='#2c3e50', fg='#ecf0f1', 
                               yscrollcommand=scroll.set, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        scroll.config(command=self.log_text.yview)
        
        self.log("Sistema iniciado")
    
    # =========================
    # MÉTODOS DE CONEXIÓN
    # =========================
    def toggle_connection(self):
        """Conectar/desconectar del servidor"""
        if not self.connected:
            # Intentar conectar
            response = self.udp_client.send_command("status")
            if "ERR" not in response:
                self.connected = True
                self.status_label.config(text="● CONECTADO", fg='#27ae60')
                self.btn_connect.config(text="🔌 DESCONECTAR", bg='#e74c3c')
                
                # Iniciar streams de video
                self.video_front = VideoStream(Config.SERVER_IP, Config.TCP_PORT, 0)
                if self.video_front.connect():
                    self.log("Video frontal conectado")
                
                self.log("Conexión establecida")
            else:
                messagebox.showerror("Error", "No se pudo conectar con el servidor")
        else:
            # Desconectar
            self.connected = False
            self.status_label.config(text="● DESCONECTADO", fg='#e74c3c')
            self.btn_connect.config(text="🔌 CONECTAR", bg='#27ae60')
            
            if self.video_front:
                self.video_front.disconnect()
            
            self.log("Desconectado")
    
    # =========================
    # MÉTODOS DE CONTROL
    # =========================
    def move(self, direction):
        """Enviar comando de movimiento"""
        if not self.connected:
            return
        
        speed = self.current_speed.get()
        cmd = f"move:{direction}:{speed}"
        response = self.udp_client.send_command(cmd)
        
        if "OK" in response:
            self.log(f"Movimiento: {direction} a velocidad {speed}")
        else:
            self.log(f"Error en movimiento: {response}", level="ERROR")
    
    def set_servo(self, servo_id, angle):
        """Enviar comando de servo"""
        if not self.connected:
            return
        
        cmd = f"servo:{servo_id}:{angle}"
        response = self.udp_client.send_command(cmd)
        
        if "OK" in response:
            self.log(f"Servo {servo_id} → {angle}°")
    
    def change_mode(self):
        """Cambiar modo manual/autónomo"""
        mode = "autónomo" if self.autonomous_mode.get() else "manual"
        self.log(f"Modo cambiado a: {mode}")
    
    def toggle_saving(self):
        """Activar/desactivar guardado continuo"""
        if not self.connected:
            self.saving_data.set(False)
            return
        
        if self.saving_data.get():
            cmd = "save:start"
            self.log("Guardado continuo activado")
        else:
            cmd = "save:stop"
            self.log("Guardado continuo desactivado")
        
        self.udp_client.send_command(cmd)
    
    def save_point(self):
        """Guardar punto individual"""
        if not self.connected:
            return
        
        response = self.udp_client.send_command("save:point")
        if "OK" in response:
            self.log("Punto guardado en CSV")
        else:
            self.log("Error al guardar punto", level="ERROR")
    
    def take_sample(self):
        """Tomar muestra de humedad (activar brazo)"""
        if not self.connected:
            return
        
        self.log("Tomando muestra de humedad...")
        # Este comando debería ser procesado por el servidor
        # que a su vez enviará ACTUATE BRACO START al Arduino
        response = self.udp_client.send_command("action:sample")
        self.log(f"Resultado: {response}")
    
    def emergency_stop(self):
        """Parada de emergencia"""
        if not self.connected:
            return
        
        result = messagebox.askyesno("Confirmación", 
                                    "¿Activar parada de emergencia?\nEl rover se detendrá inmediatamente.")
        
        if result:
            self.udp_client.send_command("emergency:stop")
            self.emergency_active = True
            self.btn_emergency.config(text="⚠️ EMERGENCIA ACTIVA", bg='#e67e22')
            self.log("PARADA DE EMERGENCIA ACTIVADA", level="CRITICAL")
            
            # Mostrar ventana para reactivar
            self.show_emergency_clear_dialog()
    
    def show_emergency_clear_dialog(self):
        """Diálogo para limpiar emergencia"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Emergencia Activa")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()
        
        tk.Label(dialog, text="⚠️ Emergencia Activa", 
                font=('Arial', 14, 'bold'), fg='red').pack(pady=10)
        
        tk.Label(dialog, text="El rover está detenido.\n¿Desea reactivar el sistema?",
                font=('Arial', 10)).pack(pady=10)
        
        def clear_emergency():
            self.udp_client.send_command("emergency:clear")
            self.emergency_active = False
            self.btn_emergency.config(text="🚨 PARADA DE EMERGENCIA", bg='#c0392b')
            self.log("Emergencia limpiada, sistema reactivado")
            dialog.destroy()
        
        tk.Button(dialog, text="✓ Reactivar Sistema", command=clear_emergency,
                 bg='#27ae60', fg='white', font=('Arial', 10, 'bold')).pack(pady=10)
    
    # =========================
    # MÉTODOS DE TECLADO
    # =========================
    def on_key_press(self, event):
        """Manejo de teclas presionadas"""
        if not self.connected or self.emergency_active:
            return
        
        key = event.keysym.lower()
        
        # WASD
        if key == 'w':
            self.move('forward')
        elif key == 's':
            self.move('back')
        elif key == 'a':
            self.move('left')
        elif key == 'd':
            self.move('right')
        
        # Flechas
        elif key == 'up':
            self.move('forward')
        elif key == 'down':
            self.move('back')
        elif key == 'left':
            self.move('left')
        elif key == 'right':
            self.move('right')
    
    def on_key_release(self, event):
        """Manejo de teclas liberadas"""
        if not self.connected or self.emergency_active:
            return
        
        key = event.keysym.lower()
        
        # Detener al soltar tecla de movimiento
        if key in ['w', 's', 'a', 'd', 'up', 'down', 'left', 'right']:
            self.move('stop')
    
    # =========================
    # MÉTODOS DE ACTUALIZACIÓN
    # =========================
    def update_telemetry(self):
        """Actualizar datos de telemetría"""
        if self.connected:
            # Temperatura
            resp = self.udp_client.send_command("sensor:temp")
            if resp.startswith("TEMP:"):
                self.temp_var.set(resp.split(':')[1][:5])
            
            # Luz
            resp = self.udp_client.send_command("sensor:light")
            if resp.startswith("LDR:"):
                self.light_var.set(resp.split(':')[1][:4])
            
            # Distancia
            resp = self.udp_client.send_command("sensor:dist")
            if resp.startswith("DIST:"):
                dist = resp.split(':')[1]
                self.dist_var.set(dist)
                
                # Actualizar mapa 2D
                try:
                    dist_cm = float(dist) if dist != "OUT_OF_RANGE" else 0
                    self.update_map(dist_cm)
                except:
                    pass
            
            # Humedad
            resp = self.udp_client.send_command("sensor:hum")
            if resp.startswith("HUM:"):
                self.hum_var.set(resp.split(':')[1])
            
            # Estado (incluye contador de marcadores)
            resp = self.udp_client.send_command("status")
            if "MARKERS:" in resp:
                parts = resp.split(',')
                for part in parts:
                    if "MARKERS:" in part:
                        count = part.split(':')[1]
                        self.marker_count_var.set(count)
        
        # Re-programar
        self.root.after(Config.TELEMETRY_UPDATE_MS, self.update_telemetry)
    
    def update_video_feeds(self):
        """Actualizar feeds de video"""
        # Cámara frontal
        if self.video_front and self.video_front.running:
            frame = self.video_front.get_frame()
            if frame is not None:
                self.display_frame(frame, self.canvas_front)
        
        # Re-programar
        self.root.after(33, self.update_video_feeds)  # ~30 FPS
    
    def display_frame(self, frame, canvas):
        """Mostrar frame en canvas"""
        # Redimensionar
        frame_resized = cv2.resize(frame, (Config.CAMERA_WIDTH, Config.CAMERA_HEIGHT))
        
        # Convertir BGR a RGB
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        
        # Convertir a ImageTk
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        
        # Mostrar en canvas
        canvas.delete("all")
        canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
        canvas.image = imgtk  # Mantener referencia
    
    def update_map(self, distance):
        """Actualizar mapa 2D con nueva lectura"""
        # Centro del canvas
        center_x = 160
        center_y = 280
        
        # Escala: 1 pixel = 1 cm
        scale = 2
        
        # Agregar punto (simplificado sin IMU por ahora)
        angle = 0  # Aquí se usaría el heading del IMU
        
        x = center_x + distance * scale * np.cos(np.radians(angle))
        y = center_y - distance * scale * np.sin(np.radians(angle))
        
        self.map_points.append((x, y))
        
        # Redibujar mapa
        self.draw_map()
    
    def draw_map(self):
        """Dibujar mapa 2D"""
        self.canvas_map.delete("all")
        
        # Dibujar grid
        for i in range(0, 320, 40):
            self.canvas_map.create_line(i, 0, i, 320, fill='#ecf0f1')
            self.canvas_map.create_line(0, i, 320, i, fill='#ecf0f1')
        
        # Dibujar rover en el centro
        center_x, center_y = 160, 280
        self.canvas_map.create_oval(center_x-5, center_y-5, center_x+5, center_y+5,
                                    fill='red', outline='darkred', width=2)
        
        # Dibujar puntos de distancia
        for i, (x, y) in enumerate(self.map_points[-50:]):  # Últimos 50 puntos
            color = f'#{200-i*2:02x}{100+i*2:02x}00'  # Gradiente
            self.canvas_map.create_oval(x-2, y-2, x+2, y+2, fill=color, outline=color)
    
    def clear_map(self):
        """Limpiar mapa"""
        self.map_points = []
        self.canvas_map.delete("all")
        self.log("Mapa limpiado")
    
    # =========================
    # MÉTODOS AUXILIARES
    # =========================
    def log(self, message, level="INFO"):
        """Agregar mensaje al log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color según nivel
        colors = {
            "INFO": "#27ae60",
            "WARNING": "#f39c12",
            "ERROR": "#e74c3c",
            "CRITICAL": "#c0392b"
        }
        
        color = colors.get(level, "#ecf0f1")
        
        self.log_text.insert(tk.END, f"[{timestamp}] ", 'time')
        self.log_text.insert(tk.END, f"[{level}] ", 'level')
        self.log_text.insert(tk.END, f"{message}\n")
        
        # Tags de color
        self.log_text.tag_config('time', foreground='#3498db')
        self.log_text.tag_config('level', foreground=color, font=('Courier', 8, 'bold'))
        
        # Auto-scroll
        self.log_text.see(tk.END)

# =========================
# MAIN
# =========================
def main():
    root = tk.Tk()
    app = RoverGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()