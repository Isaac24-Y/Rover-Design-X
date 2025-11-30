#!/usr/bin/env python3
"""
ROVER DE EXPLORACIÓN - CLIENTE GUI

Funcionalidades:
- Control manual y automático
- Vista dual de cámaras
- Panel de telemetría en tiempo real
- Control por teclado (WASD + flechas)
- Sliders para servos
- Sistema de guardado CSV
- Botón de emergencia
- Visualización de mapa 2D

Autor: Sistema Rover
Versión: 2.0
"""

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
from collections import deque

# =========================
# CONFIGURACIÓN
# =========================
class Config:
    SERVER_IP = "192.168.1.100"  # Cambiar a IP de Raspberry Pi
    UDP_PORT = 50000
    TCP_PORT = 50001
    BUFFER_SIZE = 4096
    
    # UI
    WINDOW_WIDTH = 1400
    WINDOW_HEIGHT = 800
    VIDEO_WIDTH = 640
    VIDEO_HEIGHT = 480
    
    # Control
    DEFAULT_SPEED = 80
    SERVO_UPDATE_DELAY = 100  # ms

# =========================
# CLASE DE NETWORKING
# =========================
class NetworkManager:
    def __init__(self, ip, udp_port, tcp_port):
        self.server_ip = ip
        self.udp_port = udp_port
        self.tcp_port = tcp_port
        self.udp_lock = threading.Lock()
        self.connected = False
    
    def send_udp_command(self, command, timeout=2.0):
        """Enviar comando UDP y esperar respuesta"""
        try:
            with self.udp_lock:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(timeout)
                sock.sendto(command.encode('utf-8'), (self.server_ip, self.udp_port))
                data, _ = sock.recvfrom(Config.BUFFER_SIZE)
                response = data.decode('utf-8')
                sock.close()
                self.connected = True
                return response
        except socket.timeout:
            self.connected = False
            return "⚠️ Timeout: sin respuesta del servidor"
        except Exception as e:
            self.connected = False
            return f"❌ Error: {e}"
    
    def send_udp_no_response(self, command):
        """Enviar comando UDP sin esperar respuesta"""
        try:
            with self.udp_lock:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.1)
                sock.sendto(command.encode('utf-8'), (self.server_ip, self.udp_port))
                sock.close()
        except:
            pass

# =========================
# CLASE VIDEO RECEIVER
# =========================
class VideoReceiver:
    def __init__(self, ip, port, callback):
        self.server_ip = ip
        self.port = port
        self.callback = callback
        self.running = False
        self.paused = False
        self.thread = None
    
    def start(self):
        """Iniciar recepción de video"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Detener recepción de video"""
        self.running = False
    
    def toggle_pause(self):
        """Pausar/reanudar video"""
        self.paused = not self.paused
        return self.paused
    
    def _receive_loop(self):
        """Loop principal de recepción"""
        while self.running:
            try:
                # Conectar
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.server_ip, self.port))
                sock.settimeout(None)
                print(f"[VIDEO] Conectado a {self.server_ip}:{self.port}")
                
                data = b""
                payload_size = struct.calcsize("Q")
                
                while self.running:
                    # Leer tamaño del frame
                    while len(data) < payload_size:
                        packet = sock.recv(Config.BUFFER_SIZE)
                        if not packet:
                            raise ConnectionError("Servidor desconectado")
                        data += packet
                    
                    packed_size = data[:payload_size]
                    data = data[payload_size:]
                    frame_size = struct.unpack("Q", packed_size)[0]
                    
                    # Leer frame completo
                    while len(data) < frame_size:
                        packet = sock.recv(Config.BUFFER_SIZE)
                        if not packet:
                            raise ConnectionError("Servidor desconectado")
                        data += packet
                    
                    frame_data = data[:frame_size]
                    data = data[frame_size:]
                    
                    # Decodificar frame
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    
                    if frame is not None and not self.paused:
                        self.callback(frame)
                
            except Exception as e:
                print(f"[VIDEO] Error: {e}")
                time.sleep(2)  # Esperar antes de reconectar
            
            finally:
                try:
                    sock.close()
                except:
                    pass

# =========================
# CLASE MAPA 2D
# =========================
class Map2D:
    def __init__(self, width=400, height=400):
        self.width = width
        self.height = height
        self.map_img = np.ones((height, width, 3), dtype=np.uint8) * 240
        self.robot_pos = (width // 2, height // 2)
        self.robot_angle = 0
        self.points = deque(maxlen=1000)
        self.scale = 10  # pixels por metro
    
    def update_position(self, distance_cm, angle_change=0):
        """Actualizar posición del robot"""
        self.robot_angle += angle_change
        
        # Calcular nueva posición
        distance_px = (distance_cm / 100) * self.scale
        dx = int(distance_px * np.cos(np.radians(self.robot_angle)))
        dy = int(distance_px * np.sin(np.radians(self.robot_angle)))
        
        new_x = self.robot_pos[0] + dx
        new_y = self.robot_pos[1] + dy
        
        # Mantener dentro de límites
        new_x = max(20, min(self.width - 20, new_x))
        new_y = max(20, min(self.height - 20, new_y))
        
        self.robot_pos = (new_x, new_y)
        self.points.append(self.robot_pos)
    
    def add_obstacle(self, distance_cm):
        """Agregar obstáculo detectado"""
        distance_px = (distance_cm / 100) * self.scale
        obs_x = int(self.robot_pos[0] + distance_px * np.cos(np.radians(self.robot_angle)))
        obs_y = int(self.robot_pos[1] + distance_px * np.sin(np.radians(self.robot_angle)))
        
        if 0 <= obs_x < self.width and 0 <= obs_y < self.height:
            cv2.circle(self.map_img, (obs_x, obs_y), 3, (0, 0, 255), -1)
    
    def get_map_image(self):
        """Obtener imagen del mapa"""
        img = self.map_img.copy()
        
        # Dibujar trayectoria
        if len(self.points) > 1:
            for i in range(1, len(self.points)):
                cv2.line(img, self.points[i-1], self.points[i], (0, 255, 0), 2)
        
        # Dibujar robot
        cv2.circle(img, self.robot_pos, 8, (255, 0, 0), -1)
        
        # Dibujar dirección
        end_x = int(self.robot_pos[0] + 20 * np.cos(np.radians(self.robot_angle)))
        end_y = int(self.robot_pos[1] + 20 * np.sin(np.radians(self.robot_angle)))
        cv2.arrowedLine(img, self.robot_pos, (end_x, end_y), (255, 0, 0), 2)
        
        return img
    
    def reset(self):
        """Resetear mapa"""
        self.map_img = np.ones((self.height, self.width, 3), dtype=np.uint8) * 240
        self.robot_pos = (self.width // 2, self.height // 2)
        self.robot_angle = 0
        self.points.clear()

# =========================
# CLASE GUI PRINCIPAL
# =========================
class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🚀 Control de Rover de Exploración v2.0")
        self.root.geometry(f"{Config.WINDOW_WIDTH}x{Config.WINDOW_HEIGHT}")
        self.root.configure(bg='#2b2b2b')
        
        # Variables
        self.mode_auto = tk.BooleanVar(value=False)
        self.speed_var = tk.IntVar(value=Config.DEFAULT_SPEED)
        self.servo_vars = [tk.IntVar(value=90) for _ in range(4)]
        self.marker_count = tk.IntVar(value=0)
        self.saving = tk.BooleanVar(value=False)
        
        # Datos de sensores
        self.sensor_temp = tk.StringVar(value="-- °C")
        self.sensor_light = tk.StringVar(value="-- V")
        self.sensor_dist = tk.StringVar(value="-- cm")
        self.sensor_hum = tk.StringVar(value="-- %")
        self.connection_status = tk.StringVar(value="🔴 Desconectado")
        
        # Managers
        self.network = NetworkManager(Config.SERVER_IP, Config.UDP_PORT, Config.TCP_PORT)
        self.video_receiver = VideoReceiver(Config.SERVER_IP, Config.TCP_PORT, self.update_video_frame)
        self.map_2d = Map2D()
        
        # Control de movimiento
        self.moving = False
        self.current_direction = None
        
        # Frames de video
        self.current_frame = None
        
        # Construir UI
        self.build_ui()
        
        # Iniciar sistemas
        self.video_receiver.start()
        self.start_sensor_updates()
        
        # Bind de teclado
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def build_ui(self):
        """Construir interfaz de usuario"""
        # Frame principal
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # ===== COLUMNA IZQUIERDA: VIDEO Y MAPA =====
        left_frame = tk.Frame(main_frame, bg='#2b2b2b')
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Video principal
        video_frame = tk.LabelFrame(left_frame, text="📹 Cámara Frontal", 
                                   bg='#2b2b2b', fg='white', font=('Arial', 12, 'bold'))
        video_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.video_label = tk.Label(video_frame, bg='black')
        self.video_label.pack(padx=5, pady=5)
        
        # Mapa 2D
        map_frame = tk.LabelFrame(left_frame, text="🗺️ Mapa 2D", 
                                 bg='#2b2b2b', fg='white', font=('Arial', 12, 'bold'))
        map_frame.pack(fill=tk.BOTH, pady=(0, 10))
        
        self.map_label = tk.Label(map_frame, bg='white')
        self.map_label.pack(padx=5, pady=5)
        
        # Log de eventos
        log_frame = tk.LabelFrame(left_frame, text="📋 Log de Eventos", 
                                 bg='#2b2b2b', fg='white', font=('Arial', 10, 'bold'))
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, bg='#1e1e1e', 
                                                  fg='#00ff00', font=('Courier', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # ===== COLUMNA DERECHA: CONTROLES =====
        right_frame = tk.Frame(main_frame, bg='#2b2b2b', width=400)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH)
        right_frame.pack_propagate(False)
        
        # Estado de conexión
        status_frame = tk.Frame(right_frame, bg='#3b3b3b', relief=tk.RAISED, bd=2)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(status_frame, textvariable=self.connection_status, 
                bg='#3b3b3b', fg='white', font=('Arial', 11, 'bold')).pack(pady=5)
        
        # Modo manual/automático
        mode_frame = tk.LabelFrame(right_frame, text="⚙️ Modo de Operación", 
                                  bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        mode_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Radiobutton(mode_frame, text="Manual", variable=self.mode_auto, value=False,
                      bg='#2b2b2b', fg='white', selectcolor='#1e1e1e',
                      font=('Arial', 10), command=self.change_mode).pack(anchor=tk.W, padx=10, pady=2)
        tk.Radiobutton(mode_frame, text="Automático", variable=self.mode_auto, value=True,
                      bg='#2b2b2b', fg='white', selectcolor='#1e1e1e',
                      font=('Arial', 10), command=self.change_mode).pack(anchor=tk.W, padx=10, pady=2)
        
        # Contador de marcadores
        marker_frame = tk.Frame(mode_frame, bg='#2b2b2b')
        marker_frame.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(marker_frame, text="Marcadores detectados:", bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
        tk.Label(marker_frame, textvariable=self.marker_count, bg='#2b2b2b', fg='#00ff00',
                font=('Arial', 12, 'bold')).pack(side=tk.LEFT, padx=5)
        
        # Control de velocidad
        speed_frame = tk.LabelFrame(right_frame, text="🎚️ Velocidad", 
                                   bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        speed_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Scale(speed_frame, from_=0, to=100, orient=tk.HORIZONTAL, 
                variable=self.speed_var, bg='#2b2b2b', fg='white',
                troughcolor='#1e1e1e', activebackground='#4a9eff',
                length=350, label="Velocidad (%)", font=('Arial', 9)).pack(padx=10, pady=5)
        
        # Controles de movimiento
        move_frame = tk.LabelFrame(right_frame, text="🎮 Control de Movimiento", 
                                  bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        move_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Botones direccionales
        btn_frame = tk.Frame(move_frame, bg='#2b2b2b')
        btn_frame.pack(pady=10)
        
        btn_style = {'width': 10, 'height': 2, 'font': ('Arial', 10, 'bold')}
        
        self.btn_forward = tk.Button(btn_frame, text="↑\nAdelante", bg='#4a9eff', **btn_style)
        self.btn_forward.grid(row=0, column=1, padx=5, pady=5)
        self.btn_forward.bind('<ButtonPress-1>', lambda e: self.start_move('forward'))
        self.btn_forward.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        self.btn_left = tk.Button(btn_frame, text="←\nIzquierda", bg='#4a9eff', **btn_style)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_left.bind('<ButtonPress-1>', lambda e: self.start_move('left'))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        self.btn_stop = tk.Button(btn_frame, text="■\nSTOP", bg='#ff4a4a', 
                                 command=self.emergency_stop, **btn_style)
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        
        self.btn_right = tk.Button(btn_frame, text="→\nDerecha", bg='#4a9eff', **btn_style)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_right.bind('<ButtonPress-1>', lambda e: self.start_move('right'))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        self.btn_back = tk.Button(btn_frame, text="↓\nAtrás", bg='#4a9eff', **btn_style)
        self.btn_back.grid(row=2, column=1, padx=5, pady=5)
        self.btn_back.bind('<ButtonPress-1>', lambda e: self.start_move('back'))
        self.btn_back.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        tk.Label(move_frame, text="Control: WASD o Flechas", bg='#2b2b2b', 
                fg='#aaaaaa', font=('Arial', 9, 'italic')).pack()
        
        # Control de servos
        servo_frame = tk.LabelFrame(right_frame, text="🦾 Control de Servos", 
                                   bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        servo_frame.pack(fill=tk.X, pady=(0, 10))
        
        servo_names = ["Cámara (0-30°)", "Brazo 1 (0-180°)", 
                      "Brazo 2 (0-180°)", "Sensor (0-90°)"]
        
        for i, name in enumerate(servo_names):
            frame = tk.Frame(servo_frame, bg='#2b2b2b')
            frame.pack(fill=tk.X, padx=10, pady=3)
            
            tk.Label(frame, text=name, bg='#2b2b2b', fg='white', 
                    width=20, anchor=tk.W).pack(side=tk.LEFT)
            
            scale = tk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL,
                           variable=self.servo_vars[i], bg='#2b2b2b', fg='white',
                           troughcolor='#1e1e1e', activebackground='#4a9eff',
                           length=150, showvalue=True,
                           command=lambda v, idx=i: self.servo_changed(idx, v))
            scale.pack(side=tk.LEFT, padx=5)
        
        # Telemetría
        telemetry_frame = tk.LabelFrame(right_frame, text="📊 Telemetría", 
                                       bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        telemetry_frame.pack(fill=tk.X, pady=(0, 10))
        
        sensors = [
            ("🌡️ Temperatura:", self.sensor_temp, '#ff6b6b'),
            ("💡 Luz:", self.sensor_light, '#ffd93d'),
            ("📏 Distancia:", self.sensor_dist, '#6bcf7f'),
            ("💧 Humedad:", self.sensor_hum, '#4a9eff')
        ]
        
        for label, var, color in sensors:
            frame = tk.Frame(telemetry_frame, bg='#2b2b2b')
            frame.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(frame, text=label, bg='#2b2b2b', fg='white', 
                    width=15, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(frame, textvariable=var, bg='#2b2b2b', fg=color,
                    font=('Arial', 10, 'bold'), width=12, anchor=tk.E).pack(side=tk.RIGHT)
        
        # Controles de guardado
        save_frame = tk.LabelFrame(right_frame, text="💾 Guardado de Datos", 
                                  bg='#2b2b2b', fg='white', font=('Arial', 11, 'bold'))
        save_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Button(save_frame, text="▶️ Iniciar Guardado", command=self.start_saving,
                 bg='#4a9eff', fg='white', font=('Arial', 10, 'bold')).pack(fill=tk.X, padx=10, pady=3)
        tk.Button(save_frame, text="⏸️ Detener Guardado", command=self.stop_saving,
                 bg='#ff9800', fg='white', font=('Arial', 10, 'bold')).pack(fill=tk.X, padx=10, pady=3)
        tk.Button(save_frame, text="📸 Guardar Punto", command=self.save_point,
                 bg='#9c27b0', fg='white', font=('Arial', 10, 'bold')).pack(fill=tk.X, padx=10, pady=3)
        
        # Botones de acción
        action_frame = tk.Frame(right_frame, bg='#2b2b2b')
        action_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Button(action_frame, text="⏸️ Pausar Video", command=self.toggle_video,
                 bg='#607d8b', fg='white', font=('Arial', 10, 'bold')).pack(fill=tk.X, padx=10, pady=3)
        tk.Button(action_frame, text="🔄 Resetear Mapa", command=self.reset_map,
                 bg='#607d8b', fg='white', font=('Arial', 10, 'bold')).pack(fill=tk.X, padx=10, pady=3)
        
        # Botón de emergencia (grande y rojo)
        emergency_frame = tk.Frame(right_frame, bg='#2b2b2b')
        emergency_frame.pack(fill=tk.X)
        
        tk.Button(emergency_frame, text="🚨 EMERGENCIA STOP 🚨", 
                 command=self.emergency_stop, bg='#d32f2f', fg='white',
                 font=('Arial', 14, 'bold'), height=2).pack(fill=tk.X, padx=10, pady=10)
    
    # ===== MÉTODOS DE CONTROL =====
    
    def start_move(self, direction):
        """Iniciar movimiento"""
        if not self.moving:
            self.moving = True
            self.current_direction = direction
            self._send_move_command()
    
    def stop_move(self):
        """Detener movimiento"""
        self.moving = False
        self.current_direction = None
        self.network.send_udp_no_response("move:stop")
        self.log_event("Motores detenidos")
    
    def _send_move_command(self):
        """Enviar comandos de movimiento repetidamente"""
        if self.moving and self.current_direction:
            vel = self.speed_var.get()
            cmd = f"move:{self.current_direction}:{vel}"
            self.network.send_udp_no_response(cmd)
            self.root.after(150, self._send_move_command)
    
    def servo_changed(self, servo_id, value):
        """Cambio en slider de servo"""
        angle = int(float(value))
        cmd = f"servo:{servo_id+1}:{angle}"
        self.network.send_udp_no_response(cmd)
    
    def change_mode(self):
        """Cambiar modo manual/automático"""
        if self.mode_auto.get():
            self.log_event("Modo AUTOMÁTICO activado")
        else:
            self.log_event("Modo MANUAL activado")
    
    def start_saving(self):
        """Iniciar guardado de datos"""
        response = self.network.send_udp_command("save:start")
        self.log_event(f"Guardado iniciado: {response}")
        self.saving.set(True)
    
    def stop_saving(self):
        """Detener guardado de datos"""
        response = self.network.send_udp_command("save:stop")
        self.log_event(f"Guardado detenido: {response}")
        self.saving.set(False)
    
    def save_point(self):
        """Guardar un punto único"""
        response = self.network.send_udp_command("save:point")
        self.log_event(f"Punto guardado: {response}")
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.moving = False
        self.network.send_udp_command("emergency:stop")
        self.log_event("⚠️ PARADA DE EMERGENCIA ACTIVADA", level="CRITICAL")
        messagebox.showwarning("Emergencia", "Parada de emergencia activada")
    
    def toggle_video(self):
        """Pausar/reanudar video"""
        paused = self.video_receiver.toggle_pause()
        if paused:
            self.log_event("Video pausado")
        else:
            self.log_event("Video reanudado")
    
    def reset_map(self):
        """Resetear mapa 2D"""
        self.map_2d.reset()
        self.log_event("Mapa reseteado")
    
    # ===== TECLADO =====
    
    def on_key_press(self, event):
        """Evento de tecla presionada"""
        key = event.keysym.lower()
        
        if key in ('w', 'up') and not self.moving:
            self.start_move('forward')
        elif key in ('s', 'down') and not self.moving:
            self.start_move('back')
        elif key in ('a', 'left') and not self.moving:
            self.start_move('left')
        elif key in ('d', 'right') and not self.moving:
            self.start_move('right')
    
    def on_key_release(self, event):
        """Evento de tecla liberada"""
        key = event.keysym.lower()
        
        if key in ('w', 's', 'a', 'd', 'up', 'down', 'left', 'right'):
            self.stop_move()
    
    # ===== ACTUALIZACIONES =====
    
    def update_video_frame(self, frame):
        """Actualizar frame de video en GUI"""
        try:
            self.current_frame = frame.copy()
            
            # Redimensionar
            frame_resized = cv2.resize(frame, (Config.VIDEO_WIDTH, Config.VIDEO_HEIGHT))
            
            # Convertir a formato Tkinter
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(img)
            
            # Actualizar label
            self.video_label.configure(image=img_tk)
            self.video_label.image = img_tk
            
            # Actualizar mapa 2D
            self.update_map()
            
        except Exception as e:
            print(f"[GUI] Error actualizando video: {e}")
    
    def update_map(self):
        """Actualizar mapa 2D"""
        try:
            map_img = self.map_2d.get_map_image()
            img = Image.fromarray(cv2.cvtColor(map_img, cv2.COLOR_BGR2RGB))
            img_tk = ImageTk.PhotoImage(img)
            self.map_label.configure(image=img_tk)
            self.map_label.image = img_tk
        except Exception as e:
            print(f"[GUI] Error actualizando mapa: {e}")
    
    def start_sensor_updates(self):
        """Iniciar actualización periódica de sensores"""
        self.update_sensors()
    
    def update_sensors(self):
        """Actualizar valores de sensores"""
        try:
            # Temperatura
            threading.Thread(target=self._update_temp, daemon=True).start()
            
            # Luz
            threading.Thread(target=self._update_light, daemon=True).start()
            
            # Distancia
            threading.Thread(target=self._update_dist, daemon=True).start()
            
            # Estado de conexión
            if self.network.connected:
                self.connection_status.set("🟢 Conectado")
            else:
                self.connection_status.set("🔴 Desconectado")
        
        except Exception as e:
            print(f"[GUI] Error actualizando sensores: {e}")
        
        # Repetir cada 1 segundo
        self.root.after(1000, self.update_sensors)
    
    def _update_temp(self):
        resp = self.network.send_udp_command("sensor:temp", timeout=1.0)
        if resp.startswith("TEMP:"):
            self.sensor_temp.set(f"{resp.split(':')[1]} °C")
    
    def _update_light(self):
        resp = self.network.send_udp_command("sensor:light", timeout=1.0)
        if resp.startswith("LDR:"):
            self.sensor_light.set(f"{resp.split(':')[1]} V")
    
    def _update_dist(self):
        resp = self.network.send_udp_command("sensor:dist", timeout=1.0)
        if resp.startswith("DIST:"):
            dist_str = resp.split(':')[1]
            self.sensor_dist.set(f"{dist_str} cm")
            
            # Actualizar mapa con distancia
            try:
                dist_val = int(''.join(filter(str.isdigit, dist_str)))
                if dist_val < 100:  # Solo si está cerca
                    self.map_2d.add_obstacle(dist_val)
            except:
                pass
    
    def log_event(self, message, level="INFO"):
        """Agregar evento al log"""
        timestamp = time.strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] [{level}] {message}\n"
        
        self.log_text.insert(tk.END, log_msg)
        self.log_text.see(tk.END)
        
        # Limitar líneas
        if int(self.log_text.index('end-1c').split('.')[0]) > 100:
            self.log_text.delete('1.0', '2.0')
    
    def on_closing(self):
        """Cerrar aplicación"""
        self.video_receiver.stop()
        self.network.send_udp_command("move:stop")
        self.root.destroy()

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    root = tk.Tk()
    app = RoverGUI(root)
    root.mainloop()
