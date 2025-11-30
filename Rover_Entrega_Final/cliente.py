"""
CLIENTE GUI OPTIMIZADO
Sin delays, UI responsive
"""

import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import time

# ===== CONFIG =====
SERVER_IP = "172.32.200.137"
UDP_PORT = 50000
TCP_PORT = 50001

# ===== NETWORK =====
class Network:
    def __init__(self):
        self.ip = SERVER_IP
        self.udp_port = UDP_PORT
        self.tcp_port = TCP_PORT
    
    def send_udp(self, cmd):
        """Envío UDP sin esperar respuesta"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.1)
            sock.sendto(cmd.encode(), (self.ip, self.udp_port))
            sock.close()
        except:
            pass
    
    def query_udp(self, cmd, timeout=1.0):
        """Envío UDP con respuesta"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(timeout)
            sock.sendto(cmd.encode(), (self.ip, self.udp_port))
            data, _ = sock.recvfrom(1024)
            sock.close()
            return data.decode()
        except:
            return "TIMEOUT"

# ===== VIDEO RECEIVER =====
class VideoReceiver:
    def __init__(self, callback):
        self.callback = callback
        self.running = False
        self.thread = None
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
    
    def _loop(self):
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((SERVER_IP, TCP_PORT))
                sock.settimeout(None)
                print("[VIDEO] Conectado")
                
                data = b""
                payload_size = struct.calcsize("Q")
                
                while self.running:
                    # Leer tamaño
                    while len(data) < payload_size:
                        packet = sock.recv(4096)
                        if not packet:
                            raise ConnectionError()
                        data += packet
                    
                    frame_size = struct.unpack("Q", data[:payload_size])[0]
                    data = data[payload_size:]
                    
                    # Leer frame
                    while len(data) < frame_size:
                        packet = sock.recv(4096)
                        if not packet:
                            raise ConnectionError()
                        data += packet
                    
                    frame_data = data[:frame_size]
                    data = data[frame_size:]
                    
                    # Decodificar
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        self.callback(frame)
                
            except Exception as e:
                print(f"[VIDEO] Error: {e}")
                time.sleep(2)

# ===== GUI =====
class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🚀 Rover Control Optimizado")
        self.root.geometry("1200x700")
        self.root.configure(bg='#2b2b2b')
        
        self.network = Network()
        self.video = VideoReceiver(self.update_frame)
        
        # Variables
        self.mode_auto = tk.BooleanVar(value=False)
        self.speed = tk.IntVar(value=80)
        self.servos = [tk.IntVar(value=90) for _ in range(4)]
        
        self.sensor_temp = tk.StringVar(value="--")
        self.sensor_light = tk.StringVar(value="--")
        self.sensor_dist = tk.StringVar(value="--")
        
        # Control de movimiento
        self.moving = False
        self.current_dir = None
        
        self.build_ui()
        self.video.start()
        
        # Actualizar sensores cada 1s
        self.update_sensors()
        
        # Bind teclado
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
    
    def build_ui(self):
        # Frame principal
        main = tk.Frame(self.root, bg='#2b2b2b')
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # ===== IZQUIERDA: VIDEO =====
        left = tk.Frame(main, bg='#2b2b2b')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,10))
        
        video_frame = tk.LabelFrame(left, text="📹 Cámara", bg='#2b2b2b', fg='white', font=('Arial',12,'bold'))
        video_frame.pack(fill=tk.BOTH, expand=True)
        
        self.video_label = tk.Label(video_frame, bg='black')
        self.video_label.pack(padx=5, pady=5)
        
        # ===== DERECHA: CONTROLES =====
        right = tk.Frame(main, bg='#2b2b2b', width=400)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)
        right.pack_propagate(False)
        
        # Modo
        mode_frame = tk.LabelFrame(right, text="⚙️ Modo", bg='#2b2b2b', fg='white', font=('Arial',11,'bold'))
        mode_frame.pack(fill=tk.X, pady=(0,10))
        
        tk.Radiobutton(mode_frame, text="Manual", variable=self.mode_auto, value=False,
                      bg='#2b2b2b', fg='white', selectcolor='#1e1e1e',
                      command=self.change_mode).pack(anchor=tk.W, padx=10, pady=2)
        tk.Radiobutton(mode_frame, text="Automático", variable=self.mode_auto, value=True,
                      bg='#2b2b2b', fg='white', selectcolor='#1e1e1e',
                      command=self.change_mode).pack(anchor=tk.W, padx=10, pady=2)
        
        # Velocidad
        speed_frame = tk.LabelFrame(right, text="🎚️ Velocidad", bg='#2b2b2b', fg='white', font=('Arial',11,'bold'))
        speed_frame.pack(fill=tk.X, pady=(0,10))
        
        tk.Scale(speed_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                variable=self.speed, bg='#2b2b2b', fg='white',
                troughcolor='#1e1e1e', length=350).pack(padx=10, pady=5)
        
        # Movimiento
        move_frame = tk.LabelFrame(right, text="🎮 Movimiento", bg='#2b2b2b', fg='white', font=('Arial',11,'bold'))
        move_frame.pack(fill=tk.X, pady=(0,10))
        
        btn = tk.Frame(move_frame, bg='#2b2b2b')
        btn.pack(pady=10)
        
        style = {'width':10, 'height':2, 'font':('Arial',10,'bold')}
        
        self.btn_fwd = tk.Button(btn, text="↑", bg='#4a9eff', **style)
        self.btn_fwd.grid(row=0, column=1, padx=5, pady=5)
        self.btn_fwd.bind('<ButtonPress-1>', lambda e: self.start_move('forward'))
        self.btn_fwd.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        self.btn_left = tk.Button(btn, text="←", bg='#4a9eff', **style)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_left.bind('<ButtonPress-1>', lambda e: self.start_move('left'))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        tk.Button(btn, text="■", bg='#ff4a4a', command=self.emergency, **style).grid(row=1, column=1, padx=5, pady=5)
        
        self.btn_right = tk.Button(btn, text="→", bg='#4a9eff', **style)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_right.bind('<ButtonPress-1>', lambda e: self.start_move('right'))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        self.btn_back = tk.Button(btn, text="↓", bg='#4a9eff', **style)
        self.btn_back.grid(row=2, column=1, padx=5, pady=5)
        self.btn_back.bind('<ButtonPress-1>', lambda e: self.start_move('back'))
        self.btn_back.bind('<ButtonRelease-1>', lambda e: self.stop_move())
        
        tk.Label(move_frame, text="WASD o Flechas", bg='#2b2b2b', fg='#aaa', font=('Arial',9,'italic')).pack()
        
        # Servos
        servo_frame = tk.LabelFrame(right, text="🦾 Servos", bg='#2b2b2b', fg='white', font=('Arial',11,'bold'))
        servo_frame.pack(fill=tk.X, pady=(0,10))
        
        names = ["Cámara", "Brazo 1", "Brazo 2", "Sensor"]
        for i, name in enumerate(names):
            f = tk.Frame(servo_frame, bg='#2b2b2b')
            f.pack(fill=tk.X, padx=10, pady=3)
            tk.Label(f, text=name, bg='#2b2b2b', fg='white', width=10, anchor=tk.W).pack(side=tk.LEFT)
            tk.Scale(f, from_=0, to=180, orient=tk.HORIZONTAL, variable=self.servos[i],
                    bg='#2b2b2b', fg='white', troughcolor='#1e1e1e', length=200,
                    command=lambda v, idx=i: self.servo_change(idx, v)).pack(side=tk.LEFT)
        
        # Telemetría
        telem_frame = tk.LabelFrame(right, text="📊 Sensores", bg='#2b2b2b', fg='white', font=('Arial',11,'bold'))
        telem_frame.pack(fill=tk.X, pady=(0,10))
        
        sensors = [
            ("🌡️ Temp:", self.sensor_temp, '#ff6b6b'),
            ("💡 Luz:", self.sensor_light, '#ffd93d'),
            ("📏 Dist:", self.sensor_dist, '#6bcf7f')
        ]
        
        for label, var, color in sensors:
            f = tk.Frame(telem_frame, bg='#2b2b2b')
            f.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(f, text=label, bg='#2b2b2b', fg='white', width=10, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(f, textvariable=var, bg='#2b2b2b', fg=color, font=('Arial',10,'bold'), width=15, anchor=tk.E).pack(side=tk.RIGHT)
        
        # Emergencia
        tk.Button(right, text="🚨 EMERGENCIA 🚨", command=self.emergency,
                 bg='#d32f2f', fg='white', font=('Arial',14,'bold'), height=2).pack(fill=tk.X, padx=10, pady=10)
    
    # ===== MÉTODOS =====
    
    def start_move(self, direction):
        if not self.moving:
            self.moving = True
            self.current_dir = direction
            self._send_move()
    
    def stop_move(self):
        self.moving = False
        self.current_dir = None
        self.network.send_udp("move:stop")
    
    def _send_move(self):
        if self.moving and self.current_dir:
            vel = self.speed.get()
            self.network.send_udp(f"move:{self.current_dir}:{vel}")
            self.root.after(100, self._send_move)
    
    def servo_change(self, idx, val):
        angle = int(float(val))
        self.network.send_udp(f"servo:{idx+1}:{angle}")
    
    def change_mode(self):
        mode = "AUTONOMOUS" if self.mode_auto.get() else "MANUAL"
        self.network.send_udp(f"mode:{mode}")
    
    def emergency(self):
        self.moving = False
        self.network.send_udp("emergency:stop")
        print("[EMERGENCIA] Activada")
    
    def key_press(self, event):
        key = event.keysym.lower()
        if key in ('w', 'up') and not self.moving:
            self.start_move('forward')
        elif key in ('s', 'down') and not self.moving:
            self.start_move('back')
        elif key in ('a', 'left') and not self.moving:
            self.start_move('left')
        elif key in ('d', 'right') and not self.moving:
            self.start_move('right')
    
    def key_release(self, event):
        key = event.keysym.lower()
        if key in ('w', 's', 'a', 'd', 'up', 'down', 'left', 'right'):
            self.stop_move()
    
    def update_frame(self, frame):
        try:
            frame = cv2.resize(frame, (640, 480))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_tk = ImageTk.PhotoImage(img)
            self.video_label.configure(image=img_tk)
            self.video_label.image = img_tk
        except:
            pass
    
    def update_sensors(self):
        def read():
            resp = self.network.query_udp("sensor:temp", 0.5)
            if resp.startswith("TEMP:"):
                self.sensor_temp.set(f"{resp.split(':')[1]} °C")
            
            resp = self.network.query_udp("sensor:light", 0.5)
            if resp.startswith("LDR:"):
                self.sensor_light.set(f"{resp.split(':')[1]} V")
            
            resp = self.network.query_udp("sensor:dist", 0.5)
            if resp.startswith("DIST:"):
                self.sensor_dist.set(f"{resp.split(':')[1]} cm")
        
        threading.Thread(target=read, daemon=True).start()
        self.root.after(1000, self.update_sensors)
    
    def close(self):
        self.video.stop()
        self.network.send_udp("move:stop")
        self.root.destroy()

# ===== MAIN =====
if __name__ == "__main__":
    root = tk.Tk()
    app = RoverGUI(root)
    root.mainloop()