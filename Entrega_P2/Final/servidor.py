#!/usr/bin/env python3
"""
ROVER DE EXPLORACIÓN - SERVIDOR RASPBERRY PI

Funciones:
- Servidor UDP para comandos (puerto 50000)
- Servidor TCP para streaming de video (puerto 50001)
- Puente serial con Arduino
- Detección de marcadores con OpenCV
- Gestión de CSV y logs
- Lógica autónoma

Autor: Sistema Rover
Versión: 2.0
"""

import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os
import json
import numpy as np
from datetime import datetime
from collections import deque
import yaml

# =========================
# CONFIGURACIÓN
# =========================
class Config:
    # Serial
    ARDUINO_PORT = '/dev/ttyUSB0'  # Ajustar según conexión
    BAUD_RATE = 115200
    SERIAL_TIMEOUT = 1.0
    SERIAL_RETRIES = 3
    
    # Red
    SERVER_IP = '0.0.0.0'
    UDP_PORT = 50000
    TCP_PORT = 50001
    BUFFER_SIZE = 4096
    
    # Cámaras
    CAMERA_FRONT = 0  # /dev/video0
    CAMERA_TOP = 2    # /dev/video2
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    JPEG_QUALITY = 70
    FPS_TARGET = 30
    
    # Archivos
    CSV_MARKERS = "data/marcadores.csv"
    CSV_LOGS = "data/logs.csv"
    CSV_TELEMETRY = "data/telemetria.csv"
    LOG_FILE = "data/rover.log"
    
    # Detección de marcadores
    MARKER_KEY = "Círculo"  # Configurar según staff
    MARKER_CONFIDENCE = 0.75
    
    # Estados
    MODE_MANUAL = "MANUAL"
    MODE_AUTONOMOUS = "AUTONOMOUS"
    MODE_SAMPLING = "SAMPLING"
    MODE_EMERGENCY = "EMERGENCY"

# =========================
# CLASE ARDUINO BRIDGE
# =========================
class ArduinoBridge:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()
        self.connect()
    
    def connect(self):
        """Conectar con Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=Config.SERIAL_TIMEOUT)
            time.sleep(2)  # Esperar reset de Arduino
            response = self.ser.readline().decode().strip()
            print(f"[ARDUINO] {response}")
            return True
        except Exception as e:
            print(f"[ERROR] No se pudo conectar con Arduino: {e}")
            self.ser = None
            return False
    
    def send_command(self, cmd, wait_response=True, retries=Config.SERIAL_RETRIES):
        """Enviar comando y esperar respuesta"""
        if not self.ser or not self.ser.is_open:
            return "ERR:ARDUINO_DISCONNECTED"
        
        with self.lock:
            for attempt in range(retries):
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write((cmd + '\n').encode())
                    
                    if wait_response:
                        deadline = time.time() + Config.SERIAL_TIMEOUT
                        while time.time() < deadline:
                            if self.ser.in_waiting > 0:
                                response = self.ser.readline().decode().strip()
                                if response:
                                    return response
                            time.sleep(0.01)
                        
                        if attempt < retries - 1:
                            print(f"[ARDUINO] Timeout en intento {attempt+1}, reintentando...")
                            continue
                        return "ERR:TIMEOUT"
                    else:
                        return "OK:SENT"
                        
                except Exception as e:
                    print(f"[ARDUINO] Error en intento {attempt+1}: {e}")
                    if attempt < retries - 1:
                        time.sleep(0.1)
                        continue
                    return f"ERR:{str(e)}"
        
        return "ERR:MAX_RETRIES"
    
    def close(self):
        if self.ser:
            self.ser.close()

# =========================
# CLASE LOGGER
# =========================
class Logger:
    def __init__(self):
        os.makedirs("data", exist_ok=True)
        self.csv_lock = threading.Lock()
        self._init_csvs()
    
    def _init_csvs(self):
        """Inicializar archivos CSV con encabezados"""
        # Marcadores
        if not os.path.exists(Config.CSV_MARKERS):
            with open(Config.CSV_MARKERS, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'mode', 'px_u', 'px_v', 'marker_type',
                               'light_V', 'temp_C', 'humidity_percent', 'action', 'notes'])
        
        # Logs
        if not os.path.exists(Config.CSV_LOGS):
            with open(Config.CSV_LOGS, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'component', 'level', 'message'])
        
        # Telemetría
        if not os.path.exists(Config.CSV_TELEMETRY):
            with open(Config.CSV_TELEMETRY, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'temp_C', 'light_V', 'distance_cm',
                               'humidity_ambient', 'imu_ax', 'imu_ay', 'imu_az',
                               'imu_gx', 'imu_gy', 'imu_gz'])
    
    def log_marker(self, mode, px_u, px_v, marker_type, light, temp, humidity, action, notes=""):
        """Registrar detección de marcador"""
        with self.csv_lock:
            with open(Config.CSV_MARKERS, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    mode, px_u, px_v, marker_type, light, temp, humidity, action, notes
                ])
    
    def log_event(self, component, level, message):
        """Registrar evento en log"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{component}] {level}: {message}")
        
        with self.csv_lock:
            with open(Config.CSV_LOGS, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, component, level, message])
    
    def log_telemetry(self, sensors):
        """Registrar telemetría"""
        with self.csv_lock:
            with open(Config.CSV_TELEMETRY, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    sensors.get('temp', 0),
                    sensors.get('light', 0),
                    sensors.get('dist', 0),
                    sensors.get('hum', 0),
                    sensors.get('imu_ax', 0),
                    sensors.get('imu_ay', 0),
                    sensors.get('imu_az', 0),
                    sensors.get('imu_gx', 0),
                    sensors.get('imu_gy', 0),
                    sensors.get('imu_gz', 0)
                ])

# =========================
# CLASE DETECTOR DE MARCADORES
# =========================
class MarkerDetector:
    def __init__(self):
        # Templates de marcadores (se pueden mejorar con imágenes reales)
        self.markers = {
            'Cruz': self._create_cross_template(),
            'T': self._create_t_template(),
            'Círculo': self._create_circle_template(),
            'Cuadrado': self._create_square_template(),
            'Triángulo': self._create_triangle_template()
        }
    
    def _create_cross_template(self):
        template = np.zeros((50, 50), dtype=np.uint8)
        cv2.line(template, (25, 10), (25, 40), 255, 5)
        cv2.line(template, (10, 25), (40, 25), 255, 5)
        return template
    
    def _create_t_template(self):
        template = np.zeros((50, 50), dtype=np.uint8)
        cv2.line(template, (10, 15), (40, 15), 255, 5)
        cv2.line(template, (25, 15), (25, 40), 255, 5)
        return template
    
    def _create_circle_template(self):
        template = np.zeros((50, 50), dtype=np.uint8)
        cv2.circle(template, (25, 25), 15, 255, 3)
        return template
    
    def _create_square_template(self):
        template = np.zeros((50, 50), dtype=np.uint8)
        cv2.rectangle(template, (10, 10), (40, 40), 255, 3)
        return template
    
    def _create_triangle_template(self):
        template = np.zeros((50, 50), dtype=np.uint8)
        pts = np.array([[25, 10], [10, 40], [40, 40]], np.int32)
        cv2.polylines(template, [pts], True, 255, 3)
        return template
    
    def detect(self, frame):
        """
        Detectar marcadores en el frame
        Retorna: (marker_type, px_u, px_v, confidence) o None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        best_match = None
        best_confidence = 0
        
        for marker_name, template in self.markers.items():
            result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            
            if max_val > best_confidence and max_val > Config.MARKER_CONFIDENCE:
                best_confidence = max_val
                h, w = template.shape
                center_x = max_loc[0] + w // 2
                center_y = max_loc[1] + h // 2
                best_match = (marker_name, center_x, center_y, max_val)
        
        return best_match

# =========================
# CLASE ROVER CONTROLLER
# =========================
class RoverController:
    def __init__(self):
        self.arduino = ArduinoBridge(Config.ARDUINO_PORT, Config.BAUD_RATE)
        self.logger = Logger()
        self.detector = MarkerDetector()
        
        self.mode = Config.MODE_MANUAL
        self.saving = False
        self.running = True
        
        self.sensors_cache = {}
        self.marker_count = 0
    
    def translate_move_command(self, direction, velocity):
        """Traducir comando de movimiento a comandos Arduino"""
        vel = int(velocity)
        
        if direction == "forward":
            self.arduino.send_command(f"MOTOR LEFT FORWARD {vel}", wait_response=False)
            self.arduino.send_command(f"MOTOR RIGHT FORWARD {vel}", wait_response=False)
            self.logger.log_event("ROVER", "INFO", f"avancé a velocidad {vel}")
        
        elif direction == "back":
            self.arduino.send_command(f"MOTOR LEFT BACK {vel}", wait_response=False)
            self.arduino.send_command(f"MOTOR RIGHT BACK {vel}", wait_response=False)
            self.logger.log_event("ROVER", "INFO", f"retrocedí a velocidad {vel}")
        
        elif direction == "left":
            self.arduino.send_command(f"MOTOR LEFT BACK {vel}", wait_response=False)
            self.arduino.send_command(f"MOTOR RIGHT FORWARD {vel}", wait_response=False)
            self.logger.log_event("ROVER", "INFO", "giré a la izquierda")
        
        elif direction == "right":
            self.arduino.send_command(f"MOTOR LEFT FORWARD {vel}", wait_response=False)
            self.arduino.send_command(f"MOTOR RIGHT BACK {vel}", wait_response=False)
            self.logger.log_event("ROVER", "INFO", "giré a la derecha")
        
        elif direction == "stop":
            self.arduino.send_command("MOTORS STOP", wait_response=False)
            self.logger.log_event("ROVER", "INFO", "detuve motores")
        
        return "OK"
    
    def read_all_sensors(self):
        """Leer todos los sensores y actualizar caché"""
        sensors = {}
        
        # Temperatura
        resp = self.arduino.send_command("SENSOR TEMP")
        if resp.startswith("TEMP:"):
            sensors['temp'] = float(resp.split(':')[1])
        
        # Luz
        resp = self.arduino.send_command("SENSOR LDR")
        if resp.startswith("LDR:"):
            sensors['light'] = float(resp.split(':')[1])
        
        # Distancia
        resp = self.arduino.send_command("SENSOR DIST")
        if resp.startswith("DIST:"):
            sensors['dist'] = resp.split(':')[1]
        
        # IMU
        resp = self.arduino.send_command("SENSOR IMU")
        if resp.startswith("IMU:"):
            values = resp.split(':')[1].split(',')
            sensors['imu_ax'] = float(values[0])
            sensors['imu_ay'] = float(values[1])
            sensors['imu_az'] = float(values[2])
            sensors['imu_gx'] = float(values[3])
            sensors['imu_gy'] = float(values[4])
            sensors['imu_gz'] = float(values[5])
        
        self.sensors_cache = sensors
        return sensors
    
    def execute_autonomous_sampling(self, marker_type, px_u, px_v):
        """Ejecutar secuencia autónoma de muestreo"""
        self.logger.log_event("ROVER", "INFO", f"marcador clave detectado: {marker_type}")
        
        # Detener motores
        self.arduino.send_command("MOTORS STOP")
        
        # Leer sensores ambientales
        sensors = self.read_all_sensors()
        
        # Activar brazo
        response = self.arduino.send_command("ACTUATE BRACO START", wait_response=True)
        
        if "OK" in response:
            self.logger.log_event("ROVER", "INFO", "brazo activado correctamente")
            
            # Leer humedad capacitiva
            time.sleep(1)
            resp = self.arduino.send_command("SENSOR HUM_CAP")
            humidity = 0
            if resp.startswith("HUM:"):
                humidity = float(resp.split(':')[1])
            
            self.logger.log_event("ROVER", "INFO", f"tomé muestra de humedad: {humidity}%")
            
            # Guardar en CSV
            self.logger.log_marker(
                Config.MODE_AUTONOMOUS,
                px_u, px_v,
                marker_type,
                sensors.get('light', 0),
                sensors.get('temp', 0),
                humidity,
                "SAMPLE",
                f"Marcador clave: {marker_type}"
            )
            
            return True
        else:
            self.logger.log_event("ROVER", "ERROR", "fallo en activación de brazo")
            return False
    
    def shutdown(self):
        """Apagar sistema de forma segura"""
        self.running = False
        self.arduino.send_command("MOTORS STOP")
        self.arduino.close()

# =========================
# SERVIDOR UDP
# =========================
def udp_server(controller):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((Config.SERVER_IP, Config.UDP_PORT))
    print(f"[UDP] Servidor escuchando en {Config.SERVER_IP}:{Config.UDP_PORT}")
    
    while controller.running:
        try:
            data, addr = sock.recvfrom(Config.BUFFER_SIZE)
            cmd = data.decode('utf-8').strip()
            
            # Procesar comando
            response = process_udp_command(controller, cmd)
            sock.sendto(response.encode('utf-8'), addr)
            
        except Exception as e:
            print(f"[UDP] Error: {e}")

def process_udp_command(controller, cmd):
    """Procesar comandos UDP"""
    parts = cmd.split(':')
    command = parts[0].lower()
    
    try:
        # Movimiento
        if command == "move":
            direction = parts[1]
            velocity = parts[2] if len(parts) > 2 else "80"
            return controller.translate_move_command(direction, velocity)
        
        # Servos
        elif command == "servo":
            servo_id = parts[1]
            angle = parts[2]
            resp = controller.arduino.send_command(f"SERVO {servo_id} {angle}")
            return resp
        
        # Sensores
        elif command == "sensor":
            sensor_type = parts[1]
            if sensor_type == "temp":
                resp = controller.arduino.send_command("SENSOR TEMP")
            elif sensor_type == "light":
                resp = controller.arduino.send_command("SENSOR LDR")
            elif sensor_type == "dist":
                resp = controller.arduino.send_command("SENSOR DIST")
            elif sensor_type == "imu":
                resp = controller.arduino.send_command("SENSOR IMU")
            elif sensor_type == "hum":
                resp = controller.arduino.send_command("SENSOR HUM_CAP")
            else:
                resp = "ERR:UNKNOWN_SENSOR"
            return resp
        
        # Guardado
        elif command == "save":
            action = parts[1]
            if action == "start":
                controller.saving = True
                return "SAVE:STARTED"
            elif action == "stop":
                controller.saving = False
                return "SAVE:STOPPED"
            elif action == "point":
                sensors = controller.read_all_sensors()
                controller.logger.log_telemetry(sensors)
                return "SAVE:POINT_OK"
        
        # Emergencia
        elif command == "emergency":
            controller.arduino.send_command("EMERGENCY STOP")
            controller.logger.log_event("ROVER", "CRITICAL", "PARADA DE EMERGENCIA")
            return "EMERGENCY:OK"
        
        # Estado
        elif command == "status":
            return f"MODE:{controller.mode},MARKERS:{controller.marker_count},SAVING:{controller.saving}"
        
        else:
            return "ERR:UNKNOWN_COMMAND"
    
    except Exception as e:
        return f"ERR:{str(e)}"

# =========================
# SERVIDOR TCP VIDEO
# =========================
def tcp_video_server(controller):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((Config.SERVER_IP, Config.TCP_PORT))
    server.listen(2)
    print(f"[TCP] Servidor de video en {Config.SERVER_IP}:{Config.TCP_PORT}")
    
    while controller.running:
        try:
            client, addr = server.accept()
            print(f"[TCP] Cliente conectado: {addr}")
            threading.Thread(target=stream_video, args=(client, controller), daemon=True).start()
        except Exception as e:
            print(f"[TCP] Error: {e}")

def stream_video(client_socket, controller):
    cap_front = cv2.VideoCapture(Config.CAMERA_FRONT)
    cap_front.set(cv2.CAP_PROP_FRAME_WIDTH, Config.FRAME_WIDTH)
    cap_front.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.FRAME_HEIGHT)
    
    try:
        while controller.running:
            ret, frame = cap_front.read()
            if not ret:
                continue
            
            # Detección de marcadores si está en modo autónomo
            if controller.mode == Config.MODE_AUTONOMOUS:
                detection = controller.detector.detect(frame)
                if detection:
                    marker_type, px_u, px_v, confidence = detection
                    controller.marker_count += 1
                    
                    # Dibujar detección
                    cv2.circle(frame, (px_u, px_v), 20, (0, 255, 0), 3)
                    cv2.putText(frame, marker_type, (px_u + 25, px_v), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Si es marcador clave, ejecutar secuencia
                    if marker_type == Config.MARKER_KEY:
                        controller.execute_autonomous_sampling(marker_type, px_u, px_v)
            
            # Codificar y enviar
            _, jpeg = cv2.imencode('.jpg', frame, 
                                  [int(cv2.IMWRITE_JPEG_QUALITY), Config.JPEG_QUALITY])
            data = jpeg.tobytes()
            
            client_socket.sendall(struct.pack("Q", len(data)))
            client_socket.sendall(data)
            
            time.sleep(1.0 / Config.FPS_TARGET)
    
    except Exception as e:
        print(f"[TCP] Cliente desconectado: {e}")
    finally:
        cap_front.release()
        client_socket.close()

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    print("=" * 60)
    print("ROVER DE EXPLORACIÓN - SERVIDOR RASPBERRY PI v2.0")
    print("=" * 60)
    
    controller = RoverController()
    
    # Iniciar servidores
    threading.Thread(target=udp_server, args=(controller,), daemon=True).start()
    threading.Thread(target=tcp_video_server, args=(controller,), daemon=True).start()
    
    print("\n[SISTEMA] Todos los servicios iniciados")
    print("[SISTEMA] Presiona Ctrl+C para salir\n")
    
    try:
        while controller.running:
            # Guardado continuo si está activado
            if controller.saving:
                sensors = controller.read_all_sensors()
                controller.logger.log_telemetry(sensors)
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n[SISTEMA] Apagando...")
        controller.shutdown()
        print("[SISTEMA] Sistema apagado correctamente")
