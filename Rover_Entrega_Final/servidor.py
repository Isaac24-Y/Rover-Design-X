#!/usr/bin/env python3
"""
ROVER DE EXPLORACIÓN - SERVIDOR RASPBERRY PI MEJORADO
Con detección de marcadores usando Hu Moments

Versión: 2.1
"""

import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os
import numpy as np
from datetime import datetime
import yaml

# =========================
# CARGAR CONFIGURACIÓN
# =========================
with open('config.yaml', 'r') as f:
    CONFIG = yaml.safe_load(f)

# =========================
# CLASE ARDUINO BRIDGE
# =========================
class ArduinoBridge:
    def __init__(self):
        self.port = CONFIG['arduino']['port']
        self.baud = CONFIG['arduino']['baud_rate']
        self.timeout = CONFIG['arduino']['timeout']
        self.retries = CONFIG['arduino']['retries']
        self.ser = None
        self.lock = threading.Lock()
        self.connect()
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)
            response = self.ser.readline().decode().strip()
            print(f"[ARDUINO] {response}")
            return True
        except Exception as e:
            print(f"[ERROR] Arduino: {e}")
            self.ser = None
            return False
    
    def send_command(self, cmd, wait_response=True):
        if not self.ser or not self.ser.is_open:
            return "ERR:ARDUINO_DISCONNECTED"
        
        with self.lock:
            for attempt in range(self.retries):
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write((cmd + '\n').encode())
                    
                    if wait_response:
                        deadline = time.time() + self.timeout
                        while time.time() < deadline:
                            if self.ser.in_waiting > 0:
                                response = self.ser.readline().decode().strip()
                                if response:
                                    return response
                            time.sleep(0.01)
                        
                        if attempt < self.retries - 1:
                            continue
                        return "ERR:TIMEOUT"
                    else:
                        return "OK:SENT"
                        
                except Exception as e:
                    if attempt < self.retries - 1:
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
        os.makedirs(CONFIG['data']['base_path'], exist_ok=True)
        self.csv_lock = threading.Lock()
        self._init_csvs()
    
    def _init_csvs(self):
        # Marcadores
        if not os.path.exists(CONFIG['data']['csv_markers']):
            with open(CONFIG['data']['csv_markers'], 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'mode', 'px_u', 'px_v', 'marker_type',
                               'light_V', 'temp_C', 'humidity_percent', 'action', 'notes'])
        
        # Logs
        if not os.path.exists(CONFIG['data']['csv_logs']):
            with open(CONFIG['data']['csv_logs'], 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'component', 'level', 'message'])
        
        # Telemetría
        if not os.path.exists(CONFIG['data']['csv_telemetry']):
            with open(CONFIG['data']['csv_telemetry'], 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'temp_C', 'light_V', 'distance_cm',
                               'humidity_ambient', 'imu_ax', 'imu_ay', 'imu_az',
                               'imu_gx', 'imu_gy', 'imu_gz'])
    
    def log_marker(self, mode, px_u, px_v, marker_type, light, temp, humidity, action, notes=""):
        with self.csv_lock:
            with open(CONFIG['data']['csv_markers'], 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    mode, px_u, px_v, marker_type, light, temp, humidity, action, notes
                ])
    
    def log_event(self, component, level, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{component}] {level}: {message}")
        
        with self.csv_lock:
            with open(CONFIG['data']['csv_logs'], 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, component, level, message])
    
    def log_telemetry(self, sensors):
        with self.csv_lock:
            with open(CONFIG['data']['csv_telemetry'], 'a', newline='') as f:
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
# DETECTOR HU MOMENTS
# =========================
class HuMomentsDetector:
    def __init__(self):
        self.rangos = CONFIG['marker_detection']['hu_ranges']
        self.colors = CONFIG['marker_detection']['colors']
        
        # Convertir a numpy arrays
        self.lower_orange = np.array(self.colors['orange']['lower'])
        self.upper_orange = np.array(self.colors['orange']['upper'])
        self.lower_green = np.array(self.colors['green']['lower'])
        self.upper_green = np.array(self.colors['green']['upper'])
        self.lower_yellow = np.array(self.colors['yellow']['lower'])
        self.upper_yellow = np.array(self.colors['yellow']['upper'])
    
    def coincide_rango(self, valor, rango):
        return rango[0] <= valor <= rango[1]
    
    def clasificar_por_hu(self, hu_log):
        """Clasificar figura según Hu Moments"""
        for figura, reglas in self.rangos.items():
            ok = True
            for idx in range(1, 8):
                rango = reglas[idx]
                if not self.coincide_rango(hu_log[idx-1], rango):
                    ok = False
                    break
            if ok:
                return figura
        return None
    
    def detect(self, frame):
        """
        Detectar marcadores en el frame usando Hu Moments
        Retorna: (marker_type, px_u, px_v, confidence) o None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Máscaras por color
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Combinar colores
        mask = cv2.bitwise_or(mask_orange, mask_green)
        mask = cv2.bitwise_or(mask, mask_yellow)
        
        # Suavizar máscara
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Encontrar contornos
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not cnts:
            return None
        
        # Tomar el contorno más grande
        c = max(cnts, key=cv2.contourArea)
        
        # Calcular área mínima
        area = cv2.contourArea(c)
        if area < 500:  # Filtrar ruido
            return None
        
        # Calcular Hu Moments
        m = cv2.moments(c)
        if m["m00"] == 0:
            return None
        
        hu = cv2.HuMoments(m).flatten()
        hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-30)
        
        # Clasificar
        figura = self.clasificar_por_hu(hu_log)
        
        if figura is None:
            return None
        
        # Mapear nombres
        name_map = {
            'X': 'Cruz',
            'T': 'T',
            'Circulo': 'Círculo',
            'Triangulo': 'Triángulo',
            'Cuadrado': 'Cuadrado'
        }
        
        marker_name = name_map.get(figura, figura)
        
        # Calcular centro
        x, y, w, h = cv2.boundingRect(c)
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Confianza basada en área (simplificado)
        confidence = min(area / 5000.0, 1.0)
        
        return (marker_name, center_x, center_y, confidence)

# =========================
# CLASE ROVER CONTROLLER
# =========================
class RoverController:
    def __init__(self):
        self.arduino = ArduinoBridge()
        self.logger = Logger()
        self.detector = HuMomentsDetector()
        
        self.mode = "MANUAL"
        self.saving = False
        self.running = True
        
        self.sensors_cache = {}
        self.marker_count = 0
        
        self.key_marker = CONFIG['marker_detection']['key_marker']
    
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
            dist_str = resp.split(':')[1]
            sensors['dist'] = dist_str if dist_str != "OUT_OF_RANGE" else "0"
        
        # IMU
        resp = self.arduino.send_command("SENSOR IMU")
        if resp.startswith("IMU:"):
            values = resp.split(':')[1].split(',')
            if len(values) == 6:
                sensors['imu_ax'] = float(values[0])
                sensors['imu_ay'] = float(values[1])
                sensors['imu_az'] = float(values[2])
                sensors['imu_gx'] = float(values[3])
                sensors['imu_gy'] = float(values[4])
                sensors['imu_gz'] = float(values[5])
        
        # Humedad capacitiva
        resp = self.arduino.send_command("SENSOR HUM_CAP")
        if resp.startswith("HUM:"):
            sensors['hum'] = float(resp.split(':')[1])
        
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
        self.logger.log_event("ROVER", "INFO", "activando brazo robótico")
        response = self.arduino.send_command("ACTUATE BRACO START", wait_response=True)
        
        if "OK" in response:
            self.logger.log_event("ROVER", "INFO", "brazo activado correctamente")
            
            # Esperar estabilización
            time.sleep(1)
            
            # Leer humedad capacitiva
            resp = self.arduino.send_command("SENSOR HUM_CAP")
            humidity = 0
            if resp.startswith("HUM:"):
                humidity = float(resp.split(':')[1])
            
            self.logger.log_event("ROVER", "INFO", f"tomé muestra de humedad: {humidity}%")
            
            # Guardar en CSV
            self.logger.log_marker(
                "AUTONOMOUS",
                px_u, px_v,
                marker_type,
                sensors.get('light', 0),
                sensors.get('temp', 0),
                humidity,
                "SAMPLE",
                f"Marcador clave: {marker_type}"
            )
            
            self.marker_count += 1
            
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
    sock.bind((CONFIG['server']['ip'], CONFIG['server']['udp_port']))
    print(f"[UDP] Servidor en {CONFIG['server']['ip']}:{CONFIG['server']['udp_port']}")
    
    while controller.running:
        try:
            data, addr = sock.recvfrom(CONFIG['server']['buffer_size'])
            cmd = data.decode('utf-8').strip()
            
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
        
        # Acciones
        elif command == "action":
            action_type = parts[1]
            if action_type == "sample":
                # Activar brazo manualmente
                resp = controller.arduino.send_command("ACTUATE BRACO START")
                return resp
        
        # Emergencia
        elif command == "emergency":
            action = parts[1] if len(parts) > 1 else "stop"
            if action == "stop":
                controller.arduino.send_command("EMERGENCY STOP")
                controller.logger.log_event("ROVER", "CRITICAL", "PARADA DE EMERGENCIA")
                return "EMERGENCY:OK"
            elif action == "clear":
                controller.arduino.send_command("EMERGENCY CLEAR")
                controller.logger.log_event("ROVER", "INFO", "Emergencia limpiada")
                return "EMERGENCY:CLEARED"
        
        # Estado
        elif command == "status":
            return f"MODE:{controller.mode},MARKERS:{controller.marker_count},SAVING:{controller.saving}"
        
        # Modo
        elif command == "mode":
            new_mode = parts[1].upper()
            if new_mode in ["MANUAL", "AUTONOMOUS"]:
                controller.mode = new_mode
                return f"MODE:CHANGED:{new_mode}"
            else:
                return "ERR:INVALID_MODE"
        
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
    server.bind((CONFIG['server']['ip'], CONFIG['server']['tcp_port']))
    server.listen(2)
    print(f"[TCP] Servidor de video en {CONFIG['server']['ip']}:{CONFIG['server']['tcp_port']}")
    
    while controller.running:
        try:
            client, addr = server.accept()
            print(f"[TCP] Cliente conectado: {addr}")
            threading.Thread(target=stream_video, args=(client, controller), daemon=True).start()
        except Exception as e:
            print(f"[TCP] Error: {e}")

def stream_video(client_socket, controller):
    cam_config = CONFIG['cameras']['front']
    cap = cv2.VideoCapture(cam_config['device'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_config['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_config['height'])
    cap.set(cv2.CAP_PROP_FPS, cam_config['fps'])
    
    fps_target = cam_config['fps']
    frame_time = 1.0 / fps_target
    
    try:
        while controller.running:
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Detección de marcadores si está en modo autónomo
            if controller.mode == "AUTONOMOUS":
                detection = controller.detector.detect(frame)
                if detection:
                    marker_type, px_u, px_v, confidence = detection
                    
                    # Dibujar detección
                    cv2.circle(frame, (px_u, px_v), 20, (0, 255, 0), 3)
                    cv2.putText(frame, marker_type, (px_u + 25, px_v), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"{confidence:.2f}", (px_u + 25, px_v + 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Si es marcador clave, ejecutar secuencia
                    if marker_type == controller.key_marker and confidence > CONFIG['marker_detection']['confidence_threshold']:
                        controller.execute_autonomous_sampling(marker_type, px_u, px_v)
                        time.sleep(5)  # Pausa tras muestreo
            
            # Codificar y enviar
            _, jpeg = cv2.imencode('.jpg', frame, 
                                  [int(cv2.IMWRITE_JPEG_QUALITY), CONFIG['cameras']['jpeg_quality']])
            data = jpeg.tobytes()
            
            try:
                client_socket.sendall(struct.pack("Q", len(data)))
                client_socket.sendall(data)
            except:
                break
            
            # Control de FPS
            elapsed = time.time() - start_time
            sleep_time = frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        print(f"[TCP] Cliente desconectado: {e}")
    finally:
        cap.release()
        client_socket.close()

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    print("=" * 60)
    print("ROVER DE EXPLORACIÓN - SERVIDOR RASPBERRY PI v2.1")
    print("=" * 60)
    
    controller = RoverController()
    
    # Iniciar servidores
    threading.Thread(target=udp_server, args=(controller,), daemon=True).start()
    threading.Thread(target=tcp_video_server, args=(controller,), daemon=True).start()
    
    print("\n[SISTEMA] Todos los servicios iniciados")
    print(f"[SISTEMA] Marcador clave configurado: {controller.key_marker}")
    print("[SISTEMA] Presiona Ctrl+C para salir\n")
    
    try:
        while controller.running:
            # Guardado continuo si está activado
            if controller.saving:
                sensors = controller.read_all_sensors()
                controller.logger.log_telemetry(sensors)
            time.sleep(CONFIG['control']['sensor_read_interval'])
    
    except KeyboardInterrupt:
        print("\n[SISTEMA] Apagando...")
        controller.shutdown()
        print("[SISTEMA] Sistema apagado correctamente")