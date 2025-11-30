#!/usr/bin/env python3
"""
ROVER DE EXPLORACIÓN - SIMULADOR DE PRUEBAS
Permite probar el sistema sin hardware físico

Funciones:
- Simula Arduino (responde a comandos serial)
- Simula sensores con valores aleatorios
- Simula cámaras con imágenes de prueba
- Servidor UDP/TCP de prueba
- Generador de marcadores sintéticos

Uso:
    python3 test_simulacion.py [modo]
    
Modos:
    arduino  - Simular Arduino por serial
    servidor - Simular servidor completo
    cliente  - Probar comandos UDP
    completo - Todo junto
"""

import socket
import threading
import time
import random
import sys
import cv2
import numpy as np
import struct

# =========================
# SIMULADOR ARDUINO
# =========================
class ArduinoSimulator:
    """Simula respuestas del Arduino Nano"""
    
    def __init__(self):
        self.motor_left_speed = 0
        self.motor_right_speed = 0
        self.motor_left_dir = "STOP"
        self.motor_right_dir = "STOP"
        
        self.servos = {1: 15, 2: 90, 3: 90, 4: 45}
        
        self.temp = 25.0
        self.light = 3.2
        self.humidity = 65
        self.distance = 120
        self.imu = [0.12, -0.05, 9.81, 0.02, -0.01, 0.00]
        
        self.emergency = False
    
    def process_command(self, cmd):
        """Procesar comando y retornar respuesta"""
        cmd = cmd.strip().upper()
        parts = cmd.split()
        
        if not parts:
            return "ERR:EMPTY_CMD"
        
        # MOTORES
        if cmd.startswith("MOTOR LEFT FORWARD"):
            self.motor_left_speed = int(parts[3]) if len(parts) > 3 else 0
            self.motor_left_dir = "FORWARD"
            return "OK:MOTOR_LEFT_FWD"
        
        elif cmd.startswith("MOTOR LEFT BACK"):
            self.motor_left_speed = int(parts[3]) if len(parts) > 3 else 0
            self.motor_left_dir = "BACK"
            return "OK:MOTOR_LEFT_BACK"
        
        elif cmd.startswith("MOTOR RIGHT FORWARD"):
            self.motor_right_speed = int(parts[3]) if len(parts) > 3 else 0
            self.motor_right_dir = "FORWARD"
            return "OK:MOTOR_RIGHT_FWD"
        
        elif cmd.startswith("MOTOR RIGHT BACK"):
            self.motor_right_speed = int(parts[3]) if len(parts) > 3 else 0
            self.motor_right_dir = "BACK"
            return "OK:MOTOR_RIGHT_BACK"
        
        elif cmd.startswith("MOTORS STOP"):
            self.motor_left_speed = 0
            self.motor_right_speed = 0
            self.motor_left_dir = "STOP"
            self.motor_right_dir = "STOP"
            return "OK:MOTORS_STOP"
        
        # SERVOS
        elif cmd.startswith("SERVO"):
            try:
                servo_id = int(parts[1])
                angle = int(parts[2])
                
                if servo_id not in [1, 2, 3, 4]:
                    return "ERR:INVALID_SERVO_ID"
                
                # Aplicar límites
                if servo_id == 1:
                    angle = min(angle, 30)
                elif servo_id == 4:
                    angle = min(angle, 90)
                
                self.servos[servo_id] = angle
                return f"OK:SERVO{servo_id}:{angle}"
            except:
                return "ERR:INVALID_SERVO_CMD"
        
        # SENSORES
        elif cmd.startswith("SENSOR TEMP"):
            # Simular variación
            self.temp += random.uniform(-0.5, 0.5)
            return f"TEMP:{self.temp:.2f}"
        
        elif cmd.startswith("SENSOR LDR"):
            self.light += random.uniform(-0.1, 0.1)
            self.light = max(0, min(5, self.light))
            return f"LDR:{self.light:.2f}"
        
        elif cmd.startswith("SENSOR HUM_CAP"):
            self.humidity += random.randint(-5, 5)
            self.humidity = max(0, min(100, self.humidity))
            return f"HUM:{self.humidity}"
        
        elif cmd.startswith("SENSOR DIST"):
            if random.random() < 0.9:  # 90% de éxito
                self.distance += random.randint(-10, 10)
                self.distance = max(5, min(200, self.distance))
                return f"DIST:{self.distance}"
            else:
                return "DIST:OUT_OF_RANGE"
        
        elif cmd.startswith("SENSOR IMU"):
            # Simular pequeñas variaciones
            for i in range(6):
                self.imu[i] += random.uniform(-0.05, 0.05)
            imu_str = ','.join([f"{v:.3f}" for v in self.imu])
            return f"IMU:{imu_str}"
        
        # SECUENCIAS
        elif cmd.startswith("ACTUATE BRACO START"):
            # Simular secuencia
            return "BRACO:STARTING"
            # En realidad debería retornar BRACO:OK después de 4s
            # pero para simplificar lo hacemos inmediato en el test
        
        # EMERGENCIA
        elif cmd.startswith("EMERGENCY STOP"):
            self.emergency = True
            self.motor_left_speed = 0
            self.motor_right_speed = 0
            return "EMERGENCY:OK"
        
        elif cmd.startswith("EMERGENCY CLEAR"):
            self.emergency = False
            return "EMERGENCY:CLEARED"
        
        else:
            return f"ERR:UNKNOWN_CMD:{cmd}"
    
    def get_status(self):
        """Obtener estado actual"""
        return {
            'left_motor': f"{self.motor_left_dir} {self.motor_left_speed}",
            'right_motor': f"{self.motor_right_dir} {self.motor_right_speed}",
            'servos': self.servos,
            'temp': self.temp,
            'light': self.light,
            'humidity': self.humidity,
            'distance': self.distance,
            'emergency': self.emergency
        }

# =========================
# SIMULADOR DE SERVIDOR
# =========================
class ServerSimulator:
    """Simula el servidor completo (UDP + TCP)"""
    
    def __init__(self, ip='0.0.0.0', udp_port=50000, tcp_port=50001):
        self.ip = ip
        self.udp_port = udp_port
        self.tcp_port = tcp_port
        self.arduino = ArduinoSimulator()
        self.running = True
        self.marker_count = 0
        self.mode = "MANUAL"
        self.saving = False
    
    def start(self):
        """Iniciar servidores"""
        print(f"[SIMULADOR] Iniciando servidor en {self.ip}")
        print(f"[SIMULADOR] UDP: {self.udp_port}, TCP: {self.tcp_port}")
        
        # UDP Thread
        threading.Thread(target=self.udp_server, daemon=True).start()
        
        # TCP Thread
        threading.Thread(target=self.tcp_server, daemon=True).start()
        
        print("[SIMULADOR] Servidores activos")
        print("[SIMULADOR] Presiona Ctrl+C para salir")
        print()
        
        # Status loop
        try:
            while self.running:
                time.sleep(5)
                status = self.arduino.get_status()
                print(f"\n[STATUS] Temp: {status['temp']:.1f}°C, "
                      f"Luz: {status['light']:.1f}V, "
                      f"Dist: {status['distance']}cm, "
                      f"Marcadores: {self.marker_count}")
        except KeyboardInterrupt:
            print("\n[SIMULADOR] Apagando...")
            self.running = False
    
    def udp_server(self):
        """Servidor UDP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip, self.udp_port))
        
        print(f"[UDP] Escuchando en {self.udp_port}")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(4096)
                cmd = data.decode('utf-8').strip()
                
                print(f"[UDP] <- {cmd}")
                
                # Procesar comando
                response = self.process_udp_command(cmd)
                
                print(f"[UDP] -> {response}")
                
                sock.sendto(response.encode('utf-8'), addr)
            
            except Exception as e:
                if self.running:
                    print(f"[UDP] Error: {e}")
    
    def process_udp_command(self, cmd):
        """Procesar comando UDP"""
        parts = cmd.split(':')
        command = parts[0].lower()
        
        try:
            # Movimiento
            if command == "move":
                direction = parts[1]
                velocity = parts[2] if len(parts) > 2 else "80"
                
                if direction == "forward":
                    self.arduino.process_command(f"MOTOR LEFT FORWARD {velocity}")
                    self.arduino.process_command(f"MOTOR RIGHT FORWARD {velocity}")
                elif direction == "back":
                    self.arduino.process_command(f"MOTOR LEFT BACK {velocity}")
                    self.arduino.process_command(f"MOTOR RIGHT BACK {velocity}")
                elif direction == "left":
                    self.arduino.process_command(f"MOTOR LEFT BACK {velocity}")
                    self.arduino.process_command(f"MOTOR RIGHT FORWARD {velocity}")
                elif direction == "right":
                    self.arduino.process_command(f"MOTOR LEFT FORWARD {velocity}")
                    self.arduino.process_command(f"MOTOR RIGHT BACK {velocity}")
                elif direction == "stop":
                    self.arduino.process_command("MOTORS STOP")
                
                return "OK"
            
            # Servos
            elif command == "servo":
                servo_id = parts[1]
                angle = parts[2]
                return self.arduino.process_command(f"SERVO {servo_id} {angle}")
            
            # Sensores
            elif command == "sensor":
                sensor_type = parts[1].upper()
                return self.arduino.process_command(f"SENSOR {sensor_type}")
            
            # Guardado
            elif command == "save":
                action = parts[1]
                if action == "start":
                    self.saving = True
                    return "SAVE:STARTED"
                elif action == "stop":
                    self.saving = False
                    return "SAVE:STOPPED"
                elif action == "point":
                    return "SAVE:POINT_OK"
            
            # Acción
            elif command == "action":
                if parts[1] == "sample":
                    return self.arduino.process_command("ACTUATE BRACO START")
            
            # Emergencia
            elif command == "emergency":
                if parts[1] == "stop":
                    return self.arduino.process_command("EMERGENCY STOP")
                elif parts[1] == "clear":
                    return self.arduino.process_command("EMERGENCY CLEAR")
            
            # Estado
            elif command == "status":
                return f"MODE:{self.mode},MARKERS:{self.marker_count},SAVING:{self.saving}"
            
            # Modo
            elif command == "mode":
                self.mode = parts[1].upper()
                return f"MODE:CHANGED:{self.mode}"
            
            else:
                return "ERR:UNKNOWN_COMMAND"
        
        except Exception as e:
            return f"ERR:{str(e)}"
    
    def tcp_server(self):
        """Servidor TCP de video"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.ip, self.tcp_port))
        server.listen(2)
        
        print(f"[TCP] Escuchando en {self.tcp_port}")
        
        while self.running:
            try:
                client, addr = server.accept()
                print(f"[TCP] Cliente conectado: {addr}")
                threading.Thread(target=self.stream_video, args=(client,), daemon=True).start()
            except Exception as e:
                if self.running:
                    print(f"[TCP] Error: {e}")
    
    def stream_video(self, client_socket):
        """Streaming de video simulado"""
        try:
            frame_count = 0
            while self.running:
                # Generar frame sintético
                frame = self.generate_test_frame(frame_count)
                frame_count += 1
                
                # Si está en modo autónomo, simular detección
                if self.mode == "AUTONOMOUS" and random.random() < 0.1:
                    # 10% de probabilidad de detectar marcador
                    marker_type = random.choice(['Cruz', 'T', 'Círculo', 'Cuadrado', 'Triángulo'])
                    px_u, px_v = 320, 240
                    
                    cv2.circle(frame, (px_u, px_v), 20, (0, 255, 0), 3)
                    cv2.putText(frame, marker_type, (px_u + 25, px_v),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    self.marker_count += 1
                    print(f"[SIMULADOR] Marcador detectado: {marker_type}")
                
                # Codificar y enviar
                _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                data = jpeg.tobytes()
                
                try:
                    client_socket.sendall(struct.pack("Q", len(data)))
                    client_socket.sendall(data)
                except:
                    break
                
                time.sleep(1.0 / 30)  # 30 FPS
        
        except Exception as e:
            print(f"[TCP] Cliente desconectado: {e}")
        finally:
            client_socket.close()
    
    def generate_test_frame(self, frame_num):
        """Generar frame de prueba"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Fondo degradado
        for y in range(480):
            color = int(50 + (y / 480) * 100)
            frame[y, :] = [color, color, color]
        
        # Texto
        cv2.putText(frame, "SIMULADOR ROVER", (180, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        
        cv2.putText(frame, f"Frame: {frame_num}", (20, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        status = self.arduino.get_status()
        cv2.putText(frame, f"Temp: {status['temp']:.1f}C", (20, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.putText(frame, f"Luz: {status['light']:.1f}V", (20, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.putText(frame, f"Modo: {self.mode}", (20, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Círculo animado
        radius = 30 + int(20 * np.sin(frame_num * 0.1))
        cv2.circle(frame, (320, 360), radius, (0, 0, 255), 2)
        
        return frame

# =========================
# TEST CLIENTE
# =========================
def test_client(server_ip='127.0.0.1'):
    """Probar comandos UDP"""
    print(f"[TEST] Conectando a {server_ip}:50000")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    def send_cmd(cmd):
        print(f"\n> {cmd}")
        sock.sendto(cmd.encode('utf-8'), (server_ip, 50000))
        try:
            data, _ = sock.recvfrom(4096)
            response = data.decode('utf-8')
            print(f"< {response}")
            return response
        except socket.timeout:
            print("< [TIMEOUT]")
            return None
    
    print("\n" + "="*50)
    print("TEST DE COMANDOS UDP")
    print("="*50)
    
    # Tests
    print("\n--- MOVIMIENTO ---")
    send_cmd("move:forward:80")
    time.sleep(0.5)
    send_cmd("move:stop")
    
    print("\n--- SERVOS ---")
    send_cmd("servo:1:15")
    send_cmd("servo:2:90")
    
    print("\n--- SENSORES ---")
    send_cmd("sensor:temp")
    send_cmd("sensor:light")
    send_cmd("sensor:dist")
    send_cmd("sensor:imu")
    send_cmd("sensor:hum")
    
    print("\n--- ESTADO ---")
    send_cmd("status")
    
    print("\n--- MODO ---")
    send_cmd("mode:AUTONOMOUS")
    send_cmd("status")
    send_cmd("mode:MANUAL")
    
    print("\n--- EMERGENCIA ---")
    send_cmd("emergency:stop")
    send_cmd("emergency:clear")
    
    print("\n[TEST] Completado")

# =========================
# MAIN
# =========================
def print_usage():
    print("""
Uso: python3 test_simulacion.py [modo]

Modos:
    arduino   - Simular Arduino por terminal
    servidor  - Simular servidor completo (UDP+TCP)
    cliente   - Probar comandos UDP al servidor
    completo  - Iniciar servidor y luego test de cliente
    
Ejemplos:
    python3 test_simulacion.py servidor
    python3 test_simulacion.py cliente 192.168.1.100
    """)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == "arduino":
        print("=== SIMULADOR ARDUINO ===")
        print("Escribe comandos (Ctrl+C para salir):\n")
        
        arduino = ArduinoSimulator()
        
        try:
            while True:
                cmd = input("> ")
                response = arduino.process_command(cmd)
                print(f"< {response}")
        except KeyboardInterrupt:
            print("\nSaliendo...")
    
    elif mode == "servidor":
        server = ServerSimulator()
        server.start()
    
    elif mode == "cliente":
        server_ip = sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1'
        test_client(server_ip)
    
    elif mode == "completo":
        print("[INFO] Iniciando servidor...")
        server = ServerSimulator()
        threading.Thread(target=server.start, daemon=True).start()
        
        print("[INFO] Esperando 2 segundos...")
        time.sleep(2)
        
        print("[INFO] Ejecutando tests...")
        test_client('127.0.0.1')
        
        print("\n[INFO] Servidor sigue corriendo. Presiona Ctrl+C para salir.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nSaliendo...")
    
    else:
        print(f"Modo '{mode}' no reconocido")
        print_usage()
        sys.exit(1)