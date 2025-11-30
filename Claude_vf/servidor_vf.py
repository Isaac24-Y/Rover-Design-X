# server_mejorado.py - Servidor Raspberry Pi optimizado
import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os
import json
from queue import Queue, Empty
from datetime import datetime

# ==================== CONFIGURACIÓN ====================
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 4096
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT_VIDEO = 50001
TCP_PORT_STATUS = 50002

# Configuración de cámaras
CAMERA_FRONTAL = 0
CAMERA_SUPERIOR = 2

# ==================== VARIABLES GLOBALES ====================
ser = None
csv_lock = threading.Lock()
guardar_datos = False
estado_rover = {
    "temperatura": "N/A",
    "humedad": "N/A",
    "luz": "N/A",
    "distancia": "N/A",
    "accel": {"x": 0, "y": 0, "z": 0},
    "gyro": {"x": 0, "y": 0, "z": 0},
    "marcadores_detectados": 0,
    "modo": "manual",  # manual o autonomo
    "brazo_activo": False,
    "ultimo_log": "Sistema iniciado"
}
estado_lock = threading.Lock()

# Cola de comandos serial para evitar colisiones
serial_queue = Queue()

# ==================== INICIALIZAR ARDUINO ====================
def conectar_arduino():
    global ser
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"[SERVIDOR] ✓ Arduino conectado en {ARDUINO_PORT}")
        agregar_log("Arduino conectado correctamente")
        return True
    except Exception as e:
        print(f"[ERROR] ✗ No se pudo conectar al Arduino: {e}")
        agregar_log(f"Error conectando Arduino: {e}")
        return False

# ==================== GESTIÓN DE LOGS ====================
def agregar_log(mensaje):
    """Agrega un log con timestamp al estado global"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    log_mensaje = f"[{timestamp}] {mensaje}"
    with estado_lock:
        estado_rover["ultimo_log"] = log_mensaje
    print(f"[LOG] {log_mensaje}")

# ==================== COMUNICACIÓN SERIAL OPTIMIZADA ====================
def serial_worker():
    """Worker thread que procesa comandos seriales en orden"""
    while True:
        try:
            comando, callback = serial_queue.get(timeout=0.1)
            if comando is None:  # señal de parada
                break
            
            respuesta = enviar_a_arduino_directo(comando)
            if callback:
                callback(respuesta)
            
            serial_queue.task_done()
        except Empty:
            continue
        except Exception as e:
            print(f"[ERROR SERIAL] {e}")
            agregar_log(f"Error en serial worker: {e}")

def enviar_a_arduino_async(comando, callback=None):
    """Envía comando de forma asíncrona"""
    serial_queue.put((comando, callback))

def enviar_a_arduino_directo(comando, espera_respuesta=True, timeout=1.0):
    """Envía comando directamente al Arduino (uso interno)"""
    if ser is None or not ser.is_open:
        return "Arduino no disponible"
    
    try:
        ser.reset_input_buffer()
        ser.write((comando + '\n').encode())
        
        if espera_respuesta:
            deadline = time.time() + timeout
            while time.time() < deadline:
                if ser.in_waiting > 0:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line:
                        return line
                time.sleep(0.01)
            return "Sin respuesta"
        return "Enviado"
    except Exception as e:
        return f"Error: {e}"

# ==================== ACTUALIZACIÓN PERIÓDICA DE SENSORES ====================
def actualizar_sensores_thread():
    """Thread que actualiza sensores cada 0.5 segundos"""
    while True:
        try:
            # Leer temperatura
            temp = enviar_a_arduino_directo("temp", timeout=0.5)
            with estado_lock:
                estado_rover["temperatura"] = temp
            
            # Leer humedad
            humedad = enviar_a_arduino_directo("humedad", timeout=0.5)
            with estado_lock:
                estado_rover["humedad"] = humedad
            
            # Leer luz
            luz = enviar_a_arduino_directo("luz", timeout=0.5)
            with estado_lock:
                estado_rover["luz"] = luz
            
            # Leer distancia
            dist = enviar_a_arduino_directo("dist", timeout=0.5)
            with estado_lock:
                estado_rover["distancia"] = dist
            
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR SENSORES] {e}")
            time.sleep(1)

# ==================== GUARDADO DE DATOS ====================
def inicializar_csv():
    """Crea archivo CSV con encabezados si no existe"""
    archivo = "datos_exploracion.csv"
    if not os.path.exists(archivo):
        with csv_lock:
            with open(archivo, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Timestamp", "PixelsU", "PixelsV", 
                    "Tipo_Marcador", "Intensidad_Luz", 
                    "Temperatura", "Humedad", "Modo"
                ])
    return archivo

def guardar_marcador(pixelsU, pixelsV, tipo_marcador, modo="manual"):
    """Guarda datos de un marcador detectado"""
    archivo = inicializar_csv()
    
    with estado_lock:
        temp = estado_rover["temperatura"]
        luz = estado_rover["luz"]
        humedad = estado_rover["humedad"] if modo == "autonomo" else "0"
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    with csv_lock:
        with open(archivo, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, pixelsU, pixelsV, 
                tipo_marcador, luz, temp, humedad, modo
            ])
            f.flush()
    
    agregar_log(f"Marcador '{tipo_marcador}' guardado ({modo})")
    return timestamp

# ==================== SECUENCIA AUTÓNOMA ====================
def secuencia_autonoma(tipo_marcador):
    """Ejecuta secuencia autónoma cuando se detecta marcador clave"""
    agregar_log(f"🤖 MARCADOR CLAVE DETECTADO: {tipo_marcador}")
    
    with estado_lock:
        estado_rover["modo"] = "autonomo"
        estado_rover["brazo_activo"] = True
    
    # Activar brazo - Sensor primero (90°)
    agregar_log("Activando servo sensor (90°)")
    enviar_a_arduino_directo("servo2 90", espera_respuesta=False)
    time.sleep(1)
    
    # Activar brazo - Codo después (180°)
    agregar_log("Activando servo codo (180°)")
    enviar_a_arduino_directo("servo1 180", espera_respuesta=False)
    time.sleep(1.5)
    
    # Tomar muestra de humedad
    agregar_log("Tomando muestra de humedad...")
    humedad = enviar_a_arduino_directo("humedad", timeout=2.0)
    with estado_lock:
        estado_rover["humedad"] = humedad
    
    agregar_log(f"Humedad medida: {humedad}")
    time.sleep(1)
    
    # Regresar brazo a posición inicial
    agregar_log("Regresando brazo a posición inicial")
    enviar_a_arduino_directo("servo1 0", espera_respuesta=False)
    time.sleep(1)
    enviar_a_arduino_directo("servo2 0", espera_respuesta=False)
    time.sleep(1)
    
    with estado_lock:
        estado_rover["brazo_activo"] = False
        estado_rover["modo"] = "manual"
    
    agregar_log("✓ Secuencia autónoma completada")

# ==================== SERVIDOR UDP (COMANDOS) ====================
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {UDP_PORT}")
    
    while True:
        try:
            msg, address = sock.recvfrom(BUFFER_SIZE)
            comando = msg.decode('utf-8').strip()
            
            respuesta = procesar_comando(comando)
            sock.sendto(respuesta.encode('utf-8'), address)
            
        except Exception as e:
            print(f"[ERROR UDP] {e}")

def procesar_comando(comando):
    """Procesa comandos del cliente"""
    
    # ===== MOVIMIENTO =====
    if comando == "avanzar":
        enviar_a_arduino_async("avanzar 200")
        agregar_log("➡️ Avanzando")
        return "Avanzando"
    
    elif comando == "retroceder":
        enviar_a_arduino_async("retroceder 200")
        agregar_log("⬅️ Retrocediendo")
        return "Retrocediendo"
    
    elif comando == "izquierda":
        enviar_a_arduino_async("izq 200")
        agregar_log("↺ Girando izquierda")
        return "Girando izquierda"
    
    elif comando == "derecha":
        enviar_a_arduino_async("der 200")
        agregar_log("↻ Girando derecha")
        return "Girando derecha"
    
    elif comando == "stop":
        enviar_a_arduino_async("stop")
        agregar_log("⏸ Detenido")
        return "Detenido"
    
    # ===== SERVOS =====
    elif comando.startswith("servo1 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(30, angulo))  # Limitar 0-30°
            enviar_a_arduino_async(f"servo1 {angulo}")
            agregar_log(f"Servo1 (cámara frontal) → {angulo}°")
            return f"Servo1: {angulo}°"
        except:
            return "Error en comando servo1"
    
    elif comando.startswith("servo4 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(180, angulo))
            enviar_a_arduino_async(f"servo4 {angulo}")
            agregar_log(f"Servo4 (cámara superior) → {angulo}°")
            return f"Servo4: {angulo}°"
        except:
            return "Error en comando servo4"
    
    # ===== MARCADORES =====
    elif comando.startswith("marcador:"):
        try:
            # Formato: marcador:tipo:pixelsU:pixelsV:clave
            partes = comando.split(":")
            tipo = partes[1]
            pixelsU = int(partes[2])
            pixelsV = int(partes[3])
            es_clave = partes[4] == "true"
            
            with estado_lock:
                estado_rover["marcadores_detectados"] += 1
            
            if es_clave:
                # Iniciar secuencia autónoma en thread separado
                threading.Thread(
                    target=secuencia_autonoma, 
                    args=(tipo,), 
                    daemon=True
                ).start()
                
                # Esperar un momento para que tome humedad
                time.sleep(3)
                timestamp = guardar_marcador(pixelsU, pixelsV, tipo, modo="autonomo")
            else:
                timestamp = guardar_marcador(pixelsU, pixelsV, tipo, modo="manual")
            
            return f"Marcador guardado: {timestamp}"
            
        except Exception as e:
            return f"Error guardando marcador: {e}"
    
    # ===== LECTURA INMEDIATA MPU =====
    elif comando == "mpu":
        mpu_data = enviar_a_arduino_directo("mpu", timeout=1.0)
        try:
            # Parse: "AX: 123 AY: 456 AZ: 789 GX: 12 GY: 34 GZ: 56"
            partes = mpu_data.split()
            with estado_lock:
                estado_rover["accel"]["x"] = partes[1]
                estado_rover["accel"]["y"] = partes[3]
                estado_rover["accel"]["z"] = partes[5]
                estado_rover["gyro"]["x"] = partes[7]
                estado_rover["gyro"]["y"] = partes[9]
                estado_rover["gyro"]["z"] = partes[11]
        except:
            pass
        return mpu_data
    
    # ===== OBTENER ESTADO =====
    elif comando == "get_estado":
        with estado_lock:
            return json.dumps(estado_rover)
    
    # ===== OTROS =====
    else:
        resp = enviar_a_arduino_directo(comando)
        return resp

# ==================== SERVIDOR TCP VIDEO ====================
def servidor_video_frontal():
    """Servidor de video para cámara frontal"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT_VIDEO))
    srv.listen(1)
    print(f"[SERVIDOR] Video frontal en puerto {TCP_PORT_VIDEO}")
    
    while True:
        conn, addr = srv.accept()
        print(f"[VIDEO FRONTAL] Cliente conectado: {addr}")
        
        cap = cv2.VideoCapture(CAMERA_FRONTAL)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not cap.isOpened():
            print("[ERROR] Cámara frontal no disponible")
            conn.close()
            continue
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Comprimir imagen
                _, buffer = cv2.imencode('.jpg', frame, [
                    int(cv2.IMWRITE_JPEG_QUALITY), 60
                ])
                data = buffer.tobytes()
                
                # Enviar tamaño + datos
                conn.sendall(struct.pack("Q", len(data)) + data)
                time.sleep(0.033)  # ~30 FPS
                
        except Exception as e:
            print(f"[VIDEO FRONTAL] Desconectado: {e}")
        finally:
            cap.release()
            conn.close()

def servidor_video_superior():
    """Servidor de video para cámara superior"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT_VIDEO + 1))
    srv.listen(1)
    print(f"[SERVIDOR] Video superior en puerto {TCP_PORT_VIDEO + 1}")
    
    while True:
        conn, addr = srv.accept()
        print(f"[VIDEO SUPERIOR] Cliente conectado: {addr}")
        
        cap = cv2.VideoCapture(CAMERA_SUPERIOR)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not cap.isOpened():
            print("[ERROR] Cámara superior no disponible")
            conn.close()
            continue
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                _, buffer = cv2.imencode('.jpg', frame, [
                    int(cv2.IMWRITE_JPEG_QUALITY), 60
                ])
                data = buffer.tobytes()
                
                conn.sendall(struct.pack("Q", len(data)) + data)
                time.sleep(0.033)
                
        except Exception as e:
            print(f"[VIDEO SUPERIOR] Desconectado: {e}")
        finally:
            cap.release()
            conn.close()

# ==================== SERVIDOR TCP STATUS ====================
def servidor_status():
    """Envía estado del rover periódicamente via TCP"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT_STATUS))
    srv.listen(1)
    print(f"[SERVIDOR] Status en puerto {TCP_PORT_STATUS}")
    
    while True:
        conn, addr = srv.accept()
        print(f"[STATUS] Cliente conectado: {addr}")
        
        try:
            while True:
                with estado_lock:
                    estado_json = json.dumps(estado_rover)
                
                # Enviar tamaño + JSON
                data = estado_json.encode('utf-8')
                conn.sendall(struct.pack("I", len(data)) + data)
                time.sleep(0.5)
                
        except Exception as e:
            print(f"[STATUS] Desconectado: {e}")
        finally:
            conn.close()

# ==================== MAIN ====================
if __name__ == "__main__":
    print("=" * 60)
    print("   🤖 SERVIDOR ROVER DE EXPLORACIÓN")
    print("=" * 60)
    
    # Conectar Arduino
    if not conectar_arduino():
        print("[ERROR] No se puede iniciar sin Arduino")
        exit(1)
    
    # Inicializar CSV
    inicializar_csv()
    
    # Iniciar threads
    print("\n[INICIO] Iniciando threads...")
    
    # Serial worker
    threading.Thread(target=serial_worker, daemon=True).start()
    
    # Actualización de sensores
    threading.Thread(target=actualizar_sensores_thread, daemon=True).start()
    
    # Servidor UDP (comandos)
    threading.Thread(target=servidor_udp, daemon=True).start()
    
    # Servidor TCP (status)
    threading.Thread(target=servidor_status, daemon=True).start()
    
    # Servidor video superior
    threading.Thread(target=servidor_video_superior, daemon=True).start()
    
    print("[LISTO] ✓ Todos los servicios iniciados")
    print("\n🎥 Cámara frontal: puerto", TCP_PORT_VIDEO)
    print("🎥 Cámara superior: puerto", TCP_PORT_VIDEO + 1)
    print("📡 Comandos UDP: puerto", UDP_PORT)
    print("📊 Status TCP: puerto", TCP_PORT_STATUS)
    print("\n[SERVIDOR] Presiona Ctrl+C para detener\n")
    
    # Video frontal en thread principal
    servidor_video_frontal()