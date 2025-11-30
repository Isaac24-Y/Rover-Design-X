# servidor_completo.py - Servidor optimizado con todas las funciones
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
import glob

# ==================== CONFIGURACIÓN ====================
def detectar_puerto_arduino():
    posibles = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if posibles:
        print(f"[INFO] Puertos Arduino detectados: {posibles}")
        return posibles[0]
    print("[WARN] No se detectó puerto Arduino automáticamente")
    return None

ARDUINO_PORT = detectar_puerto_arduino() or '/dev/ttyUSB0'
BAUD_RATE = 9600
BUFFER_SIZE = 4096
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT_VIDEO_FRONTAL = 50001
TCP_PORT_VIDEO_SUPERIOR = 50002
TCP_PORT_STATUS = 50003

# Configuración de cámaras (se detectan automáticamente)
CAMERA_FRONTAL = 0
CAMERA_SUPERIOR = 1

# Directorios
os.makedirs("capturas", exist_ok=True)
os.makedirs("logs", exist_ok=True)

# ==================== VARIABLES GLOBALES ====================
ser = None
csv_lock = threading.Lock()
imagen_lock = threading.Lock()
estado_rover = {
    "temperatura": "N/A",
    "humedad": "N/A",
    "luz": "N/A",
    "distancia": "N/A",
    "accel": {"x": 0, "y": 0, "z": 0},
    "gyro": {"x": 0, "y": 0, "z": 0},
    "marcadores_detectados": 0,
    "modo": "manual",
    "brazo_activo": False,
    "velocidad_actual": 200,
    "ultimo_log": "Sistema iniciado",
    "camara_frontal_activa": False,
    "camara_superior_activa": False
}
estado_lock = threading.Lock()

# Frames de cámaras (para captura manual)
frame_frontal_actual = None
frame_superior_actual = None

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
    """Agrega un log con timestamp"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    log_mensaje = f"[{timestamp}] {mensaje}"
    with estado_lock:
        estado_rover["ultimo_log"] = log_mensaje
    print(f"[LOG] {log_mensaje}")
    
    # Guardar en archivo
    try:
        with open("logs/servidor_logs.txt", "a") as f:
            f.write(log_mensaje + "\n")
    except:
        pass

# ==================== COMUNICACIÓN SERIAL ====================
def serial_worker():
    """Worker thread para comandos seriales"""
    while True:
        try:
            comando, callback = serial_queue.get(timeout=0.1)
            if comando is None:
                break
            
            respuesta = enviar_a_arduino_directo(comando)
            if callback:
                callback(respuesta)
            
            serial_queue.task_done()
        except Empty:
            continue
        except Exception as e:
            print(f"[ERROR SERIAL] {e}")

def enviar_a_arduino_async(comando, callback=None):
    """Envía comando de forma asíncrona"""
    serial_queue.put((comando, callback))

def enviar_a_arduino_directo(comando, espera_respuesta=True, timeout=1.0):
    """Envía comando directamente al Arduino"""
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

# ==================== ACTUALIZACIÓN DE SENSORES ====================
def actualizar_sensores_thread():
    """Thread que actualiza sensores cada 0.5 segundos"""
    while True:
        try:
            # Temperatura
            temp = enviar_a_arduino_directo("temp", timeout=0.5)
            with estado_lock:
                estado_rover["temperatura"] = temp
            
            # Humedad
            humedad = enviar_a_arduino_directo("humedad", timeout=0.5)
            with estado_lock:
                estado_rover["humedad"] = humedad
            
            # Luz
            luz = enviar_a_arduino_directo("luz", timeout=0.5)
            with estado_lock:
                estado_rover["luz"] = luz
            
            # Distancia
            dist = enviar_a_arduino_directo("dist", timeout=0.5)
            with estado_lock:
                estado_rover["distancia"] = dist
            
            # MPU6050
            mpu_data = enviar_a_arduino_directo("mpu", timeout=0.5)
            try:
                partes = mpu_data.split()
                if len(partes) >= 12:
                    with estado_lock:
                        estado_rover["accel"]["x"] = partes[1]
                        estado_rover["accel"]["y"] = partes[3]
                        estado_rover["accel"]["z"] = partes[5]
                        estado_rover["gyro"]["x"] = partes[7]
                        estado_rover["gyro"]["y"] = partes[9]
                        estado_rover["gyro"]["z"] = partes[11]
            except:
                pass
            
            time.sleep(0.5)  # Actualización cada 0.5 segundos
            
        except Exception as e:
            print(f"[ERROR SENSORES] {e}")
            time.sleep(1)

# ==================== GUARDADO DE DATOS ====================
def inicializar_csv():
    """Crea archivo CSV con encabezados"""
    archivo = "datos_exploracion.csv"
    if not os.path.exists(archivo):
        with csv_lock:
            with open(archivo, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Timestamp", "PixelsU", "PixelsV", 
                    "Tipo_Marcador", "Intensidad_Luz", 
                    "Temperatura", "Humedad", "Distancia",
                    "Gyro_Z", "Modo"
                ])
    return archivo

def guardar_marcador(pixelsU, pixelsV, tipo_marcador, modo="manual"):
    """Guarda datos de un marcador detectado"""
    archivo = inicializar_csv()
    
    with estado_lock:
        temp = estado_rover["temperatura"]
        luz = estado_rover["luz"]
        humedad = estado_rover["humedad"]
        distancia = estado_rover["distancia"]
        gyro_z = estado_rover["gyro"]["z"]
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    with csv_lock:
        with open(archivo, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, pixelsU, pixelsV, 
                tipo_marcador, luz, temp, humedad, distancia,
                gyro_z, modo
            ])
            f.flush()
    
    agregar_log(f"Datos guardados: {tipo_marcador} ({modo})")
    return timestamp

def guardar_imagen_actual(tipo="manual"):
    """Guarda imagen de cámara frontal actual"""
    global frame_frontal_actual
    
    if frame_frontal_actual is None:
        return "No hay imagen disponible"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"capturas/{tipo}_{timestamp}.jpg"
    
    try:
        with imagen_lock:
            cv2.imwrite(filename, frame_frontal_actual)
        agregar_log(f"Imagen guardada: {filename}")
        return filename
    except Exception as e:
        agregar_log(f"Error guardando imagen: {e}")
        return f"Error: {e}"

def guardar_datos_actual():
    """Guarda datos de sensores actuales sin marcador"""
    archivo = inicializar_csv()
    
    with estado_lock:
        temp = estado_rover["temperatura"]
        luz = estado_rover["luz"]
        humedad = estado_rover["humedad"]
        distancia = estado_rover["distancia"]
        gyro_z = estado_rover["gyro"]["z"]
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    with csv_lock:
        with open(archivo, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, "N/A", "N/A", 
                "Captura_Manual", luz, temp, humedad, distancia,
                gyro_z, "manual"
            ])
            f.flush()
    
    agregar_log(f"Datos manuales guardados")
    return timestamp

# ==================== SECUENCIA AUTÓNOMA ====================
def secuencia_autonoma(tipo_marcador):
    """Ejecuta secuencia autónoma"""
    agregar_log(f"🤖 MARCADOR CLAVE DETECTADO: {tipo_marcador}")
    
    with estado_lock:
        estado_rover["modo"] = "autonomo"
        estado_rover["brazo_activo"] = True
    
    # 1. Activar servo sensor (90°)
    agregar_log("Activando servo sensor (90°)")
    enviar_a_arduino_directo("servo2 90", espera_respuesta=False)
    time.sleep(1.2)
    
    # 2. Activar servo codo (180°)
    agregar_log("Activando servo codo (180°)")
    enviar_a_arduino_directo("servo1 180", espera_respuesta=False)
    time.sleep(1.5)
    
    # 3. Tomar muestra de humedad
    agregar_log("Tomando muestra de humedad...")
    time.sleep(0.5)
    humedad = enviar_a_arduino_directo("humedad", timeout=2.0)
    with estado_lock:
        estado_rover["humedad"] = humedad
    agregar_log(f"Humedad medida: {humedad}")
    time.sleep(1.0)
    
    # 4. Regresar brazo - primero codo
    agregar_log("Regresando codo a 0°")
    enviar_a_arduino_directo("servo1 0", espera_respuesta=False)
    time.sleep(1.2)
    
    # 5. Regresar sensor
    agregar_log("Regresando sensor a 0°")
    enviar_a_arduino_directo("servo2 0", espera_respuesta=False)
    time.sleep(1.0)
    
    with estado_lock:
        estado_rover["brazo_activo"] = False
        estado_rover["modo"] = "manual"
    
    agregar_log("✓ Secuencia autónoma completada")

# ==================== SERVIDOR UDP ====================
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
    
    # ===== MOVIMIENTO CON VELOCIDAD =====
    if comando.startswith("avanzar"):
        partes = comando.split()
        vel = int(partes[1]) if len(partes) > 1 else estado_rover.get("velocidad_actual", 200)
        enviar_a_arduino_async(f"avanzar {vel}")
        agregar_log(f"➡️ Avanzando (vel: {vel})")
        return f"Avanzando {vel}"
    
    elif comando.startswith("retroceder"):
        partes = comando.split()
        vel = int(partes[1]) if len(partes) > 1 else estado_rover.get("velocidad_actual", 200)
        enviar_a_arduino_async(f"retroceder {vel}")
        agregar_log(f"⬅️ Retrocediendo (vel: {vel})")
        return f"Retrocediendo {vel}"
    
    elif comando.startswith("izquierda"):
        partes = comando.split()
        vel = int(partes[1]) if len(partes) > 1 else estado_rover.get("velocidad_actual", 200)
        enviar_a_arduino_async(f"izquierda {vel}")
        agregar_log(f"↺ Girando izquierda (vel: {vel})")
        return f"Izquierda {vel}"
    
    elif comando.startswith("derecha"):
        partes = comando.split()
        vel = int(partes[1]) if len(partes) > 1 else estado_rover.get("velocidad_actual", 200)
        enviar_a_arduino_async(f"derecha {vel}")
        agregar_log(f"↻ Girando derecha (vel: {vel})")
        return f"Derecha {vel}"
    
    elif comando == "stop":
        enviar_a_arduino_async("stop")
        agregar_log("⏸ Detenido")
        return "Detenido"
    
    # ===== CONFIGURAR VELOCIDAD =====
    elif comando.startswith("set_velocidad "):
        try:
            vel = int(comando.split()[1])
            vel = max(0, min(255, vel))
            with estado_lock:
                estado_rover["velocidad_actual"] = vel
            agregar_log(f"Velocidad configurada: {vel}")
            return f"Velocidad: {vel}"
        except:
            return "Error configurando velocidad"
    
    # ===== SERVOS =====
    elif comando.startswith("servo1 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(180, angulo))
            enviar_a_arduino_async(f"servo1 {angulo}")
            agregar_log(f"Servo1 (codo) → {angulo}°")
            return f"Servo1: {angulo}°"
        except:
            return "Error en comando servo1"
    
    elif comando.startswith("servo2 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(180, angulo))
            enviar_a_arduino_async(f"servo2 {angulo}")
            agregar_log(f"Servo2 (sensor) → {angulo}°")
            return f"Servo2: {angulo}°"
        except:
            return "Error en comando servo2"
    
    elif comando.startswith("servo3 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(30, angulo))
            enviar_a_arduino_async(f"servo3 {angulo}")
            agregar_log(f"Servo3 (cam frontal) → {angulo}°")
            return f"Servo3: {angulo}°"
        except:
            return "Error en comando servo3"
    
    elif comando.startswith("servo4 "):
        try:
            angulo = int(comando.split()[1])
            angulo = max(0, min(180, angulo))
            enviar_a_arduino_async(f"servo4 {angulo}")
            agregar_log(f"Servo4 (cam superior) → {angulo}°")
            return f"Servo4: {angulo}°"
        except:
            return "Error en comando servo4"
    
    # ===== CAPTURA MANUAL DE IMAGEN =====
    elif comando == "capturar_imagen":
        filename = guardar_imagen_actual("manual")
        return f"Imagen guardada: {filename}"
    
    # ===== GUARDAR DATOS MANUALES =====
    elif comando == "guardar_datos":
        timestamp = guardar_datos_actual()
        return f"Datos guardados: {timestamp}"
    
    # ===== MARCADORES =====
    elif comando.startswith("marcador:"):
        try:
            partes = comando.split(":")
            tipo = partes[1]
            pixelsU = int(partes[2])
            pixelsV = int(partes[3])
            es_clave = partes[4] == "true"
            
            with estado_lock:
                estado_rover["marcadores_detectados"] += 1
            
            # Guardar imagen
            filename = guardar_imagen_actual("autonomo" if es_clave else "manual")
            
            if es_clave:
                # Iniciar secuencia autónoma
                threading.Thread(
                    target=secuencia_autonoma, 
                    args=(tipo,), 
                    daemon=True
                ).start()
                
                time.sleep(0.5)
                timestamp = guardar_marcador(pixelsU, pixelsV, tipo, modo="autonomo")
            else:
                timestamp = guardar_marcador(pixelsU, pixelsV, tipo, modo="manual")
            
            return f"Marcador guardado: {timestamp}"
            
        except Exception as e:
            agregar_log(f"Error guardando marcador: {e}")
            return f"Error: {e}"
    
    # ===== MPU INMEDIATO =====
    elif comando == "mpu":
        return enviar_a_arduino_directo("mpu", timeout=1.0)
    
    # ===== ESTADO =====
    elif comando == "get_estado":
        with estado_lock:
            return json.dumps(estado_rover)
    
    # ===== TEST MOTORES =====
    elif comando == "test_motores":
        threading.Thread(target=test_motores_secuencia, daemon=True).start()
        return "Iniciando test de motores"
    
    # ===== OTROS =====
    else:
        resp = enviar_a_arduino_directo(comando)
        return resp

def test_motores_secuencia():
    """Secuencia de prueba de motores"""
    agregar_log("=== TEST DE MOTORES ===")
    
    agregar_log("Probando AVANZAR...")
    enviar_a_arduino_directo("avanzar 150", espera_respuesta=False)
    time.sleep(2)
    enviar_a_arduino_directo("stop", espera_respuesta=False)
    time.sleep(0.5)
    
    agregar_log("Probando RETROCEDER...")
    enviar_a_arduino_directo("retroceder 150", espera_respuesta=False)
    time.sleep(2)
    enviar_a_arduino_directo("stop", espera_respuesta=False)
    time.sleep(0.5)
    
    agregar_log("Probando IZQUIERDA...")
    enviar_a_arduino_directo("izquierda 150", espera_respuesta=False)
    time.sleep(2)
    enviar_a_arduino_directo("stop", espera_respuesta=False)
    time.sleep(0.5)
    
    agregar_log("Probando DERECHA...")
    enviar_a_arduino_directo("derecha 150", espera_respuesta=False)
    time.sleep(2)
    enviar_a_arduino_directo("stop", espera_respuesta=False)
    
    agregar_log("=== TEST COMPLETADO ===")

# ==================== SERVIDOR TCP VIDEO ====================
def servidor_video_frontal():
    """Servidor de video para cámara frontal - CONTINÚA SI FALLA"""
    global frame_frontal_actual
    
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT_VIDEO_FRONTAL))
    srv.listen(1)
    print(f"[SERVIDOR] Video frontal en puerto {TCP_PORT_VIDEO_FRONTAL}")
    
    while True:
        conn, addr = srv.accept()
        print(f"[VIDEO FRONTAL] Cliente conectado: {addr}")
        
        cap = cv2.VideoCapture(CAMERA_FRONTAL)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not cap.isOpened():
            print("[WARN] Cámara frontal no disponible - Enviando imagen negra")
            with estado_lock:
                estado_rover["camara_frontal_activa"] = False
            
            # Enviar imagen negra
            try:
                frame_negro = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame_negro, "CAMARA NO DISPONIBLE", (150, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                while True:
                    _, buffer = cv2.imencode('.jpg', frame_negro, [
                        int(cv2.IMWRITE_JPEG_QUALITY), 60
                    ])
                    data = buffer.tobytes()
                    conn.sendall(struct.pack("Q", len(data)) + data)
                    time.sleep(0.1)
            except:
                pass
            finally:
                conn.close()
            continue
        
        with estado_lock:
            estado_rover["camara_frontal_activa"] = True
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Guardar frame actual para captura manual
                with imagen_lock:
                    frame_frontal_actual = frame.copy()
                
                _, buffer = cv2.imencode('.jpg', frame, [
                    int(cv2.IMWRITE_JPEG_QUALITY), 60
                ])
                data = buffer.tobytes()
                
                conn.sendall(struct.pack("Q", len(data)) + data)
                time.sleep(0.033)
                
        except Exception as e:
            print(f"[VIDEO FRONTAL] Desconectado: {e}")
        finally:
            cap.release()
            conn.close()

def servidor_video_superior():
    """Servidor de video para cámara superior - CONTINÚA SI FALLA"""
    global frame_superior_actual
    
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT_VIDEO_SUPERIOR))
    srv.listen(1)
    print(f"[SERVIDOR] Video superior en puerto {TCP_PORT_VIDEO_SUPERIOR}")
    
    while True:
        conn, addr = srv.accept()
        print(f"[VIDEO SUPERIOR] Cliente conectado: {addr}")
        
        cap = cv2.VideoCapture(CAMERA_SUPERIOR)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not cap.isOpened():
            print("[WARN] Cámara superior no disponible - Enviando imagen negra")
            with estado_lock:
                estado_rover["camara_superior_activa"] = False
            
            # Enviar imagen negra
            try:
                frame_negro = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame_negro, "CAMARA NO DISPONIBLE", (150, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                while True:
                    _, buffer = cv2.imencode('.jpg', frame_negro, [
                        int(cv2.IMWRITE_JPEG_QUALITY), 60
                    ])
                    data = buffer.tobytes()
                    conn.sendall(struct.pack("Q", len(data)) + data)
                    time.sleep(0.1)
            except:
                pass
            finally:
                conn.close()
            continue
        
        with estado_lock:
            estado_rover["camara_superior_activa"] = True
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                with imagen_lock:
                    frame_superior_actual = frame.copy()
                
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
    """Envía estado del rover periódicamente"""
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
    print("   🤖 SERVIDOR ROVER - VERSION COMPLETA")
    print("=" * 60)
    
    # Conectar Arduino
    if not conectar_arduino():
        print("[WARN] Servidor continuará sin Arduino")
    
    # Inicializar CSV
    inicializar_csv()
    
    # Iniciar threads
    print("\n[INICIO] Iniciando servicios...")
    
    threading.Thread(target=serial_worker, daemon=True).start()
    threading.Thread(target=actualizar_sensores_thread, daemon=True).start()
    threading.Thread(target=servidor_udp, daemon=True).start()
    threading.Thread(target=servidor_status, daemon=True).start()
    threading.Thread(target=servidor_video_superior, daemon=True).start()
    
    print("\n[LISTO] ✓ Todos los servicios iniciados")
    print(f"\n🎥 Cámara frontal: puerto {TCP_PORT_VIDEO_FRONTAL}")
    print(f"🎥 Cámara superior: puerto {TCP_PORT_VIDEO_SUPERIOR}")
    print(f"📡 Comandos UDP: puerto {UDP_PORT}")
    print(f"📊 Status TCP: puerto {TCP_PORT_STATUS}")
    print(f"📁 Capturas: ./capturas/")
    print(f"📄 CSV: ./datos_exploracion.csv")
    print("\n[SERVIDOR] Presiona Ctrl+C para detener\n")
    
    # Video frontal en thread principal
    servidor_video_frontal()