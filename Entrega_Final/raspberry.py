# Servidor Raspberry Pi - Rover de Exploración
import socket
import threading
import struct
import time
import cv2
import serial
import sys
from queue import Queue

# ========== CONFIGURACIÓN ==========
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT_VIDEO1 = 50001  # Cámara frontal
TCP_PORT_VIDEO2 = 50002  # Cámara superior
BUFFER_SIZE = 4096

# ========== VARIABLES GLOBALES ==========
ser = None
arduino_lock = threading.Lock()
comando_queue = Queue()

# ========== CONEXIÓN SERIAL CON ARDUINO ==========
def conectar_arduino():
    """Establece conexión con Arduino"""
    global ser
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Esperar reset de Arduino
        print(f"[SERVIDOR] ✅ Conectado a Arduino en {ARDUINO_PORT}")
        
        # Leer línea de bienvenida del Arduino
        if ser.in_waiting > 0:
            bienvenida = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"[ARDUINO] {bienvenida}")
        
        return True
    except Exception as e:
        print(f"[ERROR] ❌ No se pudo conectar al Arduino: {e}")
        ser = None
        return False

def reconectar_arduino():
    """Intenta reconectar con Arduino"""
    global ser
    print("[SERVIDOR] Intentando reconectar con Arduino...")
    if ser:
        try:
            ser.close()
        except:
            pass
    time.sleep(1)
    return conectar_arduino()

# ========== COMUNICACIÓN CON ARDUINO ==========
def enviar_comando_arduino(comando, espera_respuesta=True, timeout=3.0):
    """
    Envía comando al Arduino y retorna respuesta
    """
    if ser is None or not ser.is_open:
        return "ERROR: Arduino no disponible"
    
    with arduino_lock:
        try:
            # Limpiar buffer
            ser.reset_input_buffer()
            
            # Enviar comando
            comando_completo = comando.strip() + '\n'
            ser.write(comando_completo.encode('utf-8'))
            print(f"[→ ARDUINO] {comando}")
            
            if not espera_respuesta:
                return "OK"
            
            # Esperar respuesta
            tiempo_inicio = time.time()
            respuesta_completa = []
            
            while (time.time() - tiempo_inicio) < timeout:
                if ser.in_waiting > 0:
                    linea = ser.readline().decode('utf-8', errors='ignore').strip()
                    if linea:
                        respuesta_completa.append(linea)
                        print(f"[← ARDUINO] {linea}")
                        
                        # Si es una respuesta de tipo RESP: o LOG:, retornar inmediatamente
                        if linea.startswith(("RESP:", "LOG:", "STATUS:", "SENSORES:", "IMU:")):
                            return linea
                
                time.sleep(0.05)
            
            # Si hay respuestas, retornar la última o todas concatenadas
            if respuesta_completa:
                return " | ".join(respuesta_completa)
            
            return "TIMEOUT: Sin respuesta del Arduino"
        
        except serial.SerialException as e:
            print(f"[ERROR SERIAL] {e}")
            # Intentar reconectar
            if reconectar_arduino():
                return "ERROR: Reconectado, reintente comando"
            return f"ERROR: Problema de conexión - {e}"
        
        except Exception as e:
            print(f"[ERROR] {e}")
            return f"ERROR: {e}"

def leer_logs_arduino():
    """Thread que lee continuamente logs del Arduino"""
    global ser
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    linea = ser.readline().decode('utf-8', errors='ignore').strip()
                    if linea and linea.startswith("LOG:"):
                        print(f"[ARDUINO LOG] {linea[4:]}")
            except:
                pass
        time.sleep(0.1)

# ========== SERVIDOR UDP (COMANDOS) ==========
def servidor_udp():
    """Maneja comandos UDP desde el cliente"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, SO_REUSEADDR, 1)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] 🌐 UDP escuchando en {SERVER_IP}:{UDP_PORT}")
    
    while True:
        try:
            mensaje, direccion_cliente = sock.recvfrom(BUFFER_SIZE)
            comando = mensaje.decode('utf-8', errors='ignore').strip()
            
            if not comando:
                continue
            
            print(f"[UDP] 📨 Comando recibido de {direccion_cliente}: {comando}")
            
            # Procesar comando
            respuesta = procesar_comando(comando)
            
            # Enviar respuesta al cliente
            sock.sendto(respuesta.encode('utf-8'), direccion_cliente)
            print(f"[UDP] 📤 Respuesta enviada: {respuesta[:100]}...")
        
        except Exception as e:
            print(f"[ERROR UDP] {e}")
            continue

def procesar_comando(comando):
    """
    Procesa comandos recibidos y los reenvía al Arduino
    """
    comando_lower = comando.lower().strip()
    
    # Comandos especiales del servidor
    if comando_lower == "ping":
        return "PONG"
    
    elif comando_lower == "status_servidor":
        estado_arduino = "CONECTADO" if (ser and ser.is_open) else "DESCONECTADO"
        return f"Servidor OK | Arduino: {estado_arduino}"
    
    # Comandos que requieren respuesta inmediata
    comandos_con_respuesta = [
        "sensores", "temperatura", "luz", "humedad_suelo", 
        "humedad_aire", "distancia", "imu", "status"
    ]
    
    # Comandos de movimiento (sin esperar respuesta larga)
    comandos_movimiento = [
        "adelante", "atras", "izquierda", "derecha", "stop"
    ]
    
    # Determinar si esperar respuesta
    espera_respuesta = True
    if any(cmd in comando_lower for cmd in comandos_movimiento):
        espera_respuesta = False
    
    # Enviar al Arduino
    respuesta = enviar_comando_arduino(comando, espera_respuesta=espera_respuesta)
    
    return respuesta

# ========== SERVIDOR TCP VIDEO ==========
def servidor_video(puerto, camara_id):
    """
    Servidor de streaming de video por TCP
    puerto: Puerto TCP a usar
    camara_id: 0 para cámara frontal, 1 para superior
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, puerto))
    srv.listen(5)
    print(f"[SERVIDOR] 📹 Video escuchando en {SERVER_IP}:{puerto} (Cámara {camara_id})")
    
    while True:
        try:
            conn, addr = srv.accept()
            print(f"[VIDEO-{camara_id}] 🔗 Cliente conectado: {addr}")
            
            # Abrir cámara
            cap = cv2.VideoCapture(camara_id)
            
            if not cap.isOpened():
                print(f"[ERROR] ❌ No se pudo abrir la cámara {camara_id}")
                conn.close()
                continue
            
            # Configurar cámara
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            try:
                while True:
                    ret, frame = cap.read()
                    
                    if not ret:
                        print(f"[VIDEO-{camara_id}] ⚠️ Error al capturar frame")
                        break
                    
                    # Comprimir frame
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
                    _, buffer = cv2.imencode('.jpg', frame, encode_param)
                    data = buffer.tobytes()
                    
                    # Enviar tamaño + datos
                    size = len(data)
                    conn.sendall(struct.pack("Q", size) + data)
                    
                    # Control de velocidad (evitar saturación)
                    time.sleep(0.033)  # ~30 FPS
            
            except (BrokenPipeError, ConnectionResetError):
                print(f"[VIDEO-{camara_id}] 🔌 Cliente desconectado")
            
            except Exception as e:
                print(f"[VIDEO-{camara_id}] ❌ Error: {e}")
            
            finally:
                cap.release()
                conn.close()
                print(f"[VIDEO-{camara_id}] 🔚 Conexión cerrada")
        
        except Exception as e:
            print(f"[ERROR VIDEO-{camara_id}] {e}")
            time.sleep(1)

# ========== MONITOR DE SALUD ==========
def monitor_sistema():
    """Monitorea el estado del sistema cada 10 segundos"""
    while True:
        time.sleep(10)
        
        # Verificar conexión con Arduino
        if ser is None or not ser.is_open:
            print("[MONITOR] ⚠️ Arduino desconectado, intentando reconectar...")
            reconectar_arduino()
        else:
            # Hacer ping al Arduino
            respuesta = enviar_comando_arduino("status", espera_respuesta=True, timeout=2)
            if "TIMEOUT" in respuesta or "ERROR" in respuesta:
                print("[MONITOR] ⚠️ Arduino no responde")
            else:
                print(f"[MONITOR] ✅ Sistema OK")

# ========== MAIN ==========
def main():
    """Función principal"""
    print("=" * 60)
    print("🤖 SERVIDOR ROVER DE EXPLORACIÓN")
    print("=" * 60)
    
    # Conectar con Arduino
    if not conectar_arduino():
        print("[ERROR] No se pudo iniciar sin Arduino")
        respuesta = input("¿Desea continuar sin Arduino? (s/n): ")
        if respuesta.lower() != 's':
            sys.exit(1)
    
    # Iniciar threads
    threads = [
        threading.Thread(target=servidor_udp, daemon=True, name="UDP-Server"),
        threading.Thread(target=lambda: servidor_video(TCP_PORT_VIDEO1, 0), daemon=True, name="Video-Frontal"),
        threading.Thread(target=lambda: servidor_video(TCP_PORT_VIDEO2, 1), daemon=True, name="Video-Superior"),
        threading.Thread(target=leer_logs_arduino, daemon=True, name="Arduino-Logger"),
        threading.Thread(target=monitor_sistema, daemon=True, name="System-Monitor")
    ]
    
    for t in threads:
        t.start()
        print(f"[SERVIDOR] ✅ Thread iniciado: {t.name}")
    
    print("\n" + "=" * 60)
    print("✅ SERVIDOR COMPLETAMENTE OPERATIVO")
    print("=" * 60)
    print(f"📡 Comando UDP: {UDP_PORT}")
    print(f"📹 Video Frontal: {TCP_PORT_VIDEO1}")
    print(f"📹 Video Superior: {TCP_PORT_VIDEO2}")
    print("=" * 60)
    print("\nPresione Ctrl+C para detener el servidor\n")
    
    # Mantener vivo el proceso principal
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SERVIDOR] 🛑 Deteniendo servidor...")
        if ser:
            enviar_comando_arduino("stop", espera_respuesta=False)
            ser.close()
        print("[SERVIDOR] ✅ Servidor detenido")
        sys.exit(0)

if __name__ == "__main__":
    main()