import socket
import threading
import struct
import time
import cv2
import serial

# ─ Configuración general ─
SERVER_IP = '0.0.0.0'
COMMAND_PORT = 50000  # UDP para comandos Arduino
VIDEO_PORT_1 = 50001  # TCP para cámara 1
VIDEO_PORT_2 = 50002  # TCP para cámara 2
BUFFER_SIZE = 1024

# ─ Conexión al Arduino ─
arduino_port = '/dev/ttyACM0'
baud_rate = 9600
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"✅ Conectado a Arduino en {arduino_port}")
except Exception as e:
    ser = None
    print(f"⚠️ No se pudo conectar al Arduino: {e}")

# ─ Función para enviar comandos al Arduino ─
def arduino(men):
    if ser is None:
        print("⚠️ Arduino no conectado.")
        return

    comando = men.lower()
    if comando in ['servo1', 'temperatura', 'luz', 'dc']:
        ser.write(comando.encode())
        print(f"📤 Comando '{comando}' enviado.")
        time.sleep(0.1)
        if ser.in_waiting > 0:
            respuesta = ser.readline().decode().strip()
            print(f"📥 Respuesta Arduino: {respuesta}")
    else:
        print("❌ Comando no válido.")

# ─ Función UDP para recibir comandos ─
def mensaje_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, COMMAND_PORT))
    print(f"[SERVER] Escuchando comandos UDP en {SERVER_IP}:{COMMAND_PORT}")

    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        msg = msg.decode('utf-8').strip()
        print(f"[COMANDO] '{msg}' de {addr}")
        sock.sendto(b"OK", addr)
        arduino(msg)

# ─ Función TCP para enviar video de una cámara ─
def camara_tcp(cam_index, port):
    srv_vid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_vid.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_vid.bind((SERVER_IP, port))
    srv_vid.listen(1)
    print(f"[SERVER] Cámara {cam_index} esperando conexión en puerto {port}...")

    conn_vid, addr_v = srv_vid.accept()
    print(f"[SERVER] Cliente conectado a cámara {cam_index} desde {addr_v}")

    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    time.sleep(2)

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"[CAM {cam_index}] No se pudo leer frame.")
            break

        _, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = encimg.tobytes()
        try:
            conn_vid.sendall(struct.pack("Q", len(data)) + data)
        except:
            print(f"[CAM {cam_index}] Cliente desconectado.")
            break

    cap.release()
    conn_vid.close()
    srv_vid.close()

# ─ Main ─
if __name__ == "__main__":
    # Hilo de comandos
    threading.Thread(target=mensaje_udp, daemon=True).start()

    # Hilos de cámaras
    threading.Thread(target=camara_tcp, args=(0, VIDEO_PORT_1), daemon=True).start()
    threading.Thread(target=camara_tcp, args=(1, VIDEO_PORT_2), daemon=True).start()

    print("🎥 Servidor de cámaras iniciado.")
    while True:
        time.sleep(1)
