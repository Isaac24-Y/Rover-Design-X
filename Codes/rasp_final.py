import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ────────────────────────────────────────────────
# Configuración
# ────────────────────────────────────────────────
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 1024
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001

SAVE_DIR = "server_data"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "sensores.csv")

guardar_datos_flag = False
ser = None

# ─ Conexión con Arduino
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] ✅ Conectado a Arduino en {ARDUINO_PORT}")
except Exception as e:
    print("[ERROR] No se pudo conectar al Arduino:", e)

# ────────────────────────────────────────────────
# Enviar comandos al Arduino
# ────────────────────────────────────────────────
def enviar_a_arduino(comando):
    global guardar_datos_flag
    comando = comando.strip().lower()

    if not ser or not ser.is_open:
        return "Arduino no disponible."

    # Guardado de datos
    if comando == "guardar":
        if not guardar_datos_flag:
            guardar_datos_flag = True
            threading.Thread(target=guardar_datos_sensores, daemon=True).start()
            return "Guardado de datos iniciado."
        return "Ya se está guardando."
    elif comando == "detener":
        guardar_datos_flag = False
        return "Guardado detenido."

    # Comandos de actuadores
    if comando in ['servo1', 'temperatura', 'luz', 'dc', 'derecha', 'izquierda', 'stop']:
        ser.write((comando + '\n').encode())
        time.sleep(0.1)
        if ser.in_waiting > 0:
            return ser.readline().decode(errors='ignore').strip()
        return "Comando enviado al Arduino."
    return "Comando no válido."

# ────────────────────────────────────────────────
# Guardar datos (frame + sensores)
# ────────────────────────────────────────────────
def guardar_datos_sensores():
    global guardar_datos_flag
    cap = cv2.VideoCapture(0)
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Tiempo","Temperatura","Luz","ArchivoImagen"])

    while guardar_datos_flag:
        ret, frame = cap.read()
        if not ret:
            continue

        # Guardar frame
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_file = os.path.join(SAVE_DIR, f"{timestamp}.png")
        cv2.imwrite(img_file, frame)

        # Pedir sensores al Arduino
        ser.write(b'temperatura\n')
        time.sleep(0.05)
        temp = ser.readline().decode(errors='ignore').strip()

        ser.write(b'luz\n')
        time.sleep(0.05)
        luz = ser.readline().decode(errors='ignore').strip()

        # Guardar CSV
        with open(CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, temp, luz, img_file])
            f.flush()
        print(f"[GUARDADO] {timestamp} | Temp: {temp} | Luz: {luz}")

    cap.release()
    print("[SERVIDOR] Guardado detenido.")

# ────────────────────────────────────────────────
# Servidor UDP para comandos
# ────────────────────────────────────────────────
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        cmd = msg.decode('utf-8').strip()
        print(f"[UDP] Comando recibido: {cmd}")
        respuesta = enviar_a_arduino(cmd)
        sock.sendto(respuesta.encode('utf-8'), addr)

# ────────────────────────────────────────────────
# Servidor TCP para video
# ────────────────────────────────────────────────
def servidor_video():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    srv.bind((SERVER_IP, TCP_PORT))
    srv.listen(1)
    print(f"[SERVIDOR] TCP Video escuchando en {SERVER_IP}:{TCP_PORT}")

    conn, addr = srv.accept()
    print(f"[SERVIDOR] Cliente de video conectado: {addr}")

    cap = cv2.VideoCapture(0)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            _, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY),70])
            data = enc.tobytes()
            try:
                conn.sendall(struct.pack("Q", len(data)) + data)
            except:
                print("[SERVIDOR] Cliente desconectado.")
                break
    finally:
        cap.release()
        conn.close()

# ────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    servidor_video()
