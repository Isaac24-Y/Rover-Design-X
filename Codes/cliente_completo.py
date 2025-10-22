import socket
import threading
import struct
import cv2
import numpy as np
import csv
import os
import time

# ────────────────────────────────────────────────
# Configuración del servidor
# ────────────────────────────────────────────────
SERVER_IP = "172.32.214.66"  # IP de la Raspberry Pi
SERVER_IP = '192.168.1.10'  # IP del servidor (Raspberry Pi)
UDP_PORT = 50000             # Comandos y sensor
TCP_PORT = 50001             # Video
BUFFER_SIZE = 1024

SAVE_DIR = "client_data"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "sensores.csv")

# Estados
camera_on = False
sensor_on = False
guardar = False

# ────────────────────────────────────────────────
# Hilo para recibir video TCP
# ────────────────────────────────────────────────
def recibir_video():
    global camera_on
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((SERVER_IP, TCP_PORT))
        data = b""
        payload_size = struct.calcsize("Q")
        camera_on = True
        while camera_on:
            while len(data) < payload_size:
                packet = s.recv(4096)
                if not packet:
                    camera_on = False
                    break
                data += packet
            if not camera_on: break

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += s.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)

            # Mostrar texto si estamos guardando
            if guardar:
                cv2.putText(frame, "GUARDANDO DATOS...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Video Raspberry Pi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                camera_on = False
                break
        s.close()
        cv2.destroyAllWindows()
    except Exception as e:
        print("[ERROR VIDEO]", e)

# ────────────────────────────────────────────────
# Hilo para recibir datos del sensor UDP
# ────────────────────────────────────────────────
def recibir_sensor():
    global sensor_on, guardar
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT+1))  # Puerto local para recibir datos del sensor
    sensor_on = True

    # Crear CSV si no existe
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Tiempo", "Voltaje LDR", "Temperatura"])

    while sensor_on:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            mensaje = data.decode()
            print(f"[SENSOR] {mensaje}")

            if guardar and mensaje.startswith("LDR:"):
                try:
                    ldr_val = mensaje.split(",")[0].split(":")[1]
                    temp_val = mensaje.split(",")[1].split(":")[1]
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    with open(CSV_FILE, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, ldr_val, temp_val])
                except:
                    pass
        except Exception as e:
            print(f"[ERROR SENSOR] {e}")

# ────────────────────────────────────────────────
# Hilo para enviar comandos UDP
# ────────────────────────────────────────────────
def enviar_comandos():
    global camera_on, sensor_on, guardar
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3)

    print("Escribe comandos:")
    print("start/stop camera, start/stop sensor, start/stop dc, start/stop servo, save-data, stop save, q para salir")
    while True:
        cmd = input("Comando: ").strip().lower()
        if cmd == 'q':
            camera_on = False
            sensor_on = False
            guardar = False
            break

        sock.sendto(cmd.encode(), (SERVER_IP, UDP_PORT))
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            print(f"[SERVIDOR] {data.decode()}")
        except:
            print("[SERVIDOR] Sin respuesta")

        if cmd == "save-data":
            guardar = True
            print("[CLIENTE] 🟢 Guardado de datos iniciado")
        elif cmd == "stop save":
            guardar = False
            print("[CLIENTE] 🔴 Guardado de datos detenido")

    sock.close()

# ────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────
if __name__ == "__main__":
    # Hilos para video y sensor
    threading.Thread(target=recibir_video, daemon=True).start()
    threading.Thread(target=recibir_sensor, daemon=True).start()
    # Enviar comandos desde consola
    enviar_comandos()
    print("[CLIENTE] Finalizado")
