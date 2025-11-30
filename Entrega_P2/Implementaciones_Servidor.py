import socket
import threading
import cv2
import struct
import time
import random
import os
import csv

# ============================
# CONFIG
# ============================
UDP_PORT = 50000
TCP_PORT = 50001
VIDEO_SOURCE = 0  # 0 = webcam local (cambia si usas CSI o IP cam)
BUFFER_SIZE = 4096

# ============================
# ESTADOS GENERALES
# ============================
guardando = False
csv_lock = threading.Lock()

# ============================
# FUNCIONES DE SENSORES (simuladas)
# Cambia estas si conectas sensores reales
# ============================
def get_temperature():
    return round(22 + random.random() * 5, 2)  # 22–27 °C

def get_light():
    return round(0.5 + random.random() * 1.5, 2)  # 0.5–2.0 V

def get_distance():
    return f"{random.randint(20, 150)} cm"

# ============================
# ACCIONES DE MOVIMIENTO
# ============================
def handle_move(cmd):
    print("[MOVIMIENTO]", cmd)
    return "OK"

# ============================
# SERVOS
# ============================
def handle_servo(i, ang):
    print(f"[SERVO] Servo {i} → {ang}°")
    return "OK"

# ============================
# GUARDADO CSV AUTOMÁTICO
# ============================
def save_csv_temp_light_dist():
    global guardando

    os.makedirs("server_logs", exist_ok=True)
    path = "server_logs/datos_sensores.csv"

    first_write = not os.path.exists(path)

    with csv_lock:
        with open(path, "a", newline="") as f:
            writer = csv.writer(f)
            if first_write:
                writer.writerow(["timestamp", "temp", "light", "dist"])

            writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S"),
                get_temperature(),
                get_light(),
                get_distance()
            ])

# ============================
# UDP SERVER
# ============================
def udp_server():
    global guardando

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", UDP_PORT))
    print(f"[UDP] Servidor escuchando en puerto {UDP_PORT}")

    while True:
        data, addr = s.recvfrom(BUFFER_SIZE)
        msg = data.decode("utf-8").strip()
        print(f"[UDP] Comando recibido de {addr}: {msg}")

        # ----- movimiento -----
        if msg.startswith("move:"):
            s.sendto(handle_move(msg).encode(), addr)

        # ----- servo control -----
        elif msg.startswith("servo:"):
            _, num, ang = msg.split(":")
            r = handle_servo(int(num), int(ang))
            s.sendto(r.encode(), addr)

        # ----- sensores -----
        elif msg == "sensor:temp":
            temp = get_temperature()
            s.sendto(f"TEMP:{temp}".encode(), addr)

        elif msg == "sensor:light":
            light = get_light()
            s.sendto(f"LDR:{light}".encode(), addr)

        elif msg == "sensor:dist":
            dist = get_distance()
            s.sendto(f"DIST:{dist}".encode(), addr)

        # ----- guardar en CSV -----
        elif msg == "save:start":
            guardando = True
            s.sendto(b"SAVE:STARTED", addr)

        elif msg == "save:stop":
            guardando = False
            s.sendto(b"SAVE:STOPPED", addr)

        elif msg == "save:point":
            save_csv_temp_light_dist()
            s.sendto(b"SAVE:POINT_OK", addr)

        # ----- emergencia -----
        elif msg == "emergency:stop":
            print("[EMERGENCIA] STOP")
            s.sendto(b"EMERGENCY:OK", addr)

        else:
            s.sendto(b"UNKNOWN_CMD", addr)

        # Si el guardado automático está activo:
        if guardando:
            save_csv_temp_light_dist()

# ============================
# TCP VIDEO SERVER
# ============================
def tcp_video_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("", TCP_PORT))
    server_socket.listen(1)

    print(f"[TCP] Servidor de video en puerto {TCP_PORT}")

    while True:
        try:
            client_socket, addr = server_socket.accept()
            print(f"[TCP] Cliente conectado: {addr}")

            cap = cv2.VideoCapture(VIDEO_SOURCE)

            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                _, jpeg = cv2.imencode(".jpg", frame)
                data = jpeg.tobytes()

                # Enviar tamaño
                client_socket.sendall(struct.pack("Q", len(data)))
                # Enviar bytes
                client_socket.sendall(data)

                time.sleep(0.03)  # ~30 FPS

        except Exception as e:
            print("[TCP] Cliente desconectado:", e)
        finally:
            try: client_socket.close()
            except: pass

# ============================
# MAIN
# ============================
if __name__ == "__main__":
    print("=== SERVIDOR ROVER INICIADO ===")

    threading.Thread(target=udp_server, daemon=True).start()
    threading.Thread(target=tcp_video_server, daemon=True).start()

    while True:
        time.sleep(1)
