import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from PIL import Image, ImageTk
import os
import time
import csv

# ────────────────────────────────────────────────
# Configuración
# ────────────────────────────────────────────────
SERVER_IP = "172.32.214.66"
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

SAVE_DIR = "client_data"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "sensores.csv")

# ─ Variables globales
frame_actual = None
guardando = False
running = True

# ────────────────────────────────────────────────
# Recibir video TCP
# ────────────────────────────────────────────────
def recibir_video():
    global frame_actual, running, guardando
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SERVER_IP, TCP_PORT))
        data = b""
        payload_size = struct.calcsize("Q")
    except Exception as e:
        print("[ERROR] No se pudo conectar al servidor de video:", e)
        return

    while running:
        try:
            while len(data) < payload_size:
                packet = sock.recv(4096)
                if not packet:
                    running = False
                    break
                data += packet
            if not running: break

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += sock.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            # Guardar frame si está activado
            if guardando:
                timestamp = time.strftime("%Y%m%d_%H%M%S_%f")
                img_file = os.path.join(SAVE_DIR, f"{timestamp}.png")
                cv2.imwrite(img_file, frame)
                # Guardar CSV
                with open(CSV_FILE, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, img_file])
                    f.flush()

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_actual_img = ImageTk.PhotoImage(Image.fromarray(frame_rgb))
            frame_actual = frame_actual_img
            video_label.config(image=frame_actual)
            video_label.image = frame_actual

        except Exception as e:
            print("[ERROR VIDEO LOOP]", e)
            break
    sock.close()

# ────────────────────────────────────────────────
# Enviar comando UDP
# ────────────────────────────────────────────────
def enviar_comando(cmd):
    global guardando
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3)
        sock.sendto(cmd.encode(), (SERVER_IP, UDP_PORT))
        data, _ = sock.recvfrom(BUFFER_SIZE)
        resp = data.decode()
        sock.close()
        response_label.config(text=resp)
    except:
        response_label.config(text="Sin respuesta del servidor.")

    if cmd.lower() == "guardar":
        guardando = True
        status_label.config(text="🟢 Guardando datos", fg="green")
    elif cmd.lower() == "detener":
        guardando = False
        status_label.config(text="🔴 Guardado detenido", fg="red")

# ────────────────────────────────────────────────
# GUI
# ────────────────────────────────────────────────
root = tk.Tk()
root.title("Cliente Optimizado")
root.geometry("800x600")

video_label = tk.Label(root, bg="black", width=640, height=480)
video_label.pack()

frame_control = tk.Frame(root)
frame_control.pack(pady=10)

botones = [
    ("Servo 1", "servo1"),
    ("DC Motor", "dc"),
    ("Guardar", "guardar"),
    ("Detener", "detener"),
    ("Stop Motores", "stop")
]

for txt, cmd in botones:
    tk.Button(frame_control, text=txt, width=15,
              command=lambda c=cmd: enviar_comando(c)).pack(side="left", padx=5)

status_label = tk.Label(root, text="🔴 Guardado detenido", fg="red")
status_label.pack(pady=5)

response_label = tk.Label(root, text="", fg="blue")
response_label.pack(pady=5)

# ─ Hilos
threading.Thread(target=recibir_video, daemon=True).start()

# ─ Cierre
def cerrar():
    global running
    running = False
    root.destroy()
root.protocol("WM_DELETE_WINDOW", cerrar)

root.mainloop()
