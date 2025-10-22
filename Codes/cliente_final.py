import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import os
import time
import csv

# ─ CONFIGURACIÓN ─
SERVER_IP = "172.32.214.66"
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024
SAVE_DIR = "client_data_sync"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "sensores.csv")

# ─ VARIABLES GLOBALES ─
guardando = False
running = True
frame_actual = None
ultimo_frame_rgb = None  # Para mostrar si el video se detiene

# ─ FUNCIÓN VIDEO ─
def recibir_video():
    global frame_actual, running, ultimo_frame_rgb
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((SERVER_IP, TCP_PORT))
    except Exception as e:
        print(f"[ERROR] No se pudo conectar al servidor de video: {e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")
    while running:
        try:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet: raise ConnectionError
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                packet = client_socket.recv(4096)
                if not packet: raise ConnectionError
                data += packet

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            frame_actual = frame.copy()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ultimo_frame_rgb = frame_rgb.copy()
            imgtk = ImageTk.PhotoImage(Image.fromarray(frame_rgb))
            video_label.config(image=imgtk)
            video_label.image = imgtk

        except:
            # Si falla la conexión o video se detiene, muestra último frame
            if ultimo_frame_rgb is not None:
                imgtk = ImageTk.PhotoImage(Image.fromarray(ultimo_frame_rgb))
                video_label.config(image=imgtk)
                video_label.image = imgtk
            break

    client_socket.close()
    print("[CLIENTE] Video cerrado o detenido, mostrando último frame.")

# ─ FUNCIÓN COMANDOS ─
def enviar_comando(comando):
    global guardando
    UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDPClient.settimeout(3)
    try:
        UDPClient.sendto(comando.encode(), (SERVER_IP, UDP_PORT))
        data, _ = UDPClient.recvfrom(BUFFER_SIZE)
        respuesta = data.decode()
    except:
        respuesta = "Sin respuesta del servidor"
    response_label.config(text=respuesta)
    if comando == "guardar":
        guardando = True
        status_label.config(text="🟢 Guardando datos...", foreground="green")
        threading.Thread(target=guardar_frame_csv, daemon=True).start()
    elif comando == "detener":
        guardando = False
        status_label.config(text="🔴 Guardado detenido", foreground="red")
    UDPClient.close()

# ─ FUNCIÓN GUARDADO FRAME + CSV ─
def guardar_frame_csv():
    global guardando, frame_actual
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if os.stat(CSV_FILE).st_size == 0:
            writer.writerow(["Tiempo", "Frame"])
        while guardando:
            if frame_actual is not None:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                img_path = os.path.join(SAVE_DIR, f"{timestamp}.png")
                cv2.imwrite(img_path, frame_actual)
                writer.writerow([timestamp, img_path])
                f.flush()
            time.sleep(0.5)

# ─ CERRAR GUI ─
def cerrar_cliente():
    global running
    running = False
    ventana.destroy()

# ─ INTERFAZ ─
ventana = tk.Tk()
ventana.title("Cliente RPi")
ventana.geometry("1000x600")

main_frame = ttk.Frame(ventana)
main_frame.pack(fill="both", expand=True)

video_frame = ttk.Frame(main_frame)
video_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
video_label = tk.Label(video_frame, bg="black", width=640, height=480)
video_label.pack(expand=True)

control_frame = ttk.Frame(main_frame)
control_frame.pack(side="right", fill="y", padx=15, pady=10)
ttk.Label(control_frame, text="Panel de Control", font=("Arial", 16, "bold")).pack(pady=10)

botones = [("Servo 1","servo1"),("DC Motor","dc"),("Derecha","derecha"),
           ("Izquierda","izquierda"),("Guardar","guardar"),("Detener","detener")]

for txt, cmd in botones:
    ttk.Button(control_frame, text=txt, width=15, command=lambda c=cmd: enviar_comando(c)).pack(pady=4)

status_label = tk.Label(control_frame, text="🔴 Guardado detenido", fg="red", font=("Arial", 12, "bold"))
status_label.pack(pady=8)
response_label = tk.Label(control_frame, text="", font=("Arial", 10))
response_label.pack(pady=5)
ttk.Button(control_frame, text="Salir", command=cerrar_cliente).pack(pady=15)

threading.Thread(target=recibir_video, daemon=True).start()
ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
