import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

SERVER_IP = "172.32.214.66"  # Cambiar por la IP de la Raspberry Pi
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

guardando = False
running = True
frame_actual = None

# ─────────────────────────────
# Recibir video con reconexión
# ─────────────────────────────
def recibir_video():
    global frame_actual, running
    while running:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((SERVER_IP, TCP_PORT))
            print(f"[CLIENTE] ✅ Conectado al servidor de video {SERVER_IP}:{TCP_PORT}")
            data = b""
            payload_size = struct.calcsize("Q")
            while running:
                while len(data) < payload_size:
                    packet = client_socket.recv(4096)
                    if not packet:
                        raise ConnectionError("Desconectado del servidor")
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]
                while len(data) < msg_size:
                    data += client_socket.recv(4096)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                frame = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_actual = gray.copy()
                display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(display_frame))
                video_label.config(image=img)
                video_label.image = img
        except Exception as e:
            print(f"[CLIENTE] Error video: {e}")
            time.sleep(2)  # Esperar antes de reconectar

# ─────────────────────────────
# Enviar comandos UDP
# ─────────────────────────────
def enviar_comando(comando):
    global guardando
    try:
        UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        UDPClient.settimeout(3)
        UDPClient.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        data, _ = UDPClient.recvfrom(BUFFER_SIZE)
        respuesta = data.decode('utf-8')
        UDPClient.close()
    except:
        respuesta = "Sin respuesta del servidor."
    response_label.config(text=respuesta)
    if comando == "guardar":
        guardando = True
        status_label.config(text="🟢 Guardando datos...", foreground="green")
    elif comando == "detener":
        guardando = False
        status_label.config(text="🔴 Guardado detenido", foreground="red")

# ─────────────────────────────
# GUI
# ─────────────────────────────
ventana = tk.Tk()
ventana.title("Cliente Raspberry Pi - Arduino")
ventana.geometry("1000x600")
ventana.configure(bg="#f0f0f0")

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

for texto, comando in botones:
    ttk.Button(control_frame, text=texto, width=15,
               command=lambda c=comando: enviar_comando(c)).pack(pady=4)

status_label = tk.Label(control_frame, text="🔴 Guardado detenido", fg="red", bg="#f0f0f0", font=("Arial", 12, "bold"))
status_label.pack(pady=8)

response_label = tk.Label(control_frame, text="", bg="#f0f0f0", font=("Arial", 10))
response_label.pack(pady=5)

def cerrar_cliente():
    global running
    running = False
    ventana.destroy()

threading.Thread(target=recibir_video, daemon=True).start()
ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
