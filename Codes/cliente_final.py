import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

# ────────────────────────────────────────────────
# CONFIGURACIÓN
# ────────────────────────────────────────────────
SERVER_IP = "172.32.214.66"  # ← Cambia por la IP real de la Raspberry Pi
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

# ────────────────────────────────────────────────
# VARIABLES GLOBALES
# ────────────────────────────────────────────────
running = True
recibiendo_video = False
frame_actual = None
client_socket = None

# ────────────────────────────────────────────────
# FUNCIÓN: Recibir video del servidor
# ────────────────────────────────────────────────
def recibir_video():
    global frame_actual, running, recibiendo_video, client_socket

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, TCP_PORT))
        print(f"[CLIENTE] ✅ Conectado al servidor de video en {SERVER_IP}:{TCP_PORT}")
    except Exception as e:
        messagebox.showerror("Error de conexión", f"No se pudo conectar al servidor de video:\n{e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")
    recibiendo_video = True

    while running and recibiendo_video:
        try:
            # Recibir tamaño del frame
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("[CLIENTE] ⚠️ Servidor de video desconectado.")
                    running = False
                    break
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            # Recibir el frame completo
            while len(data) < msg_size:
                data += client_socket.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Decodificar el frame
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # ← Color, no gris
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convertir a formato para Tkinter
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.config(image=imgtk)

        except Exception as e:
            print(f"[ERROR VIDEO LOOP] {e}")
            break

    if client_socket:
        client_socket.close()
    print("[CLIENTE] Video cerrado.")


# ────────────────────────────────────────────────
# FUNCIÓN: Enviar comandos UDP al servidor
# ────────────────────────────────────────────────
def enviar_comando(comando):
    global recibiendo_video

    try:
        UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        UDPClient.settimeout(3)
        UDPClient.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        data, _ = UDPClient.recvfrom(BUFFER_SIZE)
        respuesta = data.decode('utf-8')
        UDPClient.close()
    except socket.timeout:
        respuesta = "⚠️ Sin respuesta del servidor."
    except Exception as e:
        respuesta = f"❌ Error: {e}"

    response_label.config(text=f"{respuesta}", foreground="blue")

    # Acciones según comando
    if comando == "START_CAMERA":
        if not recibiendo_video:
            threading.Thread(target=recibir_video, daemon=True).start()
            recibiendo_video = True

    elif comando == "STOP_CAMERA":
        recibiendo_video = False

    # Estado visual
    if "✅" in respuesta:
        status_label.config(text=respuesta, foreground="green")
    elif "🛑" in respuesta or "⛔" in respuesta:
        status_label.config(text=respuesta, foreground="red")
    else:
        status_label.config(text="ℹ️ " + respuesta, foreground="gray")


# ────────────────────────────────────────────────
# FUNCIÓN: Cerrar cliente
# ────────────────────────────────────────────────
def cerrar_cliente():
    global running, recibiendo_video
    running = False
    recibiendo_video = False
    ventana.destroy()


# ────────────────────────────────────────────────
# INTERFAZ GRÁFICA
# ────────────────────────────────────────────────
ventana = tk.Tk()
ventana.title("Cliente Raspberry Pi - Arduino")
ventana.geometry("1050x600")
ventana.configure(bg="#f5f5f5")

# ─ Panel principal ─
main_frame = ttk.Frame(ventana)
main_frame.pack(fill="both", expand=True, padx=10, pady=10)

# ─ Panel de video ─
video_frame = ttk.Frame(main_frame)
video_frame.pack(side="left", fill="both", expand=True)

video_label = tk.Label(video_frame, bg="black", width=640, height=480)
video_label.pack(padx=10, pady=10)

# ─ Panel de control ─
control_frame = ttk.Frame(main_frame)
control_frame.pack(side="right", fill="y", padx=10, pady=10)

tk.Label(control_frame, text="Panel de Control", font=("Arial", 16, "bold")).pack(pady=10)

botones = [
    ("Iniciar Cámara", "START_CAMERA"),
    ("Detener Cámara", "STOP_CAMERA"),
    ("Iniciar Lectura", "START_ANALOG"),
    ("Detener Lectura", "STOP_ANALOG"),
    ("Iniciar DC Motor", "START_DC"),
    ("Detener DC Motor", "STOP_DC"),
    ("Iniciar Servo", "START_SERVO"),
    ("Detener Servo", "STOP_SERVO"),
    ("Guardar Datos", "SAVE-DATA")
]

boton_frame = ttk.Frame(control_frame)
boton_frame.pack(pady=10)

for texto, comando in botones:
    b = ttk.Button(boton_frame, text=texto, width=20,
                   command=lambda c=comando: enviar_comando(c))
    b.pack(pady=4)

status_label = tk.Label(control_frame, text="Sistema inactivo",
                        bg="#f5f5f5", fg="gray", font=("Arial", 12, "bold"))
status_label.pack(pady=10)

separator = ttk.Separator(control_frame, orient='horizontal')
separator.pack(fill='x', pady=10)

response_label = tk.Label(control_frame, text="", bg="#f5f5f5", font=("Arial", 10))
response_label.pack(pady=5)

ttk.Button(control_frame, text="Salir", command=cerrar_cliente).pack(pady=15)

ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
