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
SERVER_IP = "172.32.214.66"  # Cambia por la IP de tu Raspberry Pi
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

# ────────────────────────────────────────────────
# VARIABLES GLOBALES
# ────────────────────────────────────────────────
guardando = False
running = True
frame_actual = None

# ────────────────────────────────────────────────
# FUNCIÓN: Recibir video desde Raspberry
# ────────────────────────────────────────────────
def recibir_video():
    global frame_actual, running
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, TCP_PORT))
        print(f"[CLIENTE] ✅ Conectado al servidor de video {SERVER_IP}:{TCP_PORT}")
    except Exception as e:
        messagebox.showerror("Error de conexión", f"No se pudo conectar al servidor de video:\n{e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")

    while running:
        try:
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

            while len(data) < msg_size:
                data += client_socket.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            if guardando:
                cv2.putText(frame, "GUARDANDO DATOS...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_actual = ImageTk.PhotoImage(Image.fromarray(frame))
            video_label.config(image=frame_actual)
            video_label.image = frame_actual
        except Exception as e:
            print(f"[ERROR VIDEO LOOP] {e}")
            break

    client_socket.close()
    print("[CLIENTE] Video cerrado.")


# ────────────────────────────────────────────────
# FUNCIÓN: Enviar comandos UDP
# ────────────────────────────────────────────────
def enviar_comando(comando):
    global guardando

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

    # Mostrar respuesta
    response_label.config(text=f"{respuesta}", foreground="blue")

    # Actualizar estado según comando
    if comando == "guardar":
        guardando = True
        status_label.config(text="🟢 Guardando datos...", foreground="green")
    elif comando == "detener":
        guardando = False
        status_label.config(text="🔴 Guardado detenido", foreground="red")

    # Mostrar lecturas si aplica
    if "°C" in respuesta:
        temp_value.set(respuesta.split(":")[-1].strip())
    elif "V" in respuesta:
        luz_value.set(respuesta.split(":")[-1].strip())


# ────────────────────────────────────────────────
# FUNCIÓN: Actualizar sensores automáticamente
# ────────────────────────────────────────────────
def actualizar_sensores():
    if running:
        threading.Thread(target=lambda: enviar_comando("temperatura"), daemon=True).start()
        threading.Thread(target=lambda: enviar_comando("luz"), daemon=True).start()
        ventana.after(1000, actualizar_sensores)  # Repite cada 1 segundo


# ────────────────────────────────────────────────
# FUNCIÓN: Cerrar ventana
# ────────────────────────────────────────────────
def cerrar_cliente():
    global running
    running = False
    ventana.destroy()


# ────────────────────────────────────────────────
# INTERFAZ GRÁFICA
# ────────────────────────────────────────────────
ventana = tk.Tk()
ventana.title("Cliente Raspberry Pi - Arduino")
ventana.geometry("1000x600")
ventana.configure(bg="#f0f0f0")

# ─ Dividir ventana principal ─
main_frame = ttk.Frame(ventana)
main_frame.pack(fill="both", expand=True)

# ─ Panel de video (izquierda) ─
video_frame = ttk.Frame(main_frame)
video_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

video_label = tk.Label(video_frame, bg="black", width=640, height=480)
video_label.pack(expand=True)

# ─ Panel de control (derecha) ─
control_frame = ttk.Frame(main_frame)
control_frame.pack(side="right", fill="y", padx=15, pady=10)

title_label = tk.Label(control_frame, text="Panel de Control", font=("Arial", 16, "bold"))
title_label.pack(pady=10)

# ─ Botones de control ─
botones = [
    ("Servo 1", "servo1"),
    ("DC Motor", "dc"),
    ("Derecha", "derecha"),
    ("Izquierda", "izquierda"),
    ("Guardar", "guardar"),
    ("Detener", "detener")
]

boton_frame = ttk.Frame(control_frame)
boton_frame.pack(pady=10)

for texto, comando in botones:
    b = ttk.Button(boton_frame, text=texto, width=15,
                   command=lambda c=comando: enviar_comando(c))
    b.pack(pady=4)

# ─ Estado ─
status_label = tk.Label(control_frame, text="🔴 Guardado detenido",
                        fg="red", bg="#f0f0f0", font=("Arial", 12, "bold"))
status_label.pack(pady=8)

# ─ Lecturas de sensores ─
separator = ttk.Separator(control_frame, orient='horizontal')
separator.pack(fill='x', pady=10)

tk.Label(control_frame, text="Lecturas de Sensores", font=("Arial", 14, "bold")).pack(pady=5)

temp_value = tk.StringVar(value="-- °C")
luz_value = tk.StringVar(value="-- V")

tk.Label(control_frame, text="Temperatura:", font=("Arial", 12)).pack()
tk.Label(control_frame, textvariable=temp_value, font=("Arial", 12, "bold"), fg="blue").pack(pady=3)

tk.Label(control_frame, text="Luz:", font=("Arial", 12)).pack()
tk.Label(control_frame, textvariable=luz_value, font=("Arial", 12, "bold"), fg="orange").pack(pady=3)

# ─ Respuesta del servidor ─
separator2 = ttk.Separator(control_frame, orient='horizontal')
separator2.pack(fill='x', pady=10)

response_label = tk.Label(control_frame, text="", bg="#f0f0f0", font=("Arial", 10))
response_label.pack(pady=5)

# ─ Botón de salida ─
ttk.Button(control_frame, text="Salir", command=cerrar_cliente).pack(pady=15)

# ─ Iniciar hilo de video y actualización automática ─
threading.Thread(target=recibir_video, daemon=True).start()
ventana.after(1000, actualizar_sensores)

# ─ Ejecutar GUI ─
ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
