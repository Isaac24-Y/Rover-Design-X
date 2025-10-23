# client.py
import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import time
import os
import csv

# ---------- CONFIG ----------
SERVER_IP = "172.32.214.66"  # IP Raspberry Pi
SERVER_IP = "192.168.1.10"  # IP Raspberry Pi
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 4096

# ---------- GLOBALS ----------
guardando = False         # estado de guardado (en servidor)
running = True            # hilo de video activo
pause_video = False       # si True, no actualiza la GUI con nuevos frames (congela)
last_frame_bgr = None     # numpy array BGR último frame recibido
frame_actual_tk = None

# ---------- VIDEO RECEIVER (TCP) ----------
def recibir_video():
    global last_frame_bgr, running, frame_actual_tk
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, TCP_PORT))
        print(f"[CLIENTE] conectado al servidor de video {SERVER_IP}:{TCP_PORT}")
    except Exception as e:
        messagebox.showerror("Error de conexión", f"No se pudo conectar al servidor de video:\n{e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")

    while running:
        try:
            # leer tamaño
            while len(data) < payload_size:
                packet = client_socket.recv(BUFFER_SIZE)
                if not packet:
                    print("[CLIENTE] Servidor de video desconectado.")
                    running = False
                    break
                data += packet
            if not running:
                break

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                packet = client_socket.recv(BUFFER_SIZE)
                if not packet:
                    running = False
                    break
                data += packet
            if not running:
                break

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # BGR

            if frame is None:
                continue

            last_frame_bgr = frame.copy()

            # Si no está pausado, actualizar GUI
            if not pause_video:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                frame_actual_tk = ImageTk.PhotoImage(img)
                video_label.config(image=frame_actual_tk)
                video_label.image = frame_actual_tk

        except Exception as e:
            print(f"[ERROR VIDEO LOOP] {e}")
            break

    client_socket.close()
    print("[CLIENTE] Video cerrado.")

# ---------- UDP COMMANDS ----------
def enviar_comando(comando, espera_respuesta=True):
    """
    Envía comando por UDP al servidor y devuelve la respuesta (o mensaje de error).
    """
    global guardando
    try:
        UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        UDPClient.settimeout(3)
        UDPClient.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        if espera_respuesta:
            data, _ = UDPClient.recvfrom(BUFFER_SIZE)
            respuesta = data.decode('utf-8')
        else:
            respuesta = "Enviado"
        UDPClient.close()
    except socket.timeout:
        respuesta = "⚠️ Sin respuesta del servidor."
    except Exception as e:
        respuesta = f"❌ Error: {e}"

    response_label.config(text=respuesta)

    # Actualizar estado guardado si aplica
    if comando == "guardar":
        guardando = True
        status_label.config(text="🟢 Guardando datos...", foreground="green")
    elif comando == "detener":
        guardando = False
        status_label.config(text="🔴 Guardado detenido", foreground="red")

    # parsear lecturas si vienen en la respuesta
    if "°C" in respuesta:
        try:
            temp_value.set(respuesta.split()[-2] + " " + respuesta.split()[-1])
        except:
            temp_value.set(respuesta)
    elif "V" in respuesta:
        try:
            luz_value.set(respuesta.split()[-2] + " " + respuesta.split()[-1])
        except:
            luz_value.set(respuesta)

    return respuesta

# ---------- INTERVAL SENSOR UPDATE (1s) ----------
def actualizar_sensores():
    if running:
        threading.Thread(target=lambda: enviar_comando("temperatura"), daemon=True).start()
        threading.Thread(target=lambda: enviar_comando("luz"), daemon=True).start()
        ventana.after(1000, actualizar_sensores)

# ---------- GUI ACTIONS ----------
def slider_servo_changed(val):
    angle = int(float(val))
    enviar_comando(f"servo:{angle}")

def dc_left():
    enviar_comando("dc:left")

def dc_right():
    enviar_comando("dc:right")

def dc_stop():
    enviar_comando("dc:stop")

def cmd_guardar():
    enviar_comando("guardar")

def cmd_detener():
    enviar_comando("detener")

def cmd_save_point():
    # pedir al servidor que haga una sola lectura y la guarde
    enviar_comando("save_point")

def toggle_pause():
    global pause_video
    pause_video = not pause_video
    if pause_video:
        btn_pause.config(text="Reanudar")
    else:
        btn_pause.config(text="Pausar")

def capture_image_and_uv():
    """
    Captura el frame actual, guarda la imagen (color y gris),
    y agrega los datos (fecha, dimensiones, intensidad, temperatura y luz)
    en un único CSV acumulativo.
    """
    global last_frame_bgr
    if last_frame_bgr is None:
        messagebox.showwarning("Captura", "No hay frame disponible para capturar.")
        return

    # Crear carpeta de capturas si no existe
    carpeta = "capturas"
    os.makedirs(carpeta, exist_ok=True)

    # Obtener timestamp actual
    fecha_hora = time.strftime("%Y-%m-%d %H:%M:%S")
    timestamp = time.strftime("%Y%m%d-%H%M%S")

    # Guardar imagen color y gris
    img_path = os.path.join(carpeta, f"captura_{timestamp}.jpg")
    gray_path = os.path.join(carpeta, f"gray_{timestamp}.jpg")

    cv2.imwrite(img_path, last_frame_bgr)
    gray = cv2.cvtColor(last_frame_bgr, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(gray_path, gray)

    # Obtener dimensiones e intensidad
    pixelsV, pixelsU = gray.shape  # alto, ancho
    lightIntensity = pixelsU * pixelsV

    # Obtener lecturas del servidor (temperatura y luz)
    try:
        temp_resp = enviar_comando("temperatura")
        luz_resp = enviar_comando("luz")

        temp_val = ''.join(ch for ch in temp_resp if ch.isdigit() or ch in ".-")
        luz_val = ''.join(ch for ch in luz_resp if ch.isdigit() or ch in ".-")
    except Exception as e:
        temp_val = "N/A"
        luz_val = "N/A"
        print(f"[ERROR] Lectura sensores en captura: {e}")

    # Archivo CSV único
    csv_path = os.path.join(carpeta, "datos_capturas.csv")

    # Si no existe, escribir encabezado
    write_header = not os.path.exists(csv_path)

    with open(csv_path, mode="a", newline="") as file:
        writer = csv.writer(file)
        if write_header:
            writer.writerow([
                "fecha_hora", "pixelsU", "pixelsV",
                "lightIntensity", "temperatura(°C)", "luz(V)"
            ])
        writer.writerow([fecha_hora, pixelsU, pixelsV, lightIntensity, temp_val, luz_val])

    messagebox.showinfo(
        "Captura completada",
        f"Imagen guardada: {img_path}\n"
        f"Escala de grises: {gray_path}\n"
        f"CSV: {csv_path}\n\n"
        f"Temperatura: {temp_val} °C\n"
        f"Luz: {luz_val} V"
    )
    
def cerrar_cliente():
    global running
    running = False
    ventana.destroy()

# ---------- BUILD GUI ----------
ventana = tk.Tk()
ventana.title("Cliente Raspberry Pi - Arduino")
ventana.geometry("1100x600")
ventana.configure(bg="#f0f0f0")

main_frame = ttk.Frame(ventana)
main_frame.pack(fill="both", expand=True)

# Video frame (left)
video_frame = ttk.Frame(main_frame)
video_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
video_label = tk.Label(video_frame, bg="black", width=640, height=480)
video_label.pack(expand=True)

# Control panel (right)
control_frame = ttk.Frame(main_frame)
control_frame.pack(side="right", fill="y", padx=15, pady=10)

title_label = tk.Label(control_frame, text="Panel de Control", font=("Arial", 16, "bold"))
title_label.pack(pady=10)

# Servo slider
tk.Label(control_frame, text="Servo (0-180)").pack()
servo_slider = tk.Scale(control_frame, from_=0, to=180, orient="horizontal", length=200,
                        command=slider_servo_changed)
servo_slider.set(90)
servo_slider.pack(pady=5)

# DC motor buttons
dc_frame = ttk.Frame(control_frame)
dc_frame.pack(pady=5)
ttk.Button(dc_frame, text="Izquierda", command=dc_left).grid(row=0, column=0, padx=3)
ttk.Button(dc_frame, text="Detener", command=dc_stop).grid(row=0, column=1, padx=3)
ttk.Button(dc_frame, text="Derecha", command=dc_right).grid(row=0, column=2, padx=3)

# Guardado sensores
ttk.Button(control_frame, text="Guardar (cada 0.5s)", command=cmd_guardar).pack(pady=6)
ttk.Button(control_frame, text="Detener guardado", command=cmd_detener).pack(pady=2)
ttk.Button(control_frame, text="Guardar punto (1 lectura)", command=cmd_save_point).pack(pady=2)

# Camera controls
cam_frame = ttk.Frame(control_frame)
cam_frame.pack(pady=10)
btn_pause = ttk.Button(cam_frame, text="Pausar", command=toggle_pause)
btn_pause.grid(row=0, column=0, padx=3)
ttk.Button(cam_frame, text="Capturar imagen (guardar U,V)", command=capture_image_and_uv).grid(row=0, column=1, padx=3)

# Status
status_label = tk.Label(control_frame, text="🔴 Guardado detenido", fg="red", bg="#f0f0f0", font=("Arial", 12, "bold"))
status_label.pack(pady=8)

separator = ttk.Separator(control_frame, orient='horizontal')
separator.pack(fill='x', pady=8)

# Lecturas de sensores
tk.Label(control_frame, text="Lecturas de Sensores", font=("Arial", 14, "bold")).pack(pady=5)
temp_value = tk.StringVar(value="-- °C")
luz_value = tk.StringVar(value="-- V")
tk.Label(control_frame, text="Temperatura:").pack()
tk.Label(control_frame, textvariable=temp_value, font=("Arial", 12, "bold"), fg="blue").pack(pady=3)
tk.Label(control_frame, text="Luz:").pack()
tk.Label(control_frame, textvariable=luz_value, font=("Arial", 12, "bold"), fg="orange").pack(pady=3)

separator2 = ttk.Separator(control_frame, orient='horizontal')
separator2.pack(fill='x', pady=8)

response_label = tk.Label(control_frame, text="", bg="#f0f0f0", font=("Arial", 10))
response_label.pack(pady=5)

ttk.Button(control_frame, text="Salir", command=cerrar_cliente).pack(pady=8)

# Start threads
threading.Thread(target=recibir_video, daemon=True).start()
ventana.after(1000, actualizar_sensores)

ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
