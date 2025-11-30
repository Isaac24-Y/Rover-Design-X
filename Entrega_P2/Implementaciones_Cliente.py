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
SERVER_IP = "172.32.142.175"  # IP Raspberry Pi (ajusta si hace falta)
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 4096

# ---------- GLOBALS ----------
guardando = False         # estado de guardado (en servidor)
running = True            # hilo de video activo
pause_video = False       # si True, congela la GUI de video
last_frame_bgr = None     # numpy array BGR último frame recibido
frame_actual_tk = None

# Para control de movimiento mantenido
_move_sending = False
_move_thread = None

# Para velocidad global
speed_var = tk.IntVar(value=80)

# ---------- NETWORKING ----------
def enviar_comando(comando, espera_respuesta=True, timeout=3):
    """
    Envía comando por UDP al servidor y devuelve la respuesta (o mensaje de error).
    Actualiza labels de estado y variables de sensor si la respuesta trae datos.
    """
    global guardando
    respuesta = ""
    try:
        UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        UDPClient.settimeout(timeout)
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

    # Actualizar UI (safe desde hilo principal si existe)
    try:
        response_label.config(text=respuesta)
    except:
        pass

    # Actualizar estado guardado si aplica (si usas comandos save:start/save:stop)
    if comando.lower().startswith("save:start") or comando.lower() == "save:start":
        guardando = True
        try:
            status_label.config(text="🟢 Guardando datos...", foreground="green")
        except:
            pass
    elif comando.lower().startswith("save:stop") or comando.lower() == "save:stop":
        guardando = False
        try:
            status_label.config(text="🔴 Guardado detenido", foreground="red")
        except:
            pass

    # Parseo simple de respuestas con formato TIPO:valor
    # Ejemplos esperados: "TEMP:25.3", "LDR:2.34", "DIST:120cm" o "SENSORS TEMP:25.3 LDR:2.34 DIST:120"
    try:
        up = respuesta.upper()
        if "TEMP" in up:
            # extraer número aproximado
            val = ''.join(ch for ch in up if (ch.isdigit() or ch in ".-C"))
            # fallback: show whole respuesta if parsing falla
            temp_value.set(respuesta if val == "" else val.replace("C","")+" °C")
        if "LDR" in up or "LUX" in up or "V" in up:
            # buscar número
            nums = ''.join(ch for ch in up if (ch.isdigit() or ch in ".-V"))
            luz_value.set(respuesta if nums == "" else nums + " V")
        if "DIST" in up or "CM" in up or "M" in up:
            dist_value.set(respuesta)
    except Exception:
        pass

    return respuesta

# ---------- VIDEO RECEIVER (TCP) ----------
def recibir_video():
    global last_frame_bgr, running, frame_actual_tk
    while running:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5)
            client_socket.connect((SERVER_IP, TCP_PORT))
            client_socket.settimeout(None)
            print(f"[CLIENTE] conectado al servidor de video {SERVER_IP}:{TCP_PORT}")
        except Exception as e:
            print(f"[CLIENTE] No se pudo conectar al servidor de video: {e}. Reintentando en 3s...")
            time.sleep(3)
            continue

        data = b""
        payload_size = struct.calcsize("Q")

        while running:
            try:
                # leer tamaño
                while len(data) < payload_size:
                    packet = client_socket.recv(BUFFER_SIZE)
                    if not packet:
                        print("[CLIENTE] Servidor de video desconectado.")
                        raise ConnectionError("server closed")
                    data += packet

                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                while len(data) < msg_size:
                    packet = client_socket.recv(BUFFER_SIZE)
                    if not packet:
                        raise ConnectionError("server closed during frame")
                    data += packet

                frame_data = data[:msg_size]
                data = data[msg_size:]
                frame = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # BGR

                if frame is None:
                    continue

                last_frame_bgr = frame.copy()

                # Si no está pausado, actualizar GUI (hilo que modifica TK -> use .after)
                if not pause_video:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(frame_rgb)
                    frame_actual_tk = ImageTk.PhotoImage(img)
                    # Actualizar en el hilo principal
                    def update_image():
                        video_label.config(image=frame_actual_tk)
                        video_label.image = frame_actual_tk
                    ventana.after(0, update_image)

            except Exception as e:
                print(f"[ERROR VIDEO LOOP] {e}")
                break

        try:
            client_socket.close()
        except:
            pass
        print("[CLIENTE] Intentando reconectar al video en 1s...")
        time.sleep(1)

    print("[CLIENTE] Hilo de video finalizado.")

# ---------- MOVEMENT CONTROLS (press-hold style) ----------
def _send_move_repeated(cmd, interval=0.12):
    """Envía repetidamente el comando mientras _move_sending sea True."""
    global _move_sending
    while _move_sending:
        enviar_comando(cmd, espera_respuesta=False)
        time.sleep(interval)

def move_button_press(direction):
    global _move_sending, _move_thread
    _move_sending = True
    vel = speed_var.get()
    cmd = f"move:{direction}:{vel}"
    _move_thread = threading.Thread(target=_send_move_repeated, args=(cmd,), daemon=True)
    _move_thread.start()

def move_button_release():
    global _move_sending
    _move_sending = False
    enviar_comando("move:stop", espera_respuesta=False)

# ---------- GUI ACTIONS ----------
def slider_servo_changed(i, val):
    angle = int(float(val))
    enviar_comando(f"servo:{i}:{angle}", espera_respuesta=False)

def dc_left():
    # Si prefieres comandos directos por motor: "move:left:<vel>"
    enviar_comando(f"move:left:{speed_var.get()}", espera_respuesta=False)

def dc_right():
    enviar_comando(f"move:right:{speed_var.get()}", espera_respuesta=False)

def dc_stop():
    enviar_comando("move:stop", espera_respuesta=False)

def cmd_guardar():
    enviar_comando("save:start")

def cmd_detener():
    enviar_comando("save:stop")

def cmd_save_point():
    enviar_comando("save:point")

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

    # Obtener lecturas del servidor (temperatura y luz y distancia)
    try:
        temp_resp = enviar_comando("sensor:temp")
        luz_resp = enviar_comando("sensor:light")
        dist_resp = enviar_comando("sensor:dist")

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
                "lightIntensity", "temperatura(°C)", "luz(V)", "distancia"
            ])
        writer.writerow([fecha_hora, pixelsU, pixelsV, lightIntensity, temp_val, luz_val, dist_resp])

    messagebox.showinfo(
        "Captura completada",
        f"Imagen guardada: {img_path}\n"
        f"Escala de grises: {gray_path}\n"
        f"CSV: {csv_path}\n\n"
        f"Temperatura: {temp_val} °C\n"
        f"Luz: {luz_val} V\n"
        f"Distancia: {dist_resp}"
    )

def emergency_stop():
    enviar_comando("emergency:stop", espera_respuesta=False)
    move_button_release()
    status_label.config(text="⚠️ EMERGENCIA: motores detenidos", foreground="red")

def cerrar_cliente():
    global running, _move_sending
    running = False
    _move_sending = False
    try:
        ventana.destroy()
    except:
        pass

# ---------- INTERVAL SENSOR UPDATE (1s) ----------
def actualizar_sensores():
    if running:
        # Pedimos temperatura, luz y distancia periódicamente
        threading.Thread(target=lambda: enviar_comando("sensor:temp"), daemon=True).start()
        threading.Thread(target=lambda: enviar_comando("sensor:light"), daemon=True).start()
        threading.Thread(target=lambda: enviar_comando("sensor:dist"), daemon=True).start()
        ventana.after(1000, actualizar_sensores)

# ---------- BUILD GUI ----------
ventana = tk.Tk()
ventana.title("Cliente Rover - Raspberry Pi / Arduino")
ventana.geometry("1200x650")
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

# Velocidad global
tk.Label(control_frame, text="Velocidad (0-100)").pack()
speed_slider = tk.Scale(control_frame, from_=0, to=100, orient="horizontal", variable=speed_var, length=200)
speed_slider.set(80)
speed_slider.pack(pady=5)

# Movement buttons (press & hold)
mv_frame = ttk.Frame(control_frame)
mv_frame.pack(pady=6)
btn_forward = ttk.Button(mv_frame, text="Adelante")
btn_forward.grid(row=0, column=1, padx=5, pady=2)
btn_left = ttk.Button(mv_frame, text="Izquierda")
btn_left.grid(row=1, column=0, padx=5, pady=2)
btn_stop = ttk.Button(mv_frame, text="Stop", command=dc_stop)
btn_stop.grid(row=1, column=1, padx=5, pady=2)
btn_right = ttk.Button(mv_frame, text="Derecha")
btn_right.grid(row=1, column=2, padx=5, pady=2)
btn_back = ttk.Button(mv_frame, text="Atrás")
btn_back.grid(row=2, column=1, padx=5, pady=2)

# bind press/release
btn_forward.bind("<ButtonPress>", lambda e: move_button_press("forward"))
btn_forward.bind("<ButtonRelease>", lambda e: move_button_release())
btn_back.bind("<ButtonPress>", lambda e: move_button_press("back"))
btn_back.bind("<ButtonRelease>", lambda e: move_button_release())
btn_left.bind("<ButtonPress>", lambda e: move_button_press("left"))
btn_left.bind("<ButtonRelease>", lambda e: move_button_release())
btn_right.bind("<ButtonPress>", lambda e: move_button_press("right"))
btn_right.bind("<ButtonRelease>", lambda e: move_button_release())

# Servo sliders (4)
tk.Label(control_frame, text="Servos (0-180)").pack(pady=6)
for i in range(1, 5):
    tk.Label(control_frame, text=f"Servo {i}").pack()
    s = tk.Scale(control_frame, from_=0, to=180, orient="horizontal",
                 command=lambda v, idx=i: slider_servo_changed(idx, v), length=200)
    s.set(90)
    s.pack(pady=2)

# Guardado sensores
ttk.Button(control_frame, text="Guardar (server)", command=cmd_guardar).pack(pady=6)
ttk.Button(control_frame, text="Detener guardado", command=cmd_detener).pack(pady=2)
ttk.Button(control_frame, text="Guardar punto (1 lectura)", command=cmd_save_point).pack(pady=2)

# Camera controls
cam_frame = ttk.Frame(control_frame)
cam_frame.pack(pady=10)
btn_pause = ttk.Button(cam_frame, text="Pausar", command=toggle_pause)
btn_pause.grid(row=0, column=0, padx=3)
ttk.Button(cam_frame, text="Capturar imagen (guardar U,V)", command=capture_image_and_uv).grid(row=0, column=1, padx=3)

# Emergency & exit
ttk.Button(control_frame, text="EMERGENCIA STOP", command=emergency_stop).pack(pady=6)
ttk.Button(control_frame, text="Salir", command=cerrar_cliente).pack(pady=6)

# Status
status_label = tk.Label(control_frame, text="🔴 Guardado detenido", fg="red", bg="#f0f0f0", font=("Arial", 12, "bold"))
status_label.pack(pady=8)

separator = ttk.Separator(control_frame, orient='horizontal')
separator.pack(fill='x', pady=8)

# Lecturas de sensores
tk.Label(control_frame, text="Lecturas de Sensores", font=("Arial", 14, "bold")).pack(pady=5)
temp_value = tk.StringVar(value="-- °C")
luz_value = tk.StringVar(value="-- V")
dist_value = tk.StringVar(value="-- cm")
tk.Label(control_frame, text="Temperatura:").pack()
tk.Label(control_frame, textvariable=temp_value, font=("Arial", 12, "bold"), fg="blue").pack(pady=3)
tk.Label(control_frame, text="Luz:").pack()
tk.Label(control_frame, textvariable=luz_value, font=("Arial", 12, "bold"), fg="orange").pack(pady=3)
tk.Label(control_frame, text="Distancia:").pack()
tk.Label(control_frame, textvariable=dist_value, font=("Arial", 12, "bold"), fg="green").pack(pady=3)

separator2 = ttk.Separator(control_frame, orient='horizontal')
separator2.pack(fill='x', pady=8)

response_label = tk.Label(control_frame, text="", bg="#f0f0f0", font=("Arial", 10))
response_label.pack(pady=5)

# ---------- Keyboard bindings (WASD) ----------
def key_pressed(event):
    k = event.keysym.lower()
    if k == 'w': move_button_press("forward")
    elif k == 's': move_button_press("back")
    elif k == 'a': move_button_press("left")
    elif k == 'd': move_button_press("right")

def key_released(event):
    k = event.keysym.lower()
    if k in ('w','s','a','d'):
        move_button_release()

ventana.bind("<KeyPress>", key_pressed)
ventana.bind("<KeyRelease>", key_released)
ventana.focus_set()

# ---------- Start threads ----------
threading.Thread(target=recibir_video, daemon=True).start()
ventana.after(1000, actualizar_sensores)

ventana.protocol("WM_DELETE_WINDOW", cerrar_cliente)
ventana.mainloop()
