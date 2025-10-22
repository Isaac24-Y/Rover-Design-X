import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ────────────────────────────────────────────────
# CONFIGURACIÓN
# ────────────────────────────────────────────────
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

# ────────────────────────────────────────────────
# ESTADOS GLOBALES
# ────────────────────────────────────────────────
estado = {
    "camera": False,
    "analog": False,
    "dc": False,
    "servo": False,
    "save": False
}

frame_actual = None
ser = None

# ────────────────────────────────────────────────
# CONEXIÓN CON ARDUINO
# ────────────────────────────────────────────────
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] ✅ Conectado a Arduino en {ARDUINO_PORT}")
except Exception as e:
    print(f"[ERROR] No se pudo conectar al Arduino: {e}")

# ────────────────────────────────────────────────
# ENVÍO DE COMANDOS AL ARDUINO
# ────────────────────────────────────────────────
def enviar_a_arduino(cmd):
    if ser and ser.is_open:
        ser.write((cmd + '\n').encode())
        time.sleep(0.2)
        if ser.in_waiting > 0:
            return ser.readline().decode(errors='ignore').strip()
    return ""

# ────────────────────────────────────────────────
# LECTURA DE SENSORES
# ────────────────────────────────────────────────
def leer_sensores():
    ser.write(b"luz\n")
    time.sleep(0.1)
    luz = ser.readline().decode(errors='ignore').strip()
    return luz

# ────────────────────────────────────────────────
# GUARDAR DATOS EN CSV E IMÁGENES
# ────────────────────────────────────────────────
def guardar_datos():
    os.makedirs("data", exist_ok=True)
    archivo = os.path.join("data", "datos.csv")
    existe = os.path.isfile(archivo)

    with open(archivo, "a", newline="") as f:
        writer = csv.writer(f)
        if not existe:
            writer.writerow(["timestamp", "pixelsU", "pixelsV", "analog_voltage"])

        while estado["save"]:
            if frame_actual is not None:
                # Guardar frame
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join("data", f"frame_{timestamp}.png")
                cv2.imwrite(filename, frame_actual)

                # Guardar datos analógicos
                voltaje = leer_sensores()
                pixelsU, pixelsV = frame_actual.shape[:2]

                writer.writerow([timestamp, pixelsU, pixelsV, voltaje])
                f.flush()
                print(f"[DATA] Guardado {filename} | Voltaje: {voltaje}")

            time.sleep(0.2)

# ────────────────────────────────────────────────
# SERVIDOR UDP (COMANDOS)
# ────────────────────────────────────────────────
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        msg, address = sock.recvfrom(BUFFER_SIZE)
        comando = msg.decode().strip().upper()
        print(f"[UDP] Comando recibido: {comando}")
        respuesta = manejar_comando(comando)
        sock.sendto(respuesta.encode(), address)

# ────────────────────────────────────────────────
# MANEJADOR DE COMANDOS
# ────────────────────────────────────────────────
def manejar_comando(cmd):
    if cmd == "START_CAMERA":
        estado["camera"] = True
        return "✅ Cámara iniciada"
    elif cmd == "STOP_CAMERA":
        estado["camera"] = False
        return "🛑 Cámara detenida"

    elif cmd == "START_ANALOG":
        estado["analog"] = True
        return "✅ Lectura analógica iniciada"
    elif cmd == "STOP_ANALOG":
        estado["analog"] = False
        return "🛑 Lectura analógica detenida"

    elif cmd == "START_DC":
        enviar_a_arduino("dc")
        estado["dc"] = True
        return "✅ Motor DC encendido"
    elif cmd == "STOP_DC":
        enviar_a_arduino("stop")
        estado["dc"] = False
        return "🛑 Motor DC apagado"

    elif cmd == "START_SERVO":
        enviar_a_arduino("servo1")
        estado["servo"] = True
        return "✅ Servo activado"
    elif cmd == "STOP_SERVO":
        enviar_a_arduino("stop")
        estado["servo"] = False
        return "🛑 Servo detenido"

    elif cmd == "SAVE-DATA":
        if not estado["save"]:
            estado["save"] = True
            threading.Thread(target=guardar_datos, daemon=True).start()
            return "💾 Guardando datos..."
        else:
            estado["save"] = False
            return "⛔ Guardado detenido"

    return "⚠️ Comando no reconocido"

# ────────────────────────────────────────────────
# SERVIDOR TCP (VIDEO)
# ────────────────────────────────────────────────
def servidor_video():
    global frame_actual
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, so_
