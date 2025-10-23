# server.py
import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ---------- CONFIG ----------
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 4096
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001

# ---------- GLOBALS ----------
guardar_datos = False
ser = None
csv_lock = threading.Lock()

# ---------- OPEN SERIAL ----------
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] Conectado a Arduino en {ARDUINO_PORT}")
except Exception as e:
    print("[ERROR] No se pudo conectar al Arduino:", e)
    ser = None

# ---------- HELPERS ----------
def enviar_a_arduino_simple(comando, espera_respuesta=True, timeout=1.0):
    """
    Escribe el comando al Arduino (terminado en newline) y lee línea de respuesta si espera_respuesta.
    Retorna string respuesta o mensaje de error.
    """
    if ser is None or not ser.is_open:
        return "Arduino no disponible."

    try:
        ser.reset_input_buffer()
        ser.write((comando + '\n').encode())
        if espera_respuesta:
            deadline = time.time() + timeout
            while time.time() < deadline:
                if ser.in_waiting > 0:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line:
                        return line
                time.sleep(0.05)
            return "Sin respuesta del Arduino."
        else:
            return "Comando enviado."
    except Exception as e:
        return f"Error serie: {e}"

# ---------- GUARDAR PERIÓDICO ----------
def guardar_datos_sensores_thread():
    global guardar_datos
    archivo = "sensores.csv"
    existe = os.path.isfile(archivo)
    with csv_lock:
        with open(archivo, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not existe:
                writer.writerow(["Tiempo", "Temperatura (°C)", "Luz (V)"])
    # loop fuera del lock
    while guardar_datos:
        temp_resp = enviar_a_arduino_simple("temperatura")
        luz_resp = enviar_a_arduino_simple("luz")
        tiempo = time.strftime("%Y-%m-%d %H:%M:%S")
        with csv_lock:
            with open(archivo, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([tiempo, temp_resp, luz_resp])
                f.flush()
        print(f"[GUARDADO] {tiempo} | {temp_resp} | {luz_resp}")
        time.sleep(0.5)
    print("[SERVIDOR] Guardado periódico detenido.")

# ---------- SAVE POINT (single reading) ----------
def save_point_once():
    archivo = "sensores.csv"
    existe = os.path.isfile(archivo)
    temp_resp = enviar_a_arduino_simple("temperatura")
    luz_resp = enviar_a_arduino_simple("luz")
    tiempo = time.strftime("%Y-%m-%d %H:%M:%S")
    with csv_lock:
        with open(archivo, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not existe:
                writer.writerow(["Tiempo", "Temperatura (°C)", "Luz (V)"])
            writer.writerow([tiempo, temp_resp, luz_resp])
            f.flush()
    return f"Guardado punto: {tiempo} | {temp_resp} | {luz_resp}"

# ---------- UDP SERVER (commands) ----------
def servidor_udp():
    global guardar_datos
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")
    while True:
        msg, address = sock.recvfrom(BUFFER_SIZE)
        comando = msg.decode('utf-8').strip()
        print(f"[UDP] Comando recibido: {comando}")
        respuesta = ""

        # comandos: guardar, detener, temperatura, luz, servo:90, dc:left, save_point, etc.
        if comando == "guardar":
            if not guardar_datos:
                guardar_datos = True
                threading.Thread(target=guardar_datos_sensores_thread, daemon=True).start()
                respuesta = "Guardado de datos iniciado."
            else:
                respuesta = "Ya se está guardando."
        elif comando == "detener":
            guardar_datos = False
            respuesta = "Guardado detenido."
        elif comando == "temperatura":
            respuesta = enviar_a_arduino_simple("temperatura")
        elif comando == "luz":
            respuesta = enviar_a_arduino_simple("luz")
        elif comando.startswith("servo:"):
            # enviar string tal cual al Arduino
            respuesta = enviar_a_arduino_simple(comando)
        elif comando.startswith("dc:"):
            respuesta = enviar_a_arduino_simple(comando)
        elif comando == "save_point":
            respuesta = save_point_once()
        else:
            respuesta = enviar_a_arduino_simple(comando)

        sock.sendto(respuesta.encode('utf-8'), address)

# ---------- TCP VIDEO SERVER ----------
def servidor_video():
    srv_vid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_vid.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_vid.bind((SERVER_IP, TCP_PORT))
    srv_vid.listen(1)
    print(f"[SERVIDOR] Video escuchando en {SERVER_IP}:{TCP_PORT}")

    while True:
        conn, addr = srv_vid.accept()
        print(f"[SERVIDOR] Cliente de video conectado: {addr}")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[ERROR] No se pudo acceder a la cámara.")
            conn.close()
            continue

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                _, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                data = encimg.tobytes()
                # primero largo (Q) luego datos
                conn.sendall(struct.pack("Q", len(data)) + data)
                # pequeño sleep para no saturar (ajustable)
                time.sleep(0.02)
        except Exception as e:
            print(f"[SERVIDOR] Cliente de video desconectado: {e}")
        finally:
            cap.release()
            conn.close()

# ---------- MAIN ----------
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    servidor_video()
