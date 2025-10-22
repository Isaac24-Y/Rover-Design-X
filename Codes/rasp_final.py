import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ────────────────────────────────────────────────
# Configuración
# ────────────────────────────────────────────────
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 1024
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001

guardar_datos = False
camera_on = False
ser = None

# ────────────────────────────────────────────────
# Conexión con Arduino
# ────────────────────────────────────────────────
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] ✅ Conectado a Arduino en {ARDUINO_PORT}")
except Exception as e:
    print("[ERROR] No se pudo conectar al Arduino:", e)

# ────────────────────────────────────────────────
# Funciones auxiliares
# ────────────────────────────────────────────────
def enviar_a_arduino(comando):
    global guardar_datos
    comando = comando.strip().lower()
    if not ser or not ser.is_open:
        return "Arduino no disponible."

    if comando == "save-data":
        if not guardar_datos:
            guardar_datos = True
            threading.Thread(target=guardar_datos_sensores, daemon=True).start()
            return "✅ Guardado iniciado"
        else:
            return "⚠️ Guardado ya activo"
    elif comando == "stop-save":
        guardar_datos = False
        return "🛑 Guardado detenido"
    elif comando in ["start_dc", "stop_dc", "start_servo", "stop_servo", "start_analog", "stop_analog"]:
        ser.write((comando + '\n').encode())
        time.sleep(0.1)
        if ser.in_waiting > 0:
            return ser.readline().decode(errors='ignore').strip()
        return "✅ Comando enviado"
    return "⛔ Comando no válido"

def guardar_datos_sensores():
    global guardar_datos
    archivo_csv = "datos.csv"
    if not os.path.exists("frames"):
        os.makedirs("frames")
    existe = os.path.isfile(archivo_csv)
    with open(archivo_csv, mode='a', newline='') as f:
        writer = csv.writer(f)
        if not existe:
            writer.writerow(["Timestamp", "AnalogValue", "Frame"])
        while guardar_datos:
            timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
            analog_value = 0
            if ser:
                ser.write(b"start_analog\n")
                time.sleep(0.05)
                if ser.in_waiting > 0:
                    analog_value = ser.readline().decode(errors='ignore').strip()
            # Guardar frame
            if camera_on and latest_frame is not None:
                frame_filename = f"frames/frame_{timestamp}.jpg"
                cv2.imwrite(frame_filename, latest_frame)
                writer.writerow([timestamp, analog_value, frame_filename])
                f.flush()
            time.sleep(0.5)
    print("[SERVIDOR] Guardado detenido correctamente.")

# ────────────────────────────────────────────────
# Servidor UDP
# ────────────────────────────────────────────────
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")
    global camera_on
    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        cmd = msg.decode('utf-8').strip().upper()
        respuesta = ""
        if cmd == "START_CAMERA":
            camera_on = True
            respuesta = "✅ Cámara encendida"
        elif cmd == "STOP_CAMERA":
            camera_on = False
            respuesta = "🛑 Cámara detenida"
        else:
            respuesta = enviar_a_arduino(cmd)
        sock.sendto(respuesta.encode(), addr)

# ────────────────────────────────────────────────
# Servidor TCP (video)
# ────────────────────────────────────────────────
latest_frame = None
def servidor_video():
    global latest_frame
    srv_vid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_vid.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_vid.bind((SERVER_IP, TCP_PORT))
    srv_vid.listen(1)
    print(f"[SERVIDOR] Video escuchando en {SERVER_IP}:{TCP_PORT}")

    conn, addr = srv_vid.accept()
    print(f"[SERVIDOR] Cliente de video conectado: {addr}")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] No se pudo acceder a la cámara.")
        conn.close()
        return

    try:
        while True:
            if camera_on:
                ret, frame = cap.read()
                if not ret:
                    continue
                latest_frame = frame.copy()
                _, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                data = enc.tobytes()
                conn.sendall(struct.pack("Q", len(data)) + data)
            else:
                time.sleep(0.1)
    except Exception as e:
        print(f"[SERVIDOR] Cliente de video desconectado: {e}")
    finally:
        cap.release()
        conn.close()

# ────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    servidor_video()
