import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ────────────────────────────────────────────────
# Configuración general
# ────────────────────────────────────────────────
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 1024
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001

guardar_datos = False
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
# Enviar comandos al Arduino
# ────────────────────────────────────────────────
def enviar_a_arduino(comando):
    global guardar_datos
    comando = comando.strip().lower()

    if not ser or not ser.is_open:
        print("[ERROR] Arduino no conectado.")
        return "Arduino no disponible."

    if comando == "guardar":
        if not guardar_datos:
            guardar_datos = True
            threading.Thread(target=guardar_datos_sensores, daemon=True).start()
            return "Guardado de datos iniciado."
        return "Ya se está guardando."

    elif comando == "detener":
        guardar_datos = False
        return "Guardado detenido."

    elif comando in ['servo1', 'temperatura', 'luz', 'dc', 'derecha', 'izquierda', 'stop']:
        ser.write((comando + '\n').encode())
        time.sleep(0.2)
        if ser.in_waiting > 0:
            return ser.readline().decode(errors='ignore').strip()
        return "Comando enviado al Arduino."

    else:
        return "Comando no válido."


# ────────────────────────────────────────────────
# Guardar datos cada 0.5s
# ────────────────────────────────────────────────
def guardar_datos_sensores():
    global guardar_datos
    archivo = "sensores.csv"
    existe = os.path.isfile(archivo)

    with open(archivo, mode='a', newline='') as f:
        writer = csv.writer(f)
        if not existe:
            writer.writerow(["Tiempo", "Temperatura (°C)", "Luz (V)"])

        while guardar_datos:
            ser.write(b'temperatura\n')
            time.sleep(0.1)
            temp = ser.readline().decode(errors='ignore').strip()

            ser.write(b'luz\n')
            time.sleep(0.1)
            luz = ser.readline().decode(errors='ignore').strip()

            tiempo = time.strftime("%Y-%m-%d %H:%M:%S")
            if temp and luz:
                writer.writerow([tiempo, temp, luz])
                f.flush()
                print(f"[GUARDADO] {tiempo} | {temp} | {luz}")

            time.sleep(0.5)
    print("[SERVIDOR] Guardado detenido correctamente.")


# ────────────────────────────────────────────────
# Servidor UDP (comandos)
# ────────────────────────────────────────────────
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        msg, address = sock.recvfrom(BUFFER_SIZE)
        comando = msg.decode('utf-8').strip()
        print(f"[UDP] Comando recibido: {comando}")
        respuesta = enviar_a_arduino(comando)
        sock.sendto(respuesta.encode('utf-8'), address)


# ────────────────────────────────────────────────
# Servidor TCP (video)
# ────────────────────────────────────────────────
def servidor_video():
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
            ret, frame = cap.read()
            if not ret:
                break
            _, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            data = encimg.tobytes()
            conn.sendall(struct.pack("Q", len(data)) + data)
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
