import socket
import threading
import pickle
import struct
import time
import cv2
import traceback
import serial
import csv
import os

# ────────────────────────────────────────────────
# Configuración general
# ────────────────────────────────────────────────
arduino_port = '/dev/ttyACM0'  # Puerto del Arduino
baud_rate = 9600
bufferSize = 1024

serverIP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001

guardar_datos = False   # Flag para control de guardado
ser = None              # Puerto serial

# ────────────────────────────────────────────────
# Inicializar conexión con Arduino
# ────────────────────────────────────────────────
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] Conectado a Arduino en {arduino_port}")
except Exception as e:
    print("[ERROR] No se pudo conectar al Arduino:", e)

# ────────────────────────────────────────────────
# Función para enviar comandos al Arduino
# ────────────────────────────────────────────────
def enviar_a_arduino(comando):
    global guardar_datos

    comando = comando.strip().lower()
    if not ser:
        print("[ERROR] Arduino no conectado.")
        return

    if comando == "guardar":
        if not guardar_datos:
            guardar_datos = True
            hilo_guardar = threading.Thread(target=guardar_datos_sensores, daemon=True)
            hilo_guardar.start()
            print("[SERVIDOR] Iniciando guardado de datos cada 0.5 s...")
        else:
            print("[SERVIDOR] Ya se está guardando.")
        return

    elif comando == "detener":
        guardar_datos = False
        print("[SERVIDOR] Guardado detenido.")
        return

    elif comando in ['servo1', 'temperatura', 'luz', 'dc', 'stop']:
        ser.write((comando + '\n').encode())
        time.sleep(0.1)
        if ser.in_waiting > 0:
            respuesta = ser.readline().decode(errors='ignore').strip()
            print("[ARDUINO]", respuesta)
            return respuesta
    else:
        print("[SERVIDOR] Comando no reconocido:", comando)
        return "Comando no válido."


# ────────────────────────────────────────────────
# Función para guardar datos del Arduino cada 0.5s
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
            # Pedir datos al Arduino
            ser.write(b'temperatura\n')
            time.sleep(0.1)
            temp = ser.readline().decode(errors='ignore').strip()

            ser.write(b'luz\n')
            time.sleep(0.1)
            luz = ser.readline().decode(errors='ignore').strip()

            tiempo = time.strftime("%Y-%m-%d %H:%M:%S")

            # Guardar si hay datos válidos
            if temp and luz:
                writer.writerow([tiempo, temp, luz])
                f.flush()
                print(f"[GUARDADO] {tiempo} | {temp} | {luz}")

            time.sleep(0.5)

    print("[SERVIDOR] Guardado detenido correctamente.")


# ────────────────────────────────────────────────
# Servidor UDP para recibir comandos
# ────────────────────────────────────────────────
def servidor_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((serverIP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {serverIP}:{UDP_PORT}")

    while True:
        msg, address = sock.recvfrom(bufferSize)
        comando = msg.decode('utf-8').strip()
        print(f"[UDP] Comando recibido: {comando}")

        respuesta = enviar_a_arduino(comando)
        if respuesta is None:
            respuesta = "Comando procesado."

        sock.sendto(respuesta.encode('utf-8'), address)


# ────────────────────────────────────────────────
# Servidor TCP para enviar video
# ────────────────────────────────────────────────
def servidor_video():
    srv_vid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_vid.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_vid.bind((serverIP, TCP_PORT))
    srv_vid.listen(1)
    print(f"[SERVIDOR] Video escuchando en {serverIP}:{TCP_PORT}")

    conn, addr = srv_vid.accept()
    print(f"[SERVIDOR] Cliente de video conectado: {addr}")

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        _, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = encimg.tobytes()
        try:
            conn.sendall(struct.pack("Q", len(data)) + data)
        except:
            print("[SERVIDOR] Cliente de video desconectado.")
            break

    cap.release()
    conn.close()


# ────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────
if __name__ == "__main__":
    hilo_udp = threading.Thread(target=servidor_udp, daemon=True)
    hilo_udp.start()

    servidor_video()
