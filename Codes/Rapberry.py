import socket
import threading
import struct
import time
import cv2
import serial

# ─ Puertos y configuración ─
COMMAND_PORT = 50000  # UDP
VIDEO_PORT   = 50001  # TCP
SERVER_IP   = '0.0.0.0'
BUFFER_SIZE = 1024

arduino_port = '/dev/ttyACM0'
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)
print(f"Conectado a Arduino en {arduino_port}")

# ─ Función para enviar comandos al Arduino ─
def arduino(men):
    comando = men.lower()
    if comando in ['servo1', 'temperatura', 'luz', 'dc']:
        ser.write(comando.encode())
        print(f"Comando '{comando}' enviado.")
        time.sleep(0.1)
        if ser.in_waiting > 0:
            respuesta = ser.readline().decode().strip()
            print(f"Respuesta de Arduino: {respuesta}")
    else:
        print("Comando no válido.")

# ─ Función para recibir mensajes UDP ─
def mensaje_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, COMMAND_PORT))
    print(f"[SERVER] Escuchando comandos UDP en {SERVER_IP}:{COMMAND_PORT}")
    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        msg = msg.decode('utf-8').strip()
        print(f"[COMANDO] {msg} de {addr}")
        sock.sendto(b"OK", addr)
        arduino(msg)

# ─ Función para enviar video TCP ─
def camara_tcp():
    srv_vid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_vid.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_vid.bind((SERVER_IP, VIDEO_PORT))
    srv_vid.listen(1)
    print(f"[SERVER] Video TCP en {SERVER_IP}:{VIDEO_PORT}")

    conn_vid, addr_v = srv_vid.accept()
    print(f"[SERVER] Cliente de video conectado: {addr_v}")

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    time.sleep(2)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        _, encimg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = encimg.tobytes()
        try:
            conn_vid.sendall(struct.pack("Q", len(data)) + data)
        except:
            print("[SERVER] Cliente de video desconectado.")
            break

    cap.release()
    conn_vid.close()
    srv_vid.close()

# ─ Hilos ─
if __name__ == "__main__":
    threading.Thread(target=mensaje_udp, daemon=True).start()
    threading.Thread(target=camara_tcp, daemon=True).start()

    # Mantener el main vivo
    while True:
        time.sleep(1)
