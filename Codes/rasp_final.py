import socket
import threading
import struct
import time
import cv2
import serial
import csv
import os

# ─ CONFIGURACIÓN GENERAL ─
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
BUFFER_SIZE = 1024
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000
TCP_PORT = 50001
SAVE_DIR = "server_data"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "sensores.csv")

# ─ ESTADOS GLOBALES ─
camera_on = False
sensor_on = False
dc_on = False
servo_on = False
saving_data = False
ser = None
ultimo_frame = None

# ─ CONEXIÓN CON ARDUINO ─
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] Conectado a Arduino en {ARDUINO_PORT}")
except Exception as e:
    print("[ERROR] No se pudo conectar al Arduino:", e)

# ─ FUNCIONES AUXILIARES ─
def enviar_a_arduino(comando):
    global ser
    if not ser or not ser.is_open:
        return "Arduino no disponible."
    comando = comando.strip().lower()
    ser.write((comando + '\n').encode())
    time.sleep(0.1)
    if ser.in_waiting > 0:
        return ser.readline().decode(errors='ignore').strip()
    return "Comando enviado."

# ─ GUARDAR DATOS (IMAGEN + CSV) ─
def guardar_datos_server():
    global saving_data, ultimo_frame
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if os.stat(CSV_FILE).st_size == 0:
            writer.writerow(["Tiempo", "Frame", "Temperatura (°C)", "Luz (V)"])
        while saving_data:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            # Guardar último frame
            if ultimo_frame is not None:
                img_path = os.path.join(SAVE_DIR, f"{timestamp}.png")
                cv2.imwrite(img_path, ultimo_frame)
            else:
                img_path = ""
            # Leer Arduino
            temp = enviar_a_arduino("temperatura")
            luz = enviar_a_arduino("luz")
            writer.writerow([timestamp, img_path, temp, luz])
            f.flush()
            time.sleep(0.5)

# ─ HILO VIDEO TCP ─
def video_tcp_thread(conn):
    global camera_on, ultimo_frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while camera_on:
        ret, frame = cap.read()
        if not ret:
            continue
        ultimo_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, enc = cv2.imencode('.jpg', gray, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = enc.tobytes()
        try:
            conn.sendall(struct.pack("Q", len(data)) + data)
        except:
            break
        time.sleep(0.03)  # ~30 fps

    cap.release()
    conn.close()
    print("[SERVER] Cliente TCP desconectado, video detenido en el cliente pero cámara sigue activa.")

# ─ SERVIDOR UDP (COMANDOS) ─
def servidor_udp():
    global camera_on, sensor_on, dc_on, servo_on, saving_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVER] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        cmd = msg.decode().strip().lower()
        print(f"[UDP] Comando recibido: {cmd}")

        # ─ CONTROL CAMARA ─
        if cmd == "start camera":
            camera_on = True
            threading.Thread(target=video_tcp_server, daemon=True).start()
            sock.sendto(b"Camera ON", addr)
        elif cmd == "stop camera":
            camera_on = False
            sock.sendto(b"Camera OFF (ultimo frame mostrado)", addr)

        # ─ CONTROL SENSORES ─
        elif cmd == "start sensor":
            sensor_on = True
            threading.Thread(target=hilo_sensor, args=(sock, addr), daemon=True).start()
            sock.sendto(b"Sensor ON", addr)
        elif cmd == "stop sensor":
            sensor_on = False
            sock.sendto(b"Sensor OFF", addr)

        # ─ MOTOR DC ─
        elif cmd == "start dc":
            dc_on = True
            threading.Thread(target=hilo_dc, daemon=True).start()
            sock.sendto(b"DC Motor ON", addr)
        elif cmd == "stop dc":
            dc_on = False
            sock.sendto(b"DC Motor OFF", addr)

        # ─ SERVOMOTOR ─
        elif cmd == "start servo":
            servo_on = True
            threading.Thread(target=hilo_servo, daemon=True).start()
            sock.sendto(b"Servo ON", addr)
        elif cmd == "stop servo":
            servo_on = False
            sock.sendto(b"Servo OFF", addr)

        # ─ GUARDADO DATOS ─
        elif cmd == "guardar":
            saving_data = True
            threading.Thread(target=guardar_datos_server, daemon=True).start()
            sock.sendto(b"Guardado ON", addr)
        elif cmd == "detener":
            saving_data = False
            sock.sendto(b"Guardado OFF", addr)

        else:
            sock.sendto(b"Comando invalido", addr)

# ─ HILOS DE ACTUADORES Y SENSORES ─
def hilo_dc():
    global dc_on
    while dc_on:
        enviar_a_arduino("dc")
        time.sleep(0.5)

def hilo_servo():
    global servo_on
    while servo_on:
        enviar_a_arduino("servo1")
        time.sleep(0.5)

def hilo_sensor(sock, addr):
    global sensor_on
    while sensor_on:
        temp = enviar_a_arduino("temperatura")
        luz = enviar_a_arduino("luz")
        mensaje = f"TEMP:{temp} °C | LUZ:{luz} V"
        sock.sendto(mensaje.encode(), addr)
        time.sleep(0.5)

# ─ FUNCION TCP SERVIDOR ─
def video_tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    srv.bind((SERVER_IP, TCP_PORT))
    srv.listen(1)
    print(f"[SERVER] TCP Video escuchando en {SERVER_IP}:{TCP_PORT}")
    try:
        conn, addr = srv.accept()
        print(f"[SERVER] Cliente TCP conectado: {addr}")
        video_tcp_thread(conn)
    except:
        print("[SERVER] No se pudo conectar al cliente TCP.")
    finally:
        srv.close()

# ─ MAIN ─
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    print("[SERVER] Servidor UDP iniciado, esperando comandos...")
    while True:
        time.sleep(1)
