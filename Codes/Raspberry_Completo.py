import socket, threading, struct, cv2, serial, time, csv, os

# ────────────────────────────────────────────────
# Configuración
# ────────────────────────────────────────────────
arduino_port = '/dev/ttyACM0'
baud_rate = 9600

SERVER_IP = '0.0.0.0'
UDP_PORT = 50000   # comandos
TCP_PORT = 50001   # video
BUFFER_SIZE = 1024
SAVE_DIR = "data"
os.makedirs(SAVE_DIR, exist_ok=True)

# Estados
camera_on = False
sensor_on = False
dc_on = False
servo_on = False
guardar_datos_flag = False

# Conexión con Arduino
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] Conectado a Arduino en {arduino_port}")
except Exception as e:
    ser = None
    print("[ERROR] No se pudo conectar al Arduino:", e)


# ────────────────────────────────────────────────
# Funciones auxiliares
# ────────────────────────────────────────────────
def enviar_a_arduino(comando):
    if not ser:
        return ""
    ser.write((comando + '\n').encode())
    time.sleep(0.05)
    if ser.in_waiting > 0:
        return ser.readline().decode(errors='ignore').strip()
    return ""

def hilo_video(conn):
    global camera_on
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    while camera_on:
        ret, frame = cap.read()
        if not ret: continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, enc = cv2.imencode('.jpg', gray, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = enc.tobytes()
        try:
            conn.sendall(struct.pack("Q", len(data)) + data)
        except:
            print("[SERVIDOR] Cliente de video desconectado.")
            break
        time.sleep(0.03)  # ~30 fps
    cap.release()
    conn.close()

def hilo_sensor(sock, client_addr):
    global sensor_on, guardar_datos_flag
    csv_file = os.path.join(SAVE_DIR, "sensores.csv")
    if not os.path.exists(csv_file):
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Tiempo", "Temperatura (°C)", "Luz (V)"])
    while sensor_on:
        temp = enviar_a_arduino("temp")
        luz = enviar_a_arduino("luz")
        msg = f"LDR:{luz},TEMP:{temp}"
        try:
            sock.sendto(msg.encode(), client_addr)
        except:
            break
        if guardar_datos_flag and temp and luz:
            tiempo = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([tiempo, temp, luz])
        time.sleep(0.5)

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


# ────────────────────────────────────────────────
# Servidor UDP
# ────────────────────────────────────────────────
def servidor_udp():
    global camera_on, sensor_on, dc_on, servo_on, guardar_datos_flag
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVIDOR] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        try:
            msg, addr = sock.recvfrom(BUFFER_SIZE)
            cmd = msg.decode().strip().lower()
            print(f"[CMD] {cmd} de {addr}")

            respuesta = "Comando procesado."
            if cmd == "start camera":
                camera_on = True
                threading.Thread(target=video_tcp_thread, daemon=True).start()
                respuesta = "Camera ON"

            elif cmd == "stop camera":
                camera_on = False
                respuesta = "Camera OFF"

            elif cmd == "start sensor":
                sensor_on = True
                threading.Thread(target=hilo_sensor, args=(sock, addr), daemon=True).start()
                respuesta = "Sensor ON"

            elif cmd == "stop sensor":
                sensor_on = False
                respuesta = "Sensor OFF"

            elif cmd == "start dc":
                dc_on = True
                threading.Thread(target=hilo_dc, daemon=True).start()
                respuesta = "DC Motor ON"

            elif cmd == "stop dc":
                dc_on = False
                respuesta = "DC Motor OFF"

            elif cmd == "start servo":
                servo_on = True
                threading.Thread(target=hilo_servo, daemon=True).start()
                respuesta = "Servo ON"

            elif cmd == "stop servo":
                servo_on = False
                respuesta = "Servo OFF"

            elif cmd == "save-data":
                guardar_datos_flag = True
                respuesta = "Guardado de datos ON"

            elif cmd == "stop save":
                guardar_datos_flag = False
                respuesta = "Guardado de datos OFF"

            sock.sendto(respuesta.encode(), addr)
        except Exception as e:
            print(f"[ERROR UDP] {e}")

# ────────────────────────────────────────────────
# Servidor TCP video
# ────────────────────────────────────────────────
def video_tcp_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT))
    srv.listen(1)
    print(f"[SERVIDOR] TCP video escuchando en {SERVER_IP}:{TCP_PORT}")
    conn, addr = srv.accept()
    print(f"[SERVIDOR] Cliente de video conectado {addr}")
    hilo_video(conn)

# ────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    while True:
        time.sleep(1)
