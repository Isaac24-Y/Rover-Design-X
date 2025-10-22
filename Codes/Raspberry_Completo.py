import socket, threading, struct, cv2, numpy as np, serial, time, csv, os

# Configuración de Raspberry Pi
SERVER_IP = '0.0.0.0'
UDP_PORT = 50000   # Comandos
TCP_PORT = 50001   # Video
BUFFER_SIZE = 1024
SAVE_DIR = "data"
os.makedirs(SAVE_DIR, exist_ok=True)

# Inicialización Arduino
arduino_port = '/dev/ttyACM0'
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

# Estados
camera_on = False
sensor_on = False
dc_on = False
servo_on = False
saving_data = False

# ────────────────────────────────────────────────
# Funciones auxiliares
# ────────────────────────────────────────────────

def enviar_a_arduino(comando):
    """Enviar comando al Arduino y leer respuesta"""
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
            break
        time.sleep(0.03)  # ~30 fps
    cap.release()
    conn.close()

def hilo_sensor(sock, client_addr):
    """Enviar voltaje LDR continuamente"""
    global sensor_on
    while sensor_on:
        valor = enviar_a_arduino("ldr")  # Arduino debe enviar valor analógico LDR
        mensaje = f"LDR:{valor}"
        sock.sendto(mensaje.encode(), client_addr)
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

def guardar_datos():
    """Guardar imagen y voltaje"""
    global saving_data
    cap = cv2.VideoCapture(0)
    csv_file = os.path.join(SAVE_DIR, "datos.csv")
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        if os.stat(csv_file).st_size == 0:
            writer.writerow(["Tiempo","Voltaje","ArchivoImagen"])
        while saving_data:
            ret, frame = cap.read()
            if not ret: continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            img_name = os.path.join(SAVE_DIR, f"{timestamp}.png")
            cv2.imwrite(img_name, gray)
            voltaje = enviar_a_arduino("ldr")
            writer.writerow([timestamp, voltaje, img_name])
            f.flush()
            time.sleep(0.5)
    cap.release()

# ────────────────────────────────────────────────
# Servidor UDP: comandos
# ────────────────────────────────────────────────
def servidor_udp():
    global camera_on, sensor_on, dc_on, servo_on, saving_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVER] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        msg, addr = sock.recvfrom(BUFFER_SIZE)
        cmd = msg.decode().strip().lower()
        print(f"[CMD] {cmd} de {addr}")

        if cmd == "start camera":
            camera_on = True
            threading.Thread(target=video_tcp_thread, daemon=True).start()
            sock.sendto(b"Camera ON", addr)

        elif cmd == "stop camera":
            camera_on = False
            sock.sendto(b"Camera OFF", addr)

        elif cmd == "start sensor":
            sensor_on = True
            threading.Thread(target=hilo_sensor, args=(sock, addr), daemon=True).start()
            sock.sendto(b"Sensor ON", addr)

        elif cmd == "stop sensor":
            sensor_on = False
            sock.sendto(b"Sensor OFF", addr)

        elif cmd == "start dc":
            dc_on = True
            threading.Thread(target=hilo_dc, daemon=True).start()
            sock.sendto(b"DC Motor ON", addr)

        elif cmd == "stop dc":
            dc_on = False
            sock.sendto(b"DC Motor OFF", addr)

        elif cmd == "start servo":
            servo_on = True
            threading.Thread(target=hilo_servo, daemon=True).start()
            sock.sendto(b"Servo ON", addr)

        elif cmd == "stop servo":
            servo_on = False
            sock.sendto(b"Servo OFF", addr)

        elif cmd == "save-data":
            saving_data = True
            threading.Thread(target=guardar_datos, daemon=True).start()
            sock.sendto(b"Saving Data ON", addr)

        elif cmd == "stop save":
            saving_data = False
            sock.sendto(b"Saving Data OFF", addr)

        else:
            sock.sendto(b"Comando invalido", addr)

# ────────────────────────────────────────────────
# Servidor TCP: video
# ────────────────────────────────────────────────
def video_tcp_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    srv.bind((SERVER_IP, TCP_PORT))
    srv.listen(1)
    print(f"[SERVER] TCP video escuchando en {SERVER_IP}:{TCP_PORT}")
    conn, addr = srv.accept()
    print(f"[SERVER] Cliente de video conectado {addr}")
    hilo_video(conn)

# ────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    while True:
        time.sleep(1)
