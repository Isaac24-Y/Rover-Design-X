#!/usr/bin/env python3
import socket, threading, struct, cv2, serial, time, csv, os, json

# -------- CONFIGURACIÓN ----------
arduino_port = '/dev/ttyACM0'
baud_rate = 9600

SERVER_IP = '0.0.0.0'
UDP_PORT = 50000   # comandos + respuestas status
TCP_PORT = 50001   # video (TCP)
BUFFER_SIZE = 4096
SAVE_DIR = "data"
os.makedirs(SAVE_DIR, exist_ok=True)

# -------- ESTADOS GLOBALES -----------
camera_on = False
sensor_on = False
dc_on = False
servo_on = False
guardar_datos_flag = False

# Mantener referencia al último cliente que pidió sensor (para enviarle UDP)
last_sensor_client = None

# Conexión Arduino
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"[SERVIDOR] Conectado a Arduino en {arduino_port}")
except Exception as e:
    ser = None
    print("[ERROR] No se pudo conectar al Arduino:", e)

serial_lock = threading.Lock()

# -------- FUNCIONES AUXILIARES -----------
def enviar_a_arduino(comando, timeout=0.1):
    """Enviar comando al Arduino y leer respuesta (thread-safe)."""
    if not ser:
        return ""
    with serial_lock:
        try:
            ser.reset_input_buffer()
        except:
            pass
        ser.write((comando + '\n').encode())
        time.sleep(timeout)
        if ser.in_waiting > 0:
            try:
                return ser.readline().decode(errors='ignore').strip()
            except:
                return ""
    return ""

def get_status_dict():
    return {
        "camera_on": camera_on,
        "sensor_on": sensor_on,
        "dc_on": dc_on,
        "servo_on": servo_on,
        "saving_data": guardar_datos_flag
    }

# -------- HILO VIDEO (envía color) -----------
def hilo_video(conn):
    global camera_on
    print("[VIDEO] Comenzando transmisión (color).")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[VIDEO] No se pudo abrir la cámara.")
        conn.close()
        return

    # Opcionales: ajustar resolución
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 20)

    try:
        while camera_on:
            ret, frame = cap.read()
            if not ret:
                continue
            # Enviamos color (BGR) encoded jpeg
            _, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            data = enc.tobytes()
            try:
                conn.sendall(struct.pack("Q", len(data)) + data)
            except Exception as e:
                print("[VIDEO] Error enviando frame:", e)
                break
            # Pequeña pausa (control de fps)
            time.sleep(0.03)
    finally:
        cap.release()
        try:
            conn.close()
        except:
            pass
        print("[VIDEO] Transmisión finalizada.")

# -------- HILO SENSOR (envía LDR/TEMP por UDP al cliente solicitante) -----------
def hilo_sensor(sock, client_addr):
    """Envia datos LDR y Temp cada 0.5s al client_addr vía UDP mientras sensor_on=True"""
    global sensor_on, guardar_datos_flag
    csv_file = os.path.join(SAVE_DIR, "sensores_server.csv")
    if not os.path.exists(csv_file):
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Tiempo", "Voltaje LDR", "Temperatura (V)"])
    print(f"[SENSOR] Enviando datos a {client_addr}")
    while sensor_on:
        # pedir lecturas al Arduino
        ldr = enviar_a_arduino("luz", timeout=0.08) or ""
        temp = enviar_a_arduino("temp", timeout=0.08) or ""
        # construir mensaje y enviar
        msg = json.dumps({"ldr": ldr, "temp": temp})
        try:
            sock.sendto(msg.encode(), client_addr)
        except Exception as e:
            print("[SENSOR] Error enviando a cliente:", e)
            break
        # guardar en server CSV si flag activo
        if guardar_datos_flag and ldr != "" and temp != "":
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([ts, ldr, temp])
        time.sleep(0.5)
    print("[SENSOR] Hilo sensor finalizado.")

# -------- HILOS ACTUADORES -----------
def hilo_dc():
    global dc_on
    print("[DC] DC thread started")
    while dc_on:
        enviar_a_arduino("dc", timeout=0.05)
        time.sleep(0.5)
    print("[DC] DC thread stopped")

def hilo_servo():
    global servo_on
    print("[SERVO] Servo thread started")
    while servo_on:
        enviar_a_arduino("servo1", timeout=0.05)
        time.sleep(0.5)
    print("[SERVO] Servo thread stopped")

# -------- SERVIDOR UDP (comandos) -----------
def servidor_udp():
    global camera_on, sensor_on, dc_on, servo_on, guardar_datos_flag, last_sensor_client
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, UDP_PORT))
    print(f"[SERVER] UDP escuchando en {SERVER_IP}:{UDP_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            cmd = data.decode('utf-8').strip().lower()
            print(f"[CMD] '{cmd}' desde {addr}")

            # Default respuesta
            respuesta = {"ok": True, "command": cmd, "status": get_status_dict()}

            if cmd in ("start camera", "start_camera", "camera start"):
                if not camera_on:
                    camera_on = True
                    threading.Thread(target=video_tcp_thread, daemon=True).start()
                respuesta["status"]["camera_on"] = camera_on

            elif cmd in ("stop camera", "stop_camera", "camera stop"):
                camera_on = False
                respuesta["status"]["camera_on"] = camera_on

            elif cmd in ("start sensor", "start_sensor", "sensor start"):
                if not sensor_on:
                    sensor_on = True
                    last_sensor_client = addr
                    threading.Thread(target=hilo_sensor, args=(sock, addr), daemon=True).start()
                respuesta["status"]["sensor_on"] = sensor_on

            elif cmd in ("stop sensor", "stop_sensor", "sensor stop"):
                sensor_on = False
                respuesta["status"]["sensor_on"] = sensor_on

            elif cmd in ("start dc", "start_dc", "dc start"):
                if not dc_on:
                    dc_on = True
                    threading.Thread(target=hilo_dc, daemon=True).start()
                respuesta["status"]["dc_on"] = dc_on

            elif cmd in ("stop dc", "stop_dc", "dc stop"):
                dc_on = False
                respuesta["status"]["dc_on"] = dc_on

            elif cmd in ("start servo", "start_servo", "servo start"):
                if not servo_on:
                    servo_on = True
                    threading.Thread(target=hilo_servo, daemon=True).start()
                respuesta["status"]["servo_on"] = servo_on

            elif cmd in ("stop servo", "stop_servo", "servo stop"):
                servo_on = False
                respuesta["status"]["servo_on"] = servo_on

            elif cmd in ("save-data", "save data", "guardar"):
                guardar_datos_flag = True
                respuesta["status"]["saving_data"] = guardar_datos_flag

            elif cmd in ("stop save", "stop save-data", "detener"):
                guardar_datos_flag = False
                respuesta["status"]["saving_data"] = guardar_datos_flag

            elif cmd in ("status", "get status"):
                # devuelve estado actual
                pass

            else:
                respuesta = {"ok": False, "error": "Comando inválido", "status": get_status_dict()}

            # enviar respuesta JSON (incluye booleanos de estado)
            sock.sendto(json.dumps(respuesta).encode(), addr)

        except Exception as e:
            print("[ERROR UDP]", e)

# -------- SERVIDOR TCP VIDEO (acepta una conexión a la vez) -----------
def video_tcp_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_IP, TCP_PORT))
    srv.listen(1)
    print(f"[SERVER] TCP video escuchando en {SERVER_IP}:{TCP_PORT}")
    try:
        conn, addr = srv.accept()
        print(f"[SERVER] Cliente de video conectado {addr}")
        hilo_video(conn)
    except Exception as e:
        print("[VIDEO TCP] Error accept:", e)
    finally:
        try:
            srv.close()
        except:
            pass

# -------- MAIN ----------
if __name__ == "__main__":
    threading.Thread(target=servidor_udp, daemon=True).start()
    print("[SERVIDOR] Listo. Esperando comandos.")
    while True:
        time.sleep(1)
