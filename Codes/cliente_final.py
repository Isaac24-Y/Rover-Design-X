#!/usr/bin/env python3
import socket, threading, struct, cv2, numpy as np, csv, os, time, json

# ---------- CONFIG ----------
SERVER_IP = "172.32.214.66"  # cambia por la IP de tu Raspberry Pi
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 4096

SAVE_DIR = "client_saved"
os.makedirs(SAVE_DIR, exist_ok=True)
CSV_FILE = os.path.join(SAVE_DIR, "saved_data.csv")

# Estados locales
camera_on = False
sensor_on = False
guardar = False

# Último valor sensor recibido (thread-safe)
sensor_lock = threading.Lock()
latest_sensor = {"ldr": "", "temp": ""}

# ---------- HELPER ----------
def append_csv_row(timestamp, width, height, analog_voltage, img_name):
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            # Columns requested: pixelsU, pixelsV, Analog Voltage, timestamp (plus filename)
            writer.writerow(["timestamp", "pixelsU", "pixelsV", "AnalogVoltage", "image"])
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, width, height, analog_voltage, img_name])

# ---------- THREAD: recibir video (color) ----------
def recibir_video():
    global camera_on, guardar
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((SERVER_IP, TCP_PORT))
        data = b""
        payload_size = struct.calcsize("Q")
        camera_on = True
        print("[VIDEO CLIENT] Conectado al servidor de video.")
        while camera_on:
            # recibir tamaño
            while len(data) < payload_size:
                packet = s.recv(4096)
                if not packet:
                    camera_on = False
                    break
                data += packet
            if not camera_on:
                break
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            while len(data) < msg_size:
                packet = s.recv(4096)
                if not packet:
                    break
                data += packet
            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Decodificar color
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Mostrar indicación de guardado
            display_frame = frame.copy()
            if guardar:
                cv2.putText(display_frame, "GUARDANDO DATOS...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Video (Color) - Raspberry Pi", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                camera_on = False
                break

            # Si guardamos, escribir frame y fila CSV
            if guardar:
                # Obtener latest sensor value (atomic)
                with sensor_lock:
                    ldr = latest_sensor.get("ldr", "")
                timestamp = time.strftime("%Y%m%d_%H%M%S_%f")
                h, w = frame.shape[:2]
                img_name = f"frame_{timestamp}.png"
                img_path = os.path.join(SAVE_DIR, img_name)
                # Guardar imagen color (PNG)
                cv2.imwrite(img_path, frame)
                # Append CSV: timestamp, pixelsU(width), pixelsV(height), AnalogVoltage, image file
                append_csv_row(timestamp, w, h, ldr, img_path)

        s.close()
        cv2.destroyAllWindows()
        print("[VIDEO CLIENT] Video cerrado.")
    except Exception as e:
        print("[ERROR VIDEO CLIENT]", e)

# ---------- THREAD: recibir sensor UDP ----------
def recibir_sensor():
    global sensor_on, latest_sensor
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind a any port: server will send sensor data to client IP:port used when it asked sensor.
    # We listen on a dedicated local port (UDP_PORT+2) for sensor data
    LOCAL_SENSOR_PORT = UDP_PORT + 2
    sock.bind(('', LOCAL_SENSOR_PORT))
    print(f"[SENSOR CLIENT] Escuchando sensor UDP en puerto local {LOCAL_SENSOR_PORT}")
    sensor_on = True
    while sensor_on:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            text = data.decode()
            # Esperamos JSON del servidor: {"ldr":"x.xx","temp":"y.yy"}
            try:
                js = json.loads(text)
                with sensor_lock:
                    latest_sensor["ldr"] = js.get("ldr", "")
                    latest_sensor["temp"] = js.get("temp", "")
                # Si estamos guardando, we do NOT append here; saving is triggered by frames in recibir_video
                # but you could also choose to log here if needed.
                print(f"[SENSOR CLIENT] LDR={latest_sensor['ldr']}  TEMP={latest_sensor['temp']}")
            except Exception as e:
                print("[SENSOR CLIENT] Mensaje no JSON:", text)
        except Exception as e:
            print("[ERROR SENSOR CLIENT]", e)
            break
    sock.close()

# ---------- FUNCION: enviar comandos UDP (y recibir status) ----------
def enviar_comandos():
    global guardar, camera_on, sensor_on
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3)

    print("Comandos disponibles:")
    print(" start camera | stop camera")
    print(" start sensor | stop sensor")
    print(" start dc     | stop dc")
    print(" start servo  | stop servo")
    print(" save-data    | stop save")
    print(" status       (consulta estados)")
    print(" q            (salir)\n")

    while True:
        cmd = input("Comando: ").strip()
        if cmd.lower() == 'q':
            # asegurar detener guardado y cerrar hilos
            guardar = False
            camera_on = False
            sensor_on = False
            break

        try:
            sock.sendto(cmd.encode(), (SERVER_IP, UDP_PORT))
            data, addr = sock.recvfrom(BUFFER_SIZE)
            # Esperamos JSON con estado
            try:
                resp = json.loads(data.decode())
                print("[SERVIDOR] Respuesta:", resp)
                # actualizar local flags según respuesta
                status = resp.get("status", {})
                camera_on = status.get("camera_on", camera_on)
                sensor_on = status.get("sensor_on", sensor_on)
                # guardar flag enlazado al comando save-data
                if cmd.lower() in ("save-data", "guardar"):
                    guardar = True
                elif cmd.lower() in ("stop save", "stop save-data", "detener"):
                    guardar = False
            except Exception:
                print("[SERVIDOR] (no-json) ", data.decode())
        except socket.timeout:
            print("[ERROR] Timeout esperando respuesta del servidor.")
        except Exception as e:
            print("[ERROR] Enviando comando:", e)

    sock.close()

# ---------- MAIN ----------
if __name__ == "__main__":
    # Iniciar hilo sensor (escucha local). Important: the server will send sensor data to the client's address/port it received when starting sensor.
    t_sensor = threading.Thread(target=recibir_sensor, daemon=True)
    t_sensor.start()

    # Iniciar hilo de video (abre conexión TCP al servidor cuando se ejecuta)
    t_video = threading.Thread(target=recibir_video, daemon=True)
    t_video.start()

    # Iniciar loop de comandos en main thread (entrada del usuario)
    enviar_comandos()
    print("[CLIENT] Finalizado. Asegúrate de detener camera/sensor en el servidor si aún están activos.")
