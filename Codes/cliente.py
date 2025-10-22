import socket
import struct
import cv2
import numpy as np
import threading

# ─ Configuración del servidor ─
SERVER_IP = "172.32.214.66"  # Cambia por la IP de tu Raspberry Pi
UDP_PORT = 50000             # Puerto de comandos
TCP_PORT = 50001             # Puerto de video
BUFFER_SIZE = 1024

# ─ Variables de estado ─
guardando = False


# ─ Función para recibir el video ─
def recibir_video():
    """Recibe el stream de video desde la Raspberry Pi."""
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, TCP_PORT))
        print(f"[CLIENTE] ✅ Conectado al servidor de video {SERVER_IP}:{TCP_PORT}")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar al servidor de video: {e}")
        return

    data = b""
    payload_size = struct.calcsize("Q")

    try:
        while True:
            # Leer tamaño del frame
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("[CLIENTE] ⚠️ Conexión de video cerrada por el servidor.")
                    return
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            # Leer frame
            while len(data) < msg_size:
                data += client_socket.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            if guardando:
                cv2.putText(frame, "GUARDANDO DATOS...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Video desde Raspberry Pi", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"[ERROR VIDEO] {e}")
    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        print("[CLIENTE] Video cerrado.")


# ─ Función para enviar comandos ─
def enviar_comandos():
    """Envía comandos UDP a la Raspberry y recibe respuestas."""
    global guardando
    UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDPClient.settimeout(3)
    print(f"[CLIENTE] 📡 Enviando comandos a {SERVER_IP}:{UDP_PORT}")

    while True:
        comando = input("Comando ('servo1', 'temperatura', 'luz', 'dc', 'guardar', 'derecha', 'izquierda', 'detener', 'q' para salir): ").strip().lower()

        if comando == 'q':
            print("[CLIENTE] 🔚 Finalizando conexión...")
            break

        try:
            UDPClient.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
            data, _ = UDPClient.recvfrom(BUFFER_SIZE)
            print(f"[SERVIDOR] {data.decode('utf-8')}")
        except socket.timeout:
            print("⚠️ No se recibió respuesta del servidor.")
        except Exception as e:
            print(f"[ERROR] {e}")

        if comando == "guardar":
            guardando = True
            print("[CLIENTE] 🟢 Guardado de datos iniciado.")
        elif comando == "detener":
            guardando = False
            print("[CLIENTE] 🔴 Guardado de datos detenido.")

    UDPClient.close()


# ─ Main ─
if __name__ == "__main__":
    # Hilo para video
    threading.Thread(target=recibir_video, daemon=True).start()
    enviar_comandos()
    print("[CLIENTE] Finalizado.")
