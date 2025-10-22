import socket
import struct
import cv2
import numpy as np
import threading

# ─ Configuración del servidor ─
SERVER_IP = '192.168.1.10'  # IP del servidor (Raspberry Pi)
UDP_PORT = 50000             # Puerto de comandos
TCP_PORT = 50001             # Puerto de video
BUFFER_SIZE = 1024


# ─ Función para recibir el video ─
def recibir_video():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, TCP_PORT))
    data = b""
    payload_size = struct.calcsize("Q")
    print(f"[CLIENTE] Conectado al servidor de video {SERVER_IP}:{TCP_PORT}")

    try:
        while True:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    print("[CLIENTE] Conexión de video cerrada por el servidor.")
                    return
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            cv2.imshow("Video desde Raspberry Pi", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"[ERROR VIDEO] {e}")

    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        print("[CLIENTE] Video cerrado.")


# ─ Función para enviar comandos por UDP ─
def enviar_comandos():
    UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[CLIENTE] Enviando comandos a {SERVER_IP}:{UDP_PORT}")

    while True:
        comando = input("Escribe comando ('servo1', 'temperatura', 'luz', 'dc', 'q' para salir): ").strip()
        if comando.lower() == 'q':
            print("Saliendo...")
            break

        bytesToSend = comando.encode('utf-8')
        UDPClient.sendto(bytesToSend, (SERVER_IP, UDP_PORT))

        try:
            data, address = UDPClient.recvfrom(BUFFER_SIZE)
            data = data.decode('utf-8')
            print('Respuesta del servidor:', data)
        except socket.timeout:
            print("No se recibió respuesta del servidor.")

    UDPClient.close()


# ─ Main ─
if __name__ == "__main__":
    # Hilo para recibir video
    hilo_video = threading.Thread(target=recibir_video, daemon=True)
    hilo_video.start()

    # Hilo para enviar comandos
    enviar_comandos()

    print("[CLIENTE] Finalizado.")
