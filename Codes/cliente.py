import socket
import struct
import cv2
import numpy as np
import threading

SERVER_IP = '172.32.214.66'  # Cambia a la IP de tu Raspberry
UDP_PORT = 50000
TCP_PORT_1 = 50001
TCP_PORT_2 = 50002
BUFFER_SIZE = 1024

# ─ Recibir video ─
def recibir_video(port, window_name):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, port))
    print(f"[CLIENTE] Conectado a {SERVER_IP}:{port}")

    data = b""
    payload_size = struct.calcsize("Q")

    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet:
                print(f"[{window_name}] Conexión cerrada.")
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
        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    client_socket.close()

# ─ Enviar comandos UDP ─
def enviar_comandos():
    UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[CLIENTE] Enviando comandos a {SERVER_IP}:{UDP_PORT}")

    while True:
        comando = input("Escribe comando ('servo1', 'temperatura', 'luz', 'dc', 'q' para salir): ").strip()
        if comando.lower() == 'q':
            print("Saliendo...")
            break

        UDPClient.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        data, _ = UDPClient.recvfrom(BUFFER_SIZE)
        print("Respuesta:", data.decode('utf-8'))

    UDPClient.close()

# ─ Main ─
if __name__ == "__main__":
    threading.Thread(target=recibir_video, args=(TCP_PORT_1, "Camara 1"), daemon=True).start()
    threading.Thread(target=recibir_video, args=(TCP_PORT_2, "Camara 2"), daemon=True).start()
    enviar_comandos()
