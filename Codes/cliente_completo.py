import socket, struct, cv2, numpy as np, threading, time

SERVER_IP = '192.168.1.10'  # IP del servidor (Raspberry Pi)
#SERVER_IP = "172.32.214.66"
UDP_PORT = 50000
TCP_PORT = 50001
BUFFER_SIZE = 1024

camera_on = False

def recibir_video():
    global camera_on
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SERVER_IP, TCP_PORT))
    data = b""
    payload_size = struct.calcsize("Q")
    camera_on = True
    while camera_on:
        while len(data) < payload_size:
            packet = s.recv(4096)
            if not packet: return
            data += packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]
        while len(data) < msg_size:
            data += s.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
        cv2.imshow("Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            camera_on = False
            break
    s.close()
    cv2.destroyAllWindows()

def enviar_comandos():
    global camera_on
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3)
    while True:
        cmd = input("Comando: ")
        if cmd.lower() == "q": break
        sock.sendto(cmd.encode(), (SERVER_IP, UDP_PORT))
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            print(f"[SERVER] {data.decode()}")
        except:
            print("No hay respuesta")
    sock.close()

if __name__=="__main__":
    threading.Thread(target=recibir_video, daemon=True).start()
    enviar_comandos()
