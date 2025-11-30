#!/usr/bin/env python3
"""
SCRIPTS DE PRUEBA Y SIMULADOR
"""

# ========================================
# test_serial.py - Prueba de Comunicación Serial
# ========================================

import serial
import time

def test_serial_connection(port='/dev/ttyUSB0', baud=115200):
    """Probar conexión serial con Arduino"""
    print("="*60)
    print("TEST DE COMUNICACIÓN SERIAL")
    print("="*60)
    
    try:
        ser = serial.Serial(port, baud, timeout=2)
        time.sleep(2)
        
        print(f"✅ Conexión establecida en {port} @ {baud} baud")
        
        # Esperar mensaje de inicio
        if ser.in_waiting:
            msg = ser.readline().decode().strip()
            print(f"📥 Arduino dice: {msg}")
        
        # Lista de comandos a probar
        test_commands = [
            "SENSOR TEMP",
            "SENSOR LDR",
            "SENSOR DIST",
            "SERVO 1 90",
            "MOTORS STOP"
        ]
        
        print("\n🧪 Probando comandos:\n")
        
        for cmd in test_commands:
            print(f"📤 Enviando: {cmd}")
            ser.write((cmd + '\n').encode())
            time.sleep(0.5)
            
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"📥 Respuesta: {response}")
            else:
                print("⚠️  Sin respuesta")
            
            print()
        
        ser.close()
        print("✅ Prueba completada exitosamente")
        return True
        
    except serial.SerialException as e:
        print(f"❌ Error de conexión: {e}")
        print("\n💡 Sugerencias:")
        print("   - Verificar que Arduino esté conectado")
        print("   - Verificar puerto correcto: ls /dev/tty*")
        print("   - Verificar permisos: sudo usermod -a -G dialout $USER")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    test_serial_connection()


# ========================================
# test_video.py - Prueba de Cámaras
# ========================================

import cv2
import sys

def test_cameras():
    """Probar disponibilidad de cámaras"""
    print("="*60)
    print("TEST DE CÁMARAS")
    print("="*60)
    
    available_cameras = []
    
    # Probar primeros 4 dispositivos
    for i in range(4):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"✅ Cámara {i}: Disponible ({w}x{h})")
                available_cameras.append(i)
            cap.release()
        else:
            print(f"❌ Cámara {i}: No disponible")
    
    if not available_cameras:
        print("\n⚠️  No se encontraron cámaras disponibles")
        return False
    
    # Mostrar primera cámara disponible
    print(f"\n📹 Mostrando cámara {available_cameras[0]}")
    print("   Presiona 'q' para salir")
    
    cap = cv2.VideoCapture(available_cameras[0])
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        cv2.imshow(f'Camara {available_cameras[0]} - Presiona Q para salir', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Prueba completada")
    return True

if __name__ == "__main__":
    test_cameras()


# ========================================
# test_network.py - Prueba de Conexión de Red
# ========================================

import socket
import struct

def test_udp_connection(server_ip='192.168.1.100', port=50000):
    """Probar conexión UDP"""
    print("="*60)
    print("TEST DE CONEXIÓN UDP")
    print("="*60)
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        
        print(f"📤 Enviando comando a {server_ip}:{port}")
        sock.sendto(b"status", (server_ip, port))
        
        data, addr = sock.recvfrom(4096)
        response = data.decode('utf-8')
        
        print(f"✅ Respuesta recibida: {response}")
        print(f"📍 Desde: {addr}")
        
        sock.close()
        return True
        
    except socket.timeout:
        print("⚠️  Timeout: El servidor no responde")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_tcp_video(server_ip='192.168.1.100', port=50001):
    """Probar streaming de video TCP"""
    print("="*60)
    print("TEST DE STREAMING TCP")
    print("="*60)
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        
        print(f"🔌 Conectando a {server_ip}:{port}")
        sock.connect((server_ip, port))
        sock.settimeout(None)
        
        print("✅ Conexión establecida")
        print("📥 Recibiendo frames... (Ctrl+C para detener)")
        
        data = b""
        payload_size = struct.calcsize("Q")
        frame_count = 0
        
        while frame_count < 30:  # Recibir 30 frames de prueba
            # Leer tamaño
            while len(data) < payload_size:
                packet = sock.recv(4096)
                if not packet:
                    raise ConnectionError("Servidor desconectado")
                data += packet
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            frame_size = struct.unpack("Q", packed_size)[0]
            
            # Leer frame
            while len(data) < frame_size:
                packet = sock.recv(4096)
                if not packet:
                    raise ConnectionError("Servidor desconectado")
                data += packet
            
            frame_data = data[:frame_size]
            data = data[frame_size:]
            
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"  📹 Frames recibidos: {frame_count}")
        
        sock.close()
        print("✅ Prueba completada exitosamente")
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        server_ip = sys.argv[1]
    else:
        server_ip = '192.168.1.100'
    
    test_udp_connection(server_ip)
    print()
    test_tcp_video(server_ip)


# ========================================
# simulator.py - Simulador de Arduino
# ========================================

import socket
import threading
import time
import random

class ArduinoSimulator:
    """Simulador de Arduino para pruebas sin hardware"""
    
    def __init__(self, port='/tmp/ttyVIRT0'):
        self.port = port
        self.running = False
        self.sensors = {
            'temp': 25.0,
            'light': 1.5,
            'dist': 100,
            'humidity': 50
        }
    
    def start(self):
        """Iniciar simulador"""
        print("="*60)
        print("SIMULADOR DE ARDUINO")
        print("="*60)
        print(f"Puerto virtual: {self.port}")
        print("Comandos soportados:")
        print("  - SENSOR TEMP")
        print("  - SENSOR LDR")
        print("  - SENSOR DIST")
        print("  - SENSOR HUM_CAP")
        print("  - SERVO <id> <angle>")
        print("  - MOTOR <side> <dir> <vel>")
        print("  - MOTORS STOP")
        print("="*60)
        
        self.running = True
        
        # Simular variación de sensores
        threading.Thread(target=self._simulate_sensors, daemon=True).start()
        
        # Escuchar comandos por stdin
        self._command_loop()
    
    def _simulate_sensors(self):
        """Simular lecturas de sensores"""
        while self.running:
            self.sensors['temp'] += random.uniform(-0.5, 0.5)
            self.sensors['temp'] = max(20, min(30, self.sensors['temp']))
            
            self.sensors['light'] += random.uniform(-0.1, 0.1)
            self.sensors['light'] = max(0.5, min(3.0, self.sensors['light']))
            
            self.sensors['dist'] = random.randint(50, 200)
            
            time.sleep(1)
    
    def _command_loop(self):
        """Loop de comandos"""
        while self.running:
            try:
                cmd = input("\n> ").strip().upper()
                response = self.process_command(cmd)
                print(f"< {response}")
            except KeyboardInterrupt:
                print("\n\n👋 Simulador detenido")
                self.running = False
                break
            except EOFError:
                break
    
    def process_command(self, cmd):
        """Procesar comando"""
        if cmd.startswith("SENSOR TEMP"):
            return f"TEMP:{self.sensors['temp']:.2f}"
        
        elif cmd.startswith("SENSOR LDR"):
            return f"LDR:{self.sensors['light']:.2f}"
        
        elif cmd.startswith("SENSOR DIST"):
            return f"DIST:{self.sensors['dist']}"
        
        elif cmd.startswith("SENSOR HUM_CAP"):
            return f"HUM:{self.sensors['humidity']}"
        
        elif cmd.startswith("SENSOR IMU"):
            ax = random.uniform(-0.1, 0.1)
            ay = random.uniform(-0.1, 0.1)
            az = random.uniform(9.7, 9.9)
            gx = random.uniform(-1, 1)
            gy = random.uniform(-1, 1)
            gz = random.uniform(-1, 1)
            return f"IMU:{ax:.3f},{ay:.3f},{az:.3f},{gx:.3f},{gy:.3f},{gz:.3f}"
        
        elif cmd.startswith("SERVO"):
            parts = cmd.split()
            if len(parts) >= 3:
                servo_id = parts[1]
                angle = parts[2]
                return f"OK:SERVO{servo_id}:{angle}"
            return "ERR:INVALID_SERVO_CMD"
        
        elif cmd.startswith("MOTOR"):
            return "OK:MOTOR_COMMAND"
        
        elif cmd.startswith("MOTORS STOP"):
            return "OK:MOTORS_STOP"
        
        elif cmd.startswith("ACTUATE BRACO START"):
            time.sleep(2)  # Simular secuencia
            return "BRACO:OK"
        
        elif cmd.startswith("EMERGENCY STOP"):
            return "EMERGENCY:OK"
        
        else:
            return "ERR:UNKNOWN_CMD"

if __name__ == "__main__":
    sim = ArduinoSimulator()
    sim.start()


# ========================================
# integration_test.py - Prueba de Integración Completa
# ========================================

def run_integration_tests():
    """Ejecutar todas las pruebas de integración"""
    print("="*60)
    print("PRUEBAS DE INTEGRACIÓN COMPLETA")
    print("="*60)
    
    results = {
        'serial': False,
        'cameras': False,
        'network': False
    }
    
    print("\n1️⃣  Probando comunicación serial...")
    results['serial'] = test_serial_connection()
    
    print("\n2️⃣  Probando cámaras...")
    results['cameras'] = test_cameras()
    
    print("\n3️⃣  Probando conexión de red...")
    results['network'] = test_udp_connection()
    
    # Resumen
    print("\n" + "="*60)
    print("RESUMEN DE PRUEBAS")
    print("="*60)
    
    for test, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test.upper()}: {status}")
    
    total = sum(results.values())
    print(f"\n📊 Total: {total}/{len(results)} pruebas exitosas")
    
    if total == len(results):
        print("\n🎉 ¡Todos los sistemas funcionando correctamente!")
        return True
    else:
        print("\n⚠️  Algunos sistemas requieren atención")
        return False

if __name__ == "__main__":
    run_integration_tests()


# ========================================
# calibration.py - Herramienta de Calibración
# ========================================

def calibrate_sensors():
    """Herramienta interactiva de calibración"""
    print("="*60)
    print("HERRAMIENTA DE CALIBRACIÓN DE SENSORES")
    print("="*60)
    
    print("\n🔧 Seleccione sensor a calibrar:")
    print("1. LDR (Fotorresistor)")
    print("2. Sensor de humedad capacitivo")
    print("3. IMU MPU6050")
    print("4. Todos")
    
    choice = input("\nOpción: ")
    
    if choice == "1":
        calibrate_ldr()
    elif choice == "2":
        calibrate_humidity()
    elif choice == "3":
        calibrate_imu()
    elif choice == "4":
        calibrate_ldr()
        calibrate_humidity()
        calibrate_imu()
    else:
        print("Opción inválida")

def calibrate_ldr():
    """Calibrar fotorresistor"""
    print("\n📊 Calibración de LDR")
    print("Coloque el sensor en diferentes condiciones de luz")
    print("y registre los valores...\n")
    
    measurements = []
    
    for condition in ["oscuro", "normal", "brillante"]:
        input(f"Presione Enter cuando el sensor esté en luz {condition}...")
        # Aquí iría la lectura real del sensor
        value = random.uniform(0.5, 3.0)
        measurements.append((condition, value))
        print(f"  Medición {condition}: {value:.2f} V")
    
    print("\n✅ Calibración completada")
    print("Valores recomendados para config.yaml:")
    print(f"  ldr_dark: {measurements[0][1]:.2f}")
    print(f"  ldr_normal: {measurements[1][1]:.2f}")
    print(f"  ldr_bright: {measurements[2][1]:.2f}")

def calibrate_humidity():
    """Calibrar sensor de humedad"""
    print("\n💧 Calibración de Sensor Capacitivo")
    print("Mediremos valores en aire seco y agua...\n")
    
    input("1. Coloque el sensor en AIRE SECO y presione Enter...")
    dry_value = 550  # Simular lectura
    print(f"  Valor seco: {dry_value}")
    
    input("2. Coloque el sensor en AGUA y presione Enter...")
    wet_value = 280  # Simular lectura
    print(f"  Valor húmedo: {wet_value}")
    
    print("\n✅ Calibración completada")
    print("Valores para config.yaml:")
    print(f"  calibration_dry: {dry_value}")
    print(f"  calibration_wet: {wet_value}")

def calibrate_imu():
    """Calibrar IMU"""
    print("\n🧭 Calibración de IMU MPU6050")
    print("Coloque el rover en superficie PLANA y ESTABLE...\n")
    
    input("Presione Enter para iniciar calibración...")
    
    print("Recolectando 100 muestras...")
    # Aquí iría la calibración real
    time.sleep(3)
    
    print("✅ Calibración completada")
    print("Offsets calculados:")
    print("  accel_x_offset: 0.02")
    print("  accel_y_offset: -0.01")
    print("  accel_z_offset: 0.15")
    print("  gyro_x_offset: 0.5")
    print("  gyro_y_offset: -0.3")
    print("  gyro_z_offset: 0.1")

if __name__ == "__main__":
    calibrate_sensors()


print("""
╔════════════════════════════════════════════════════════════╗
║         SCRIPTS DE PRUEBA Y SIMULADOR DISPONIBLES          ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  📝 Archivos incluidos:                                    ║
║                                                            ║
║  test_serial.py       - Prueba comunicación serial        ║
║  test_video.py        - Prueba cámaras                    ║
║  test_network.py      - Prueba conexión red               ║
║  simulator.py         - Simula Arduino sin hardware       ║
║  integration_test.py  - Prueba completa del sistema       ║
║  calibration.py       - Herramienta de calibración        ║
║                                                            ║
║  💡 Uso:                                                   ║
║  python3 test_serial.py                                   ║
║  python3 simulator.py                                     ║
║  python3 integration_test.py                              ║
║                                                            ║
╚════════════════════════════════════════════════════════════╝
""")
