# config.py - Configuración centralizada del sistema
"""
Archivo de configuración centralizado para el sistema de rover.
Editar estos valores según tu configuración específica.
"""

# ==================== RED Y CONEXIÓN ====================
# IP de la Raspberry Pi (servidor)
SERVER_IP = "192.168.1.10"  # ⚠️ CAMBIAR A TU IP

# Puertos de comunicación
UDP_PORT = 50000              # Comandos
TCP_PORT_VIDEO_FRONTAL = 50001  # Stream cámara frontal
TCP_PORT_VIDEO_SUPERIOR = 50002 # Stream cámara superior
TCP_PORT_STATUS = 50002         # Estado del rover

BUFFER_SIZE = 4096
CONNECTION_TIMEOUT = 5  # segundos

# ==================== ARDUINO ====================
# Puerto serial (Raspberry Pi)
ARDUINO_PORT = '/dev/ttyACM0'  # Linux/Mac
# ARDUINO_PORT = 'COM3'        # Windows (descomentar si usas Windows)

BAUD_RATE = 9600
SERIAL_TIMEOUT = 1.0  # segundos

# ==================== CÁMARAS ====================
# Índices de dispositivos de video
CAMERA_FRONTAL = 0
CAMERA_SUPERIOR = 2

# Resolución de video
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30

# Calidad de compresión JPEG (1-100, menor = más compresión)
JPEG_QUALITY = 60

# Delay entre frames (segundos)
# Menor = más FPS pero más carga de red
FRAME_DELAY = 0.033  # ~30 FPS

# ==================== SENSORES ====================
# Intervalo de actualización de sensores (segundos)
SENSOR_UPDATE_INTERVAL = 0.5

# Calibración sensor de humedad (valores ADC)
HUMEDAD_SECO = 588    # Lectura en aire seco
HUMEDAD_HUMEDO = 308  # Lectura sumergido en agua

# Calibración LDR
LUZ_OSCURO = 1023    # ADC en oscuridad
LUZ_BRILLANTE = 0    # ADC con luz intensa

# Umbrales de sensores
TEMP_MIN = 0.0       # °C mínimo válido
TEMP_MAX = 50.0      # °C máximo válido
HUMEDAD_MIN = 0      # % mínimo
HUMEDAD_MAX = 100    # % máximo

# ==================== MOTORES ====================
# Velocidad por defecto (0-255 PWM)
VELOCIDAD_AVANZAR = 200
VELOCIDAD_RETROCEDER = 200
VELOCIDAD_GIRO = 200

# Velocidad mínima/máxima
VELOCIDAD_MIN = 0
VELOCIDAD_MAX = 255

# ==================== SERVOS ====================
# Límites de servos (grados)
SERVO1_MIN = 0     # Codo brazo
SERVO1_MAX = 180

SERVO2_MIN = 0     # Sensor brazo
SERVO2_MAX = 180

SERVO3_MIN = 0     # Cámara frontal
SERVO3_MAX = 30    # ⚠️ LIMITADO A 30°

SERVO4_MIN = 0     # Cámara superior
SERVO4_MAX = 180

# Posiciones iniciales
SERVO1_INICIAL = 0    # Brazo replegado
SERVO2_INICIAL = 0    # Sensor replegado
SERVO3_INICIAL = 15   # Cámara al centro
SERVO4_INICIAL = 90   # Cámara al centro

# Secuencia autónoma (grados)
SERVO2_AUTONOMO = 90    # Sensor desplegado
SERVO1_AUTONOMO = 180   # Codo extendido

# Tiempo de espera en secuencia autónoma (segundos)
DELAY_SERVO_ACTIVACION = 1.0
DELAY_MEDICION_HUMEDAD = 1.5
DELAY_REPLIEGUE = 1.0

# ==================== DETECCIÓN DE MARCADORES ====================
# Marcador clave por defecto
MARCADOR_CLAVE_DEFAULT = "Circulo"

# Opciones disponibles
MARCADORES_DISPONIBLES = [
    "Cruz",
    "T", 
    "Circulo",
    "Triangulo",
    "Cuadrado"
]

# Área mínima de contorno (píxeles)
AREA_MINIMA_MARCADOR = 500

# Rangos HSV para colores fosforescentes
# Formato: [H_min, S_min, V_min]
COLOR_NARANJA_MIN = [8, 150, 180]
COLOR_NARANJA_MAX = [18, 255, 255]

COLOR_VERDE_MIN = [35, 120, 150]
COLOR_VERDE_MAX = [85, 255, 255]

COLOR_AMARILLO_MIN = [22, 140, 150]
COLOR_AMARILLO_MAX = [32, 255, 255]

# Kernel para operaciones morfológicas
MORPH_KERNEL_SIZE = (3, 3)

# ==================== ARCHIVOS ====================
# Nombre del archivo CSV de datos
CSV_FILENAME = "datos_exploracion.csv"

# Directorio para capturas de imágenes
CAPTURAS_DIR = "capturas"

# Directorio para logs
LOGS_DIR = "logs"

# ==================== INTERFAZ GRÁFICA ====================
# Colores (tema oscuro)
COLOR_BG = "#1e1e1e"
COLOR_PANEL = "#2d2d2d"
COLOR_WIDGET = "#3d3d3d"
COLOR_TEXT = "#ffffff"
COLOR_ACCENT = "#00ff00"

# Tamaño de ventana
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 900

# FPS de actualización GUI
GUI_FPS = 30
GUI_UPDATE_INTERVAL = 33  # ms (1000/30)

# ==================== CONTROL POR TECLADO ====================
# Mapeo de teclas
TECLA_AVANZAR = ['w', 'up']
TECLA_RETROCEDER = ['s', 'down']
TECLA_IZQUIERDA = ['a', 'left']
TECLA_DERECHA = ['d', 'right']

# Auto-stop al soltar tecla
AUTO_STOP_ENABLED = True

# ==================== MAPA 2D ====================
# Historial de puntos de distancia
HISTORIAL_MAX_PUNTOS = 50

# Tamaño del mapa (píxeles)
MAPA_WIDTH = 400
MAPA_HEIGHT = 400

# Escala (píxeles por metro)
MAPA_ESCALA = 100  # 100 píxeles = 1 metro

# ==================== DEBUG Y LOGS ====================
# Nivel de verbosidad (0=mínimo, 3=máximo)
LOG_LEVEL = 2

# Guardar logs en archivo
SAVE_LOGS = True
LOG_FILENAME = "rover_logs.txt"

# Imprimir comandos seriales
DEBUG_SERIAL = True

# Imprimir frames recibidos
DEBUG_VIDEO = False

# ==================== OPTIMIZACIÓN ====================
# Usar compresión agresiva en red lenta
AGGRESSIVE_COMPRESSION = False

# Prioridad de video sobre comandos
PRIORITIZE_VIDEO = True

# Buffer TCP aumentado para video
TCP_BUFFER_SIZE = 65536

# ==================== FUNCIONES DE VALIDACIÓN ====================
def validar_configuracion():
    """Valida que la configuración sea correcta"""
    errores = []
    
    # Validar puertos
    if not (1024 <= UDP_PORT <= 65535):
        errores.append(f"Puerto UDP inválido: {UDP_PORT}")
    
    if not (1024 <= TCP_PORT_VIDEO_FRONTAL <= 65535):
        errores.append(f"Puerto TCP video frontal inválido: {TCP_PORT_VIDEO_FRONTAL}")
    
    # Validar velocidades
    if not (0 <= VELOCIDAD_AVANZAR <= 255):
        errores.append(f"Velocidad avanzar fuera de rango: {VELOCIDAD_AVANZAR}")
    
    # Validar ángulos de servos
    if SERVO3_MAX > 30:
        errores.append(f"⚠️ ADVERTENCIA: Servo3 limitado a 30°, configurado en {SERVO3_MAX}°")
    
    # Validar marcador clave
    if MARCADOR_CLAVE_DEFAULT not in MARCADORES_DISPONIBLES:
        errores.append(f"Marcador clave inválido: {MARCADOR_CLAVE_DEFAULT}")
    
    if errores:
        print("=" * 60)
        print("❌ ERRORES DE CONFIGURACIÓN:")
        for error in errores:
            print(f"  • {error}")
        print("=" * 60)
        return False
    
    print("✓ Configuración validada correctamente")
    return True

def obtener_resumen():
    """Retorna resumen de configuración"""
    return f"""
╔══════════════════════════════════════════════════════════╗
║           CONFIGURACIÓN DEL SISTEMA ROVER               ║
╚══════════════════════════════════════════════════════════╝

🌐 RED:
   • Servidor: {SERVER_IP}
   • Puerto UDP: {UDP_PORT}
   • Puerto Video Frontal: {TCP_PORT_VIDEO_FRONTAL}
   • Puerto Video Superior: {TCP_PORT_VIDEO_SUPERIOR}

🔌 HARDWARE:
   • Puerto Arduino: {ARDUINO_PORT}
   • Baud Rate: {BAUD_RATE}
   • Cámara Frontal: /dev/video{CAMERA_FRONTAL}
   • Cámara Superior: /dev/video{CAMERA_SUPERIOR}

⚙️ MOTORES:
   • Velocidad Avanzar: {VELOCIDAD_AVANZAR}/255
   • Velocidad Giro: {VELOCIDAD_GIRO}/255

🔧 SERVOS:
   • Servo1 (Codo): {SERVO1_MIN}°-{SERVO1_MAX}°
   • Servo2 (Sensor): {SERVO2_MIN}°-{SERVO2_MAX}°
   • Servo3 (Cám Frontal): {SERVO3_MIN}°-{SERVO3_MAX}° ⚠️
   • Servo4 (Cám Superior): {SERVO4_MIN}°-{SERVO4_MAX}°

🎯 MARCADORES:
   • Marcador Clave: {MARCADOR_CLAVE_DEFAULT}
   • Área Mínima: {AREA_MINIMA_MARCADOR} px²

📹 VIDEO:
   • Resolución: {VIDEO_WIDTH}x{VIDEO_HEIGHT}
   • FPS Target: {VIDEO_FPS}
   • Calidad JPEG: {JPEG_QUALITY}%

📊 SENSORES:
   • Intervalo: {SENSOR_UPDATE_INTERVAL}s
   • Humedad: {HUMEDAD_SECO} (seco) - {HUMEDAD_HUMEDO} (húmedo)

💾 ARCHIVOS:
   • CSV: {CSV_FILENAME}
   • Capturas: {CAPTURAS_DIR}/
   • Logs: {LOGS_FILENAME}

═══════════════════════════════════════════════════════════
"""

if __name__ == "__main__":
    print(obtener_resumen())
    validar_configuracion()