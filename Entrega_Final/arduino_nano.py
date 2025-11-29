import serial
import time

PORT = "/dev/ttyUSB0"   # Para Arduino Nano con chip CH340
BAUD = 9600

print("Conectando a Arduino...")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Esperar a que Arduino reinicie

# Leer mensaje de bienvenida
ready = ser.readline().decode(errors='ignore').strip()
print("Arduino dice:", ready)

# Enviar comando de prueba
ser.write(b"ping\n")
print("Comando enviado: ping")

# Leer respuesta
respuesta = ser.readline().decode(errors='ignore').strip()
print("Respuesta:", respuesta)

ser.close()
