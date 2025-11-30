import pygame
import serial
import time

# --- Conexión serial ---
try:
    arduino = serial.Serial('COM3', 115200)  # 🔹 más rápido
    time.sleep(2)
    print("🔌 Conectado al Arduino correctamente.")
except:
    arduino = None
    print("⚠️ No se pudo conectar al Arduino. Solo mostrando en consola.")

# --- Inicializar pygame ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("⚠️ No se detectó ningún mando.")
    exit()

mando = pygame.joystick.Joystick(0)
mando.init()
print(f"✅ Mando detectado: {mando.get_name()}")
print("\nCruceta = servo | Joystick IZQ = motores IZQ | Joystick DER = motores DER")

# --- Variables de estado ---
DEAD_ZONE = 0.1  # zona muerta para el drift
servo_delay = 0.1  # cada cuánto mandar actualización de servo si se mantiene presionado
ultimo_mov_servo = time.time()

try:
    while True:
        pygame.event.pump()
        hat_x, _ = mando.get_hat(0)
        eje_y_izq = mando.get_axis(1)   # joystick izquierdo (vertical)
        eje_y_der = mando.get_axis(3)   # joystick derecho (vertical)

        # --- Aplicar zona muerta ---
        if abs(eje_y_izq) < DEAD_ZONE:
            eje_y_izq = 0
        if abs(eje_y_der) < DEAD_ZONE:
            eje_y_der = 0

        # --- Servo (mientras se mantiene presionado) ---
        ahora = time.time()
        if hat_x == 1 and (ahora - ultimo_mov_servo) > servo_delay:
            if arduino:
                arduino.write(b"DER\n")
            print("Servo → derecha")
            ultimo_mov_servo = ahora

        elif hat_x == -1 and (ahora - ultimo_mov_servo) > servo_delay:
            if arduino:
                arduino.write(b"IZQ\n")
            print("Servo → izquierda")
            ultimo_mov_servo = ahora

        # --- Motores Izquierdos ---
        pwm_izq = int(eje_y_izq * -255)
        if arduino:
            arduino.write(f"MOTORESIZQ,{pwm_izq}\n".encode())
        if abs(pwm_izq) > 5:
            print(f"PWM Izq: {pwm_izq}")
        else:
            print("PWM Izq: detenido")

        # --- Motores Derechos ---
        pwm_der = int(eje_y_der * -255)
        if abs(pwm_der) > 5:
            if pwm_der > 0:
                if arduino:
                    arduino.write(f"MOTORESDER_A,{pwm_der}\n".encode())  # hacia adelante
                print(f"PWM Der (adelante): {pwm_der}")
            else:
                if arduino:
                    arduino.write(f"MOTORESDER_R,{-pwm_der}\n".encode())  # reversa
                print(f"PWM Der (reversa): {pwm_der}")
        else:
            if arduino:
                arduino.write(b"MOTORESDER_STOP\n")
            print("PWM Der: detenido")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n👋 Programa terminado por el usuario.")
    if arduino:
        arduino.close()
finally:
    pygame.quit()
