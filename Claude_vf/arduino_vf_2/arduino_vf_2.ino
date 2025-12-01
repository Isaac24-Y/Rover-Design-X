/*
 * ============================================================
 *  ROVER DE EXPLORACIÓN - ARDUINO NANO
 *  Versión Completa y Corregida
 * ============================================================
 */

#include <Servo.h>
#include <dht.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <MPU6050.h>

// =====================================================
// SERVOS
// =====================================================
Servo servo1;  // Codo brazo (0-180°)
Servo servo2;  // Sensor brazo (0-180°)
Servo servo3;  // Cámara frontal (0-30°)
Servo servo4;  // Cámara superior (0-180°)

// =====================================================
// SENSORES
// =====================================================
dht DHT;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu;

const int PIN_TEMP = 8;
const int PIN_HUMEDAD = A0;
const int PIN_LUZ = A1;

// =====================================================
// MOTORES DC (PWM)
// =====================================================
// Motor A = Izquierdo
const int MOTOR_A1 = 5;   // Adelante
const int MOTOR_A2 = 6;   // Atrás
// Motor B = Derecho
const int MOTOR_B1 = 10;  // Adelante
const int MOTOR_B2 = 11;  // Atrás

// =====================================================
// VARIABLES MPU6050
// =====================================================
int16_t ax, ay, az;
int16_t gx, gy, gz;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(9600);
  
  // Inicializar servos
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(9);
  
  // Posiciones iniciales
  servo1.write(0);
  servo2.write(0);
  servo3.write(15);
  servo4.write(90);
  
  // Configurar pines de motores
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Detener motores
  detenerMotores();
  
  // Inicializar MPU6050
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: ERROR");
  }
  
  // Inicializar VL53L0X
  if (!lox.begin()) {
    Serial.println("VL53L0X: ERROR");
  } else {
    Serial.println("VL53L0X: OK");
  }
  
  Serial.println("ARDUINO LISTO");
  delay(500);
}

// =====================================================
// FUNCIONES DE MOTORES - CORREGIDAS
// =====================================================

void detenerMotores() {
  // Apagar todos los pines
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void avanzar(int vel) {
  vel = constrain(vel, 0, 255);
  
  // Ambos motores adelante
  analogWrite(MOTOR_A1, vel);  // Izquierdo adelante
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, vel);  // Derecho adelante
  analogWrite(MOTOR_B2, 0);
}

void retroceder(int vel) {
  vel = constrain(vel, 0, 255);
  
  // Ambos motores atrás
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, vel);  // Izquierdo atrás
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, vel);  // Derecho atrás
}

void girarIzquierda(int vel) {
  vel = constrain(vel, 0, 255);
  
  // Izquierdo atrás, Derecho adelante (giro en el lugar)
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, vel);  // Izquierdo atrás
  analogWrite(MOTOR_B1, vel);  // Derecho adelante
  analogWrite(MOTOR_B2, 0);
}

void girarDerecha(int vel) {
  vel = constrain(vel, 0, 255);
  
  // Izquierdo adelante, Derecho atrás (giro en el lugar)
  analogWrite(MOTOR_A1, vel);  // Izquierdo adelante
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, vel);  // Derecho atrás
}

// =====================================================
// FUNCIONES DE SENSORES
// =====================================================

float leerTemperatura() {
  int chk = DHT.read11(PIN_TEMP);
  if (chk == DHTLIB_OK) {
    return DHT.temperature;
  }
  return -999;
}

int leerHumedad() {
  int lectura = analogRead(PIN_HUMEDAD);
  // Calibración: 588 = 0%, 308 = 100%
  int humedad = map(lectura, 588, 308, 0, 100);
  return constrain(humedad, 0, 100);
}

int leerLuz() {
  int lectura = analogRead(PIN_LUZ);
  // 1023 = oscuro, 0 = brillante
  int luz = map(lectura, 1023, 0, 0, 100);
  return constrain(luz, 0, 100);
}

float leerDistancia() {
  VL53L0X_RangingMeasurementData_t medida;
  lox.rangingTest(&medida, false);
  
  if (medida.RangeStatus != 4) {
    return medida.RangeMilliMeter;
  }
  return -1;
}

void leerMPU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

// =====================================================
// PROCESAMIENTO DE COMANDOS
// =====================================================

void procesarComando(String comando) {
  comando.trim();
  
  int espacio = comando.indexOf(' ');
  String accion;
  int valor = 0;
  
  if (espacio > 0) {
    accion = comando.substring(0, espacio);
    valor = comando.substring(espacio + 1).toInt();
  } else {
    accion = comando;
  }
  
  // ========== MOVIMIENTO ==========
  if (accion == "avanzar") {
    avanzar(valor > 0 ? valor : 200);
    Serial.println("OK: Avanzando");
  }
  else if (accion == "retroceder") {
    retroceder(valor > 0 ? valor : 200);
    Serial.println("OK: Retrocediendo");
  }
  else if (accion == "izquierda") {
    girarIzquierda(valor > 0 ? valor : 200);
    Serial.println("OK: Girando izquierda");
  }
  else if (accion == "derecha") {
    girarDerecha(valor > 0 ? valor : 200);
    Serial.println("OK: Girando derecha");
  }
  else if (accion == "stop") {
    detenerMotores();
    Serial.println("OK: Detenido");
  }
  
  // ========== SERVOS ==========
  else if (accion == "servo1") {
    valor = constrain(valor, 0, 180);
    servo1.write(valor);
    Serial.print("Servo1: ");
    Serial.print(valor);
    Serial.println("°");
  }
  else if (accion == "servo2") {
    valor = constrain(valor, 0, 180);
    servo2.write(valor);
    Serial.print("Servo2: ");
    Serial.print(valor);
    Serial.println("°");
  }
  else if (accion == "servo3") {
    valor = constrain(valor, 0, 30);  // Limitado a 30°
    servo3.write(valor);
    Serial.print("Servo3: ");
    Serial.print(valor);
    Serial.println("°");
  }
  else if (accion == "servo4") {
    valor = constrain(valor, 0, 180);
    servo4.write(valor);
    Serial.print("Servo4: ");
    Serial.print(valor);
    Serial.println("°");
  }
  
  // ========== SENSORES ==========
  else if (accion == "temp") {
    float temp = leerTemperatura();
    if (temp > -999) {
      Serial.print(temp, 1);
      Serial.println(" °C");
    } else {
      Serial.println("Error DHT");
    }
  }
  else if (accion == "humedad") {
    int hum = leerHumedad();
    Serial.print(hum);
    Serial.println(" %");
  }
  else if (accion == "luz") {
    int luz = leerLuz();
    Serial.print(luz);
    Serial.println(" V");
  }
  else if (accion == "dist") {
    float dist = leerDistancia();
    if (dist > 0) {
      Serial.print(dist, 0);
      Serial.println(" mm");
    } else {
      Serial.println("Fuera de rango");
    }
  }
  else if (accion == "mpu") {
    leerMPU();
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" GX: "); Serial.print(gx);
    Serial.print(" GY: "); Serial.print(gy);
    Serial.print(" GZ: "); Serial.println(gz);
  }
  
  // ========== DIAGNÓSTICO ==========
  else if (accion == "status") {
    Serial.println("=== STATUS ===");
    Serial.print("Temp: "); Serial.print(leerTemperatura()); Serial.println(" C");
    Serial.print("Luz: "); Serial.print(leerLuz()); Serial.println(" V");
    Serial.print("Humedad: "); Serial.print(leerHumedad()); Serial.println(" %");
    Serial.print("Dist: "); Serial.print(leerDistancia()); Serial.println(" mm");
  }
  
  // ========== TEST MOTORES ==========
  else if (accion == "test_motores") {
    Serial.println("=== TEST DE MOTORES ===");
    
    Serial.println("Avanzar 2s...");
    avanzar(150);
    delay(2000);
    detenerMotores();
    delay(500);
    
    Serial.println("Retroceder 2s...");
    retroceder(150);
    delay(2000);
    detenerMotores();
    delay(500);
    
    Serial.println("Izquierda 2s...");
    girarIzquierda(150);
    delay(2000);
    detenerMotores();
    delay(500);
    
    Serial.println("Derecha 2s...");
    girarDerecha(150);
    delay(2000);
    detenerMotores();
    
    Serial.println("Test completado");
  }
  
  // ========== COMANDO DESCONOCIDO ==========
  else {
    Serial.print("ERROR: Comando desconocido: ");
    Serial.println(comando);
  }
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    procesarComando(comando);
  }
  
  delay(10);
}