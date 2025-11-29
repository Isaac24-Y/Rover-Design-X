/*
 * ROVER DE EXPLORACIÓN - FIRMWARE ARDUINO NANO
 * 
 * Descripción: Controlador de bajo nivel para rover con:
 * - 2 puentes H (4 motores DC)
 * - 4 servos
 * - Múltiples sensores (DHT11, LDR, VL53L0X, MPU6050, EK1940)
 * - Comunicación serial 115200 baud
 * 
 * Autor: Sistema Rover
 * Versión: 2.0
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <DHT.h>

// =========================
// CONFIGURACIÓN DE PINES
// =========================

// Motores - Puente H Izquierdo
#define MOTOR_LEFT_PWM   3
#define MOTOR_LEFT_DIR   2

// Motores - Puente H Derecho
#define MOTOR_RIGHT_PWM  5
#define MOTOR_RIGHT_DIR  4

// Servos
#define SERVO1_PIN  6   // Cámara frontal (0-30°)
#define SERVO2_PIN  7   // Brazo - hombro (0-180°)
#define SERVO3_PIN  8   // Brazo - codo (0-180°)
#define SERVO4_PIN  9   // Sensor humedad (0-90°)

// Sensores Analógicos
#define LDR_PIN     A0  // Fotorresistor
#define DHT_PIN     A1  // DHT11
#define HUM_CAP_PIN A2  // Sensor capacitivo EK1940

// I2C (VL53L0X y MPU6050)
// SDA = A4, SCL = A5

// =========================
// CONSTANTES
// =========================
#define BAUD_RATE 115200
#define DHT_TYPE DHT11
#define MAX_CMD_LENGTH 64
#define TIMEOUT_MS 100

// Límites de seguridad
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 0

// =========================
// OBJETOS GLOBALES
// =========================
Servo servo1, servo2, servo3, servo4;
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Para MPU6050 (lectura manual I2C)
#define MPU6050_ADDR 0x68

// =========================
// VARIABLES GLOBALES
// =========================
char cmdBuffer[MAX_CMD_LENGTH];
uint8_t cmdIndex = 0;
bool emergencyStop = false;

// Estados de motores
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
bool leftForward = true;
bool rightForward = true;

// =========================
// SETUP
// =========================
void setup() {
  // Inicializar serial
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000); // Espera máx 3s
  
  // Configurar pines de motores
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  
  // Motores inicialmente detenidos
  stopAllMotors();
  
  // Inicializar servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  
  // Posición inicial servos (neutral)
  servo1.write(15);  // Cámara frontal centro
  servo2.write(90);  // Brazo posición neutra
  servo3.write(90);
  servo4.write(45);  // Sensor humedad posición intermedia
  
  // Inicializar sensores
  dht.begin();
  
  // Inicializar VL53L0X
  if (!lox.begin()) {
    Serial.println("ERR:VL53L0X_INIT_FAIL");
  }
  
  // Inicializar MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
  
  Serial.println("ARDUINO:READY");
  Serial.println("VERSION:2.0");
  Serial.println("BAUD:115200");
}

// =========================
// LOOP PRINCIPAL
// =========================
void loop() {
  // Leer comandos serial
  readSerialCommand();
  
  // Watchdog: si hay emergency stop, asegurar motores detenidos
  if (emergencyStop) {
    stopAllMotors();
  }
  
  delay(10); // Anti-bounce
}

// =========================
// LECTURA DE COMANDOS SERIAL
// =========================
void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < MAX_CMD_LENGTH - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

// =========================
// PROCESAMIENTO DE COMANDOS
// =========================
void processCommand(char* cmd) {
  // Convertir a mayúsculas para comparación
  String command = String(cmd);
  command.toUpperCase();
  command.trim();
  
  // ===== MOTORES =====
  if (command.startsWith("MOTOR LEFT FORWARD")) {
    int vel = extractValue(command, 3);
    setMotorLeft(constrain(vel, 0, 100), true);
    Serial.println("OK:MOTOR_LEFT_FWD");
  }
  else if (command.startsWith("MOTOR LEFT BACK")) {
    int vel = extractValue(command, 3);
    setMotorLeft(constrain(vel, 0, 100), false);
    Serial.println("OK:MOTOR_LEFT_BACK");
  }
  else if (command.startsWith("MOTOR RIGHT FORWARD")) {
    int vel = extractValue(command, 3);
    setMotorRight(constrain(vel, 0, 100), true);
    Serial.println("OK:MOTOR_RIGHT_FWD");
  }
  else if (command.startsWith("MOTOR RIGHT BACK")) {
    int vel = extractValue(command, 3);
    setMotorRight(constrain(vel, 0, 100), false);
    Serial.println("OK:MOTOR_RIGHT_BACK");
  }
  else if (command.startsWith("MOTORS STOP")) {
    stopAllMotors();
    Serial.println("OK:MOTORS_STOP");
  }
  
  // ===== SERVOS =====
  else if (command.startsWith("SERVO")) {
    int id = extractValue(command, 1);
    int angle = extractValue(command, 2);
    setServo(id, angle);
  }
  
  // ===== SENSORES =====
  else if (command.startsWith("SENSOR TEMP")) {
    readTemperature();
  }
  else if (command.startsWith("SENSOR LDR")) {
    readLight();
  }
  else if (command.startsWith("SENSOR HUM_CAP")) {
    readHumidityCapacitive();
  }
  else if (command.startsWith("SENSOR DIST")) {
    readDistance();
  }
  else if (command.startsWith("SENSOR IMU")) {
    readIMU();
  }
  
  // ===== SECUENCIAS =====
  else if (command.startsWith("ACTUATE BRACO START")) {
    executeBracoSequence();
  }
  
  // ===== EMERGENCIA =====
  else if (command.startsWith("EMERGENCY STOP")) {
    emergencyStop = true;
    stopAllMotors();
    Serial.println("EMERGENCY:OK");
  }
  else if (command.startsWith("EMERGENCY CLEAR")) {
    emergencyStop = false;
    Serial.println("EMERGENCY:CLEARED");
  }
  
  // ===== DESCONOCIDO =====
  else {
    Serial.print("ERR:UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}

// =========================
// FUNCIONES DE MOTORES
// =========================
void setMotorLeft(int speedPercent, bool forward) {
  if (emergencyStop) return;
  
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  currentLeftSpeed = pwmValue;
  leftForward = forward;
  
  digitalWrite(MOTOR_LEFT_DIR, forward ? HIGH : LOW);
  analogWrite(MOTOR_LEFT_PWM, pwmValue);
}

void setMotorRight(int speedPercent, bool forward) {
  if (emergencyStop) return;
  
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  currentRightSpeed = pwmValue;
  rightForward = forward;
  
  digitalWrite(MOTOR_RIGHT_DIR, forward ? HIGH : LOW);
  analogWrite(MOTOR_RIGHT_PWM, pwmValue);
}

void stopAllMotors() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

// =========================
// FUNCIONES DE SERVOS
// =========================
void setServo(int id, int angle) {
  angle = constrain(angle, 0, 180);
  
  switch(id) {
    case 1:
      angle = constrain(angle, 0, 30); // Límite cámara frontal
      servo1.write(angle);
      Serial.print("OK:SERVO1:");
      Serial.println(angle);
      break;
    case 2:
      servo2.write(angle);
      Serial.print("OK:SERVO2:");
      Serial.println(angle);
      break;
    case 3:
      servo3.write(angle);
      Serial.print("OK:SERVO3:");
      Serial.println(angle);
      break;
    case 4:
      angle = constrain(angle, 0, 90); // Límite sensor humedad
      servo4.write(angle);
      Serial.print("OK:SERVO4:");
      Serial.println(angle);
      break;
    default:
      Serial.println("ERR:INVALID_SERVO_ID");
  }
}

// =========================
// LECTURA DE SENSORES
// =========================
void readTemperature() {
  float temp = dht.readTemperature();
  
  if (isnan(temp)) {
    Serial.println("ERR:TEMP_READ_FAIL");
  } else {
    Serial.print("TEMP:");
    Serial.println(temp, 2);
  }
}

void readLight() {
  int raw = analogRead(LDR_PIN);
  float voltage = raw * (5.0 / 1023.0);
  
  Serial.print("LDR:");
  Serial.println(voltage, 2);
}

void readHumidityCapacitive() {
  int raw = analogRead(HUM_CAP_PIN);
  // Calibración típica: seco ~550, húmedo ~280
  int humPercent = map(raw, 550, 280, 0, 100);
  humPercent = constrain(humPercent, 0, 100);
  
  Serial.print("HUM:");
  Serial.println(humPercent);
}

void readDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    Serial.print("DIST:");
    Serial.println(measure.RangeMilliMeter / 10); // mm a cm
  } else {
    Serial.println("DIST:OUT_OF_RANGE");
  }
}

void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Convertir a unidades físicas
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;
  float gxDeg = gx / 131.0;
  float gyDeg = gy / 131.0;
  float gzDeg = gz / 131.0;
  
  Serial.print("IMU:");
  Serial.print(axG, 3); Serial.print(",");
  Serial.print(ayG, 3); Serial.print(",");
  Serial.print(azG, 3); Serial.print(",");
  Serial.print(gxDeg, 3); Serial.print(",");
  Serial.print(gyDeg, 3); Serial.print(",");
  Serial.println(gzDeg, 3);
}

// =========================
// SECUENCIA AUTOMÁTICA DEL BRAZO
// =========================
void executeBracoSequence() {
  Serial.println("BRACO:STARTING");
  
  // 1. Extender brazo (servo 2)
  servo2.write(45);
  delay(1000);
  
  // 2. Bajar codo (servo 3)
  servo3.write(135);
  delay(1000);
  
  // 3. Girar sensor de humedad hacia el suelo (servo 4)
  servo4.write(90);
  delay(500);
  
  // 4. Esperar estabilización
  delay(1000);
  
  // 5. Leer humedad (opcional, el servidor lo pedirá)
  
  // 6. Retraer brazo
  servo4.write(45);
  delay(500);
  servo3.write(90);
  delay(1000);
  servo2.write(90);
  delay(1000);
  
  Serial.println("BRACO:OK");
}

// =========================
// UTILIDADES
// =========================
int extractValue(String cmd, int position) {
  int count = 0;
  int startIdx = 0;
  
  for (int i = 0; i < cmd.length(); i++) {
    if (cmd[i] == ' ') {
      if (count == position) {
        return cmd.substring(startIdx, i).toInt();
      }
      count++;
      startIdx = i + 1;
    }
  }
  
  if (count == position) {
    return cmd.substring(startIdx).toInt();
  }
  
  return 0;
}
