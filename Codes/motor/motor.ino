/*
 * ROVER - ARDUINO NANO OPTIMIZADO
 * Sin delays, respuestas inmediatas
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <DHT.h>

// ===== PINES =====
#define MOTOR_LEFT_PWM   3
#define MOTOR_LEFT_DIR   2
#define MOTOR_RIGHT_PWM  5
#define MOTOR_RIGHT_DIR  4

#define SERVO1_PIN  6
#define SERVO2_PIN  7
#define SERVO3_PIN  8
#define SERVO4_PIN  9

#define LDR_PIN     A0
#define DHT_PIN     A1
#define HUM_CAP_PIN A2

#define MPU6050_ADDR 0x68

// ===== OBJETOS =====
Servo servo1, servo2, servo3, servo4;
DHT dht(DHT_PIN, DHT11);
Adafruit_VL53L0X lox;

// ===== VARIABLES =====
char cmdBuffer[64];
uint8_t cmdIndex = 0;
unsigned long lastReadTime = 0;
bool emergencyStop = false;

// Cache de sensores
float cachedTemp = 0;
float cachedLight = 0;
int cachedDist = 0;
int cachedHum = 0;

void setup() {
  Serial.begin(115200);
  
  // Motores
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  stopMotors();
  
  // Servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  
  servo1.write(15);
  servo2.write(90);
  servo3.write(90);
  servo4.write(45);
  
  // Sensores
  dht.begin();
  Wire.begin();
  
  if (!lox.begin()) {
    Serial.println("ERR:VL53L0X_FAIL");
  }
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  
  Serial.println("READY");
}

void loop() {
  // Leer comandos sin blocking
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand();
        cmdIndex = 0;
      }
    } else if (cmdIndex < 63) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
  
  // Actualizar cache cada 200ms
  if (millis() - lastReadTime > 200) {
    updateSensorCache();
    lastReadTime = millis();
  }
  
  if (emergencyStop) {
    stopMotors();
  }
}

void processCommand() {
  String cmd = String(cmdBuffer);
  cmd.toUpperCase();
  cmd.trim();
  
  // MOTORES - Respuesta inmediata
  if (cmd.startsWith("ML")) { // Motor Left
    int vel = extractValue(cmd, 1);
    setMotor(true, vel > 0, abs(vel));
    Serial.println("OK");
  }
  else if (cmd.startsWith("MR")) { // Motor Right
    int vel = extractValue(cmd, 1);
    setMotor(false, vel > 0, abs(vel));
    Serial.println("OK");
  }
  else if (cmd.startsWith("MS")) { // Motor Stop
    stopMotors();
    Serial.println("OK");
  }
  
  // SERVOS
  else if (cmd.startsWith("S")) {
    int id = cmd.charAt(1) - '0';
    int angle = extractValue(cmd, 1);
    setServo(id, angle);
  }
  
  // SENSORES - Desde cache
  else if (cmd.startsWith("T")) { // Temp
    Serial.print("T:");
    Serial.println(cachedTemp, 1);
  }
  else if (cmd.startsWith("L")) { // Light
    Serial.print("L:");
    Serial.println(cachedLight, 2);
  }
  else if (cmd.startsWith("D")) { // Distance
    Serial.print("D:");
    Serial.println(cachedDist);
  }
  else if (cmd.startsWith("H")) { // Humidity
    Serial.print("H:");
    Serial.println(cachedHum);
  }
  else if (cmd.startsWith("I")) { // IMU
    readIMU();
  }
  
  // BRAZO
  else if (cmd.startsWith("B")) {
    executeBraco();
  }
  
  // EMERGENCIA
  else if (cmd.startsWith("E")) {
    emergencyStop = !emergencyStop;
    if (emergencyStop) stopMotors();
    Serial.println(emergencyStop ? "E:ON" : "E:OFF");
  }
}

void setMotor(bool isLeft, bool forward, int speed) {
  if (emergencyStop) return;
  
  int pwm = map(constrain(speed, 0, 100), 0, 100, 0, 255);
  
  if (isLeft) {
    digitalWrite(MOTOR_LEFT_DIR, forward ? HIGH : LOW);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, forward ? HIGH : LOW);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void setServo(int id, int angle) {
  angle = constrain(angle, 0, 180);
  
  switch(id) {
    case 1:
      angle = constrain(angle, 0, 30);
      servo1.write(angle);
      break;
    case 2:
      servo2.write(angle);
      break;
    case 3:
      servo3.write(angle);
      break;
    case 4:
      angle = constrain(angle, 0, 90);
      servo4.write(angle);
      break;
  }
  Serial.print("S");
  Serial.print(id);
  Serial.print(":");
  Serial.println(angle);
}

void updateSensorCache() {
  // Temperatura
  float t = dht.readTemperature();
  if (!isnan(t)) cachedTemp = t;
  
  // Luz
  cachedLight = analogRead(LDR_PIN) * (5.0 / 1023.0);
  
  // Distancia
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    cachedDist = measure.RangeMilliMeter / 10;
  }
  
  // Humedad capacitiva
  int raw = analogRead(HUM_CAP_PIN);
  cachedHum = map(constrain(raw, 280, 550), 280, 550, 100, 0);
}

void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temp
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  Serial.print("I:");
  Serial.print(ax / 16384.0, 2); Serial.print(",");
  Serial.print(ay / 16384.0, 2); Serial.print(",");
  Serial.print(az / 16384.0, 2); Serial.print(",");
  Serial.print(gx / 131.0, 1); Serial.print(",");
  Serial.print(gy / 131.0, 1); Serial.print(",");
  Serial.println(gz / 131.0, 1);
}

void executeBraco() {
  servo2.write(45);
  servo3.write(135);
  servo4.write(90);
  Serial.println("B:OK");
}

int extractValue(String cmd, int pos) {
  int count = 0;
  int start = 0;
  
  for (int i = 0; i < cmd.length(); i++) {
    if (cmd[i] == ' ' || cmd[i] == ':') {
      if (count == pos) return cmd.substring(start, i).toInt();
      count++;
      start = i + 1;
    }
  }
  
  if (count == pos) return cmd.substring(start).toInt();
  return 0;
}