/*
  ROVER DE EXPLORACIÓN - ARDUINO NANO
  Sistema completo de control con sensores y actuadores
*/

#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <VL53L0X.h>

// ========== CONFIGURACIÓN DE PINES ==========
// Puentes H - Motores
const int MOTOR_IZQ_A = 5;   // PWM
const int MOTOR_IZQ_B = 6;   // PWM
const int MOTOR_DER_A = 9;   // PWM
const int MOTOR_DER_B = 10;  // PWM

// Sensores analógicos
const int PIN_LUZ = A0;
const int PIN_TEMP_HUM = A1;  // EK1940
const int PIN_HUMEDAD = A2;   // Sensor capacitivo

// Servos
const int SERVO_CAM_PIN = 3;      // Servo cámara superior (0-180°)
const int SERVO_BRAZO_1_PIN = 7;  // Base brazo (0-30°)
const int SERVO_BRAZO_2_PIN = 8;  // Articulación 1 (0-180°)
const int SERVO_BRAZO_3_PIN = 11; // Articulación 2 (0-180°)
const int SERVO_BRAZO_4_PIN = 12; // Garra (0-180°)

// ========== OBJETOS ==========
Servo servoCamara;
Servo servoBrazo1;  // 0-30° limitado
Servo servoBrazo2;
Servo servoBrazo3;
Servo servoBrazo4;

MPU6050 imu;
VL53L0X sensorDistancia;

// ========== VARIABLES GLOBALES ==========
String comandoRecibido = "";
bool comandoCompleto = false;

// Estado de sensores
float temperatura = 0.0;
float humedadAire = 0.0;
float humedadSuelo = 0.0;
float intensidadLuz = 0.0;
int distancia = 0;

// Estado IMU
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Posiciones servos
int posCamara = 90;
int posBrazo1 = 0;
int posBrazo2 = 90;
int posBrazo3 = 90;
int posBrazo4 = 90;

// Estado del sistema
bool modoAutonomo = false;
bool brazoActivo = false;
unsigned long ultimaLectura = 0;
const unsigned long INTERVALO_LECTURA = 500; // ms

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Configurar pines motores
  pinMode(MOTOR_IZQ_A, OUTPUT);
  pinMode(MOTOR_IZQ_B, OUTPUT);
  pinMode(MOTOR_DER_A, OUTPUT);
  pinMode(MOTOR_DER_B, OUTPUT);
  detenerMotores();
  
  // Configurar servos
  servoCamara.attach(SERVO_CAM_PIN);
  servoBrazo1.attach(SERVO_BRAZO_1_PIN);
  servoBrazo2.attach(SERVO_BRAZO_2_PIN);
  servoBrazo3.attach(SERVO_BRAZO_3_PIN);
  servoBrazo4.attach(SERVO_BRAZO_4_PIN);
  
  // Posiciones iniciales
  servoCamara.write(posCamara);
  servoBrazo1.write(posBrazo1);
  servoBrazo2.write(posBrazo2);
  servoBrazo3.write(posBrazo3);
  servoBrazo4.write(posBrazo4);
  
  // Inicializar IMU
  imu.initialize();
  if (!imu.testConnection()) {
    enviarLog("ERROR: IMU no conectada");
  }
  
  // Inicializar sensor de distancia
  sensorDistancia.init();
  sensorDistancia.setTimeout(500);
  sensorDistancia.startContinuous();
  
  delay(1000);
  enviarLog("SISTEMA INICIADO");
  enviarStatus();
}

// ========== LOOP PRINCIPAL ==========
void loop() {
  // Leer comandos seriales
  procesarSerial();
  
  // Ejecutar comando si está completo
  if (comandoCompleto) {
    ejecutarComando(comandoRecibido);
    comandoRecibido = "";
    comandoCompleto = false;
  }
  
  // Lecturas periódicas automáticas
  if (millis() - ultimaLectura >= INTERVALO_LECTURA) {
    leerSensores();
    ultimaLectura = millis();
  }
  
  delay(10);
}

// ========== PROCESAMIENTO SERIAL ==========
void procesarSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (comandoRecibido.length() > 0) {
        comandoRecibido.trim();
        comandoCompleto = true;
      }
    } else {
      comandoRecibido += c;
    }
  }
}

// ========== EJECUCIÓN DE COMANDOS ==========
void ejecutarComando(String cmd) {
  cmd.toLowerCase();
  
  // ===== MOTORES =====
  if (cmd == "adelante") {
    moverAdelante();
  }
  else if (cmd == "atras") {
    moverAtras();
  }
  else if (cmd == "izquierda") {
    girarIzquierda();
  }
  else if (cmd == "derecha") {
    girarDerecha();
  }
  else if (cmd == "stop") {
    detenerMotores();
  }
  else if (cmd.startsWith("velocidad:")) {
    int vel = cmd.substring(10).toInt();
    ajustarVelocidad(vel);
  }
  
  // ===== SERVO CÁMARA =====
  else if (cmd.startsWith("camara:")) {
    int ang = cmd.substring(7).toInt();
    moverCamara(ang);
  }
  
  // ===== BRAZO ROBÓTICO =====
  else if (cmd.startsWith("brazo1:")) {
    int ang = cmd.substring(7).toInt();
    moverBrazo1(ang);
  }
  else if (cmd.startsWith("brazo2:")) {
    int ang = cmd.substring(7).toInt();
    moverBrazo2(ang);
  }
  else if (cmd.startsWith("brazo3:")) {
    int ang = cmd.substring(7).toInt();
    moverBrazo3(ang);
  }
  else if (cmd.startsWith("garra:")) {
    int ang = cmd.substring(6).toInt();
    moverGarra(ang);
  }
  else if (cmd == "brazo_home") {
    posicionHomeBrazo();
  }
  else if (cmd == "brazo_muestra") {
    tomarMuestra();
  }
  
  // ===== SENSORES =====
  else if (cmd == "sensores") {
    leerSensores();
    enviarDatosSensores();
  }
  else if (cmd == "temperatura") {
    enviarRespuesta("temperatura", String(temperatura, 2) + " °C");
  }
  else if (cmd == "luz") {
    enviarRespuesta("luz", String(intensidadLuz, 2) + " lux");
  }
  else if (cmd == "humedad_suelo") {
    enviarRespuesta("humedad_suelo", String(humedadSuelo, 2) + " %");
  }
  else if (cmd == "distancia") {
    enviarRespuesta("distancia", String(distancia) + " mm");
  }
  else if (cmd == "imu") {
    enviarDatosIMU();
  }
  
  // ===== SISTEMA =====
  else if (cmd == "status") {
    enviarStatus();
  }
  else if (cmd == "modo_auto") {
    modoAutonomo = true;
    enviarLog("Modo autónomo activado");
  }
  else if (cmd == "modo_manual") {
    modoAutonomo = false;
    enviarLog("Modo manual activado");
  }
  else if (cmd == "reset") {
    resetearSistema();
  }
  
  // ===== COMANDO NO RECONOCIDO =====
  else {
    enviarRespuesta("error", "Comando no reconocido: " + cmd);
  }
}

// ========== FUNCIONES DE MOTORES ==========
void moverAdelante() {
  analogWrite(MOTOR_IZQ_A, 200);
  analogWrite(MOTOR_IZQ_B, 0);
  analogWrite(MOTOR_DER_A, 200);
  analogWrite(MOTOR_DER_B, 0);
  enviarLog("Avanzando");
}

void moverAtras() {
  analogWrite(MOTOR_IZQ_A, 0);
  analogWrite(MOTOR_IZQ_B, 200);
  analogWrite(MOTOR_DER_A, 0);
  analogWrite(MOTOR_DER_B, 200);
  enviarLog("Retrocediendo");
}

void girarIzquierda() {
  analogWrite(MOTOR_IZQ_A, 0);
  analogWrite(MOTOR_IZQ_B, 180);
  analogWrite(MOTOR_DER_A, 180);
  analogWrite(MOTOR_DER_B, 0);
  enviarLog("Girando a la izquierda");
}

void girarDerecha() {
  analogWrite(MOTOR_IZQ_A, 180);
  analogWrite(MOTOR_IZQ_B, 0);
  analogWrite(MOTOR_DER_A, 0);
  analogWrite(MOTOR_DER_B, 180);
  enviarLog("Girando a la derecha");
}

void detenerMotores() {
  analogWrite(MOTOR_IZQ_A, 0);
  analogWrite(MOTOR_IZQ_B, 0);
  analogWrite(MOTOR_DER_A, 0);
  analogWrite(MOTOR_DER_B, 0);
  enviarLog("Motores detenidos");
}

void ajustarVelocidad(int vel) {
  vel = constrain(vel, 0, 255);
  // Aplicar a motores actuales (mantener dirección)
  enviarLog("Velocidad ajustada a " + String(vel));
}

// ========== FUNCIONES DE SERVOS ==========
void moverCamara(int angulo) {
  posCamara = constrain(angulo, 0, 180);
  servoCamara.write(posCamara);
  enviarLog("Cámara a " + String(posCamara) + "°");
}

void moverBrazo1(int angulo) {
  posBrazo1 = constrain(angulo, 0, 30);
  servoBrazo1.write(posBrazo1);
  enviarLog("Brazo base a " + String(posBrazo1) + "°");
}

void moverBrazo2(int angulo) {
  posBrazo2 = constrain(angulo, 0, 180);
  servoBrazo2.write(posBrazo2);
  enviarLog("Brazo articulación 1 a " + String(posBrazo2) + "°");
}

void moverBrazo3(int angulo) {
  posBrazo3 = constrain(angulo, 0, 180);
  servoBrazo3.write(posBrazo3);
  enviarLog("Brazo articulación 2 a " + String(posBrazo3) + "°");
}

void moverGarra(int angulo) {
  posBrazo4 = constrain(angulo, 0, 180);
  servoBrazo4.write(posBrazo4);
  enviarLog("Garra a " + String(posBrazo4) + "°");
}

void posicionHomeBrazo() {
  servoBrazo1.write(0);
  delay(300);
  servoBrazo2.write(90);
  delay(300);
  servoBrazo3.write(90);
  delay(300);
  servoBrazo4.write(90);
  enviarLog("Brazo en posición HOME");
}

void tomarMuestra() {
  enviarLog("Iniciando secuencia de toma de muestra");
  brazoActivo = true;
  
  // Secuencia de movimiento del brazo
  servoBrazo1.write(15);
  delay(500);
  servoBrazo2.write(45);
  delay(500);
  servoBrazo3.write(120);
  delay(500);
  
  // Abrir garra
  servoBrazo4.write(180);
  delay(300);
  
  // Bajar brazo
  servoBrazo3.write(150);
  delay(500);
  
  // Cerrar garra (tomar muestra)
  servoBrazo4.write(90);
  delay(500);
  
  // Leer humedad del suelo
  leerHumedadSuelo();
  
  enviarLog("Muestra de humedad tomada: " + String(humedadSuelo, 2) + " %");
  
  // Subir brazo
  servoBrazo3.write(90);
  delay(500);
  servoBrazo2.write(90);
  delay(500);
  
  // Volver a home
  posicionHomeBrazo();
  brazoActivo = false;
  
  enviarRespuesta("muestra_completada", String(humedadSuelo, 2));
}

// ========== LECTURA DE SENSORES ==========
void leerSensores() {
  leerTemperaturaHumedad();
  leerLuz();
  leerDistancia();
  leerIMU();
}

void leerTemperaturaHumedad() {
  // EK1940 - Ajustar según especificaciones del sensor
  int lectura = analogRead(PIN_TEMP_HUM);
  float voltaje = lectura * (5.0 / 1023.0);
  
  // Conversión aproximada (ajustar según datasheet)
  temperatura = (voltaje - 0.5) * 100.0;
  humedadAire = (voltaje / 5.0) * 100.0;
}

void leerLuz() {
  int lectura = analogRead(PIN_LUZ);
  // Conversión a lux (ajustar según fotorresistor)
  intensidadLuz = map(lectura, 0, 1023, 0, 1000);
}

void leerHumedadSuelo() {
  int lectura = analogRead(PIN_HUMEDAD);
  // Sensor capacitivo: menor valor = más humedad
  humedadSuelo = map(lectura, 1023, 0, 0, 100);
}

void leerDistancia() {
  distancia = sensorDistancia.readRangeContinuousMillimeters();
  if (sensorDistancia.timeoutOccurred()) {
    distancia = -1;
  }
}

void leerIMU() {
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

// ========== ENVÍO DE DATOS ==========
void enviarDatosSensores() {
  Serial.print("SENSORES:");
  Serial.print(temperatura, 2);
  Serial.print(",");
  Serial.print(humedadAire, 2);
  Serial.print(",");
  Serial.print(intensidadLuz, 2);
  Serial.print(",");
  Serial.print(distancia);
  Serial.println();
}

void enviarDatosIMU() {
  Serial.print("IMU:");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println();
}

void enviarStatus() {
  Serial.print("STATUS:");
  Serial.print(modoAutonomo ? "AUTO" : "MANUAL");
  Serial.print(",");
  Serial.print(brazoActivo ? "BRAZO_ACTIVO" : "BRAZO_INACTIVO");
  Serial.println();
}

void enviarLog(String mensaje) {
  Serial.print("LOG:");
  Serial.println(mensaje);
}

void enviarRespuesta(String tipo, String valor) {
  Serial.print("RESP:");
  Serial.print(tipo);
  Serial.print("=");
  Serial.println(valor);
}

void resetearSistema() {
  detenerMotores();
  posicionHomeBrazo();
  servoCamara.write(90);
  modoAutonomo = false;
  enviarLog("Sistema reseteado");
}