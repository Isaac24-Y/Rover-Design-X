#include <Servo.h>
#include <dht.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <MPU6050.h>

//=====================================================
// VARIABLES GLOBALES
//=====================================================

String comando;     // Comando desde Raspberry Pi
String accion;
int valor;

//---------------- SERVOS ----------------
Servo servo1;   // Codo brazo (2)
Servo servo2;   // Sensor brazo (3)
Servo servo3;   // Cámara frontal (4)
Servo servo4;   // Cámara superior (9)

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

//---------------- DHT11 ----------------
dht DHT;
const int pinTemp = 8;

//---------------- HUMEDAD ----------------
const int pinHumedad = A0;

//---------------- LDR ----------------
const int pinLuz = A1;

//---------------- DISTANCIA VL53L0X ----------------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//---------------- MPU 6050 ----------------
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//---------------- MOTORES ----------------
const int motor_A1 = 5;
const int motor_A2 = 6;
const int motor_B1 = 10;
const int motor_B2 = 11;


//=====================================================
// SETUP
//=====================================================
void setup() {

  Serial.begin(9600);

  //---------------- SERVOS ----------------
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(9);

  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);

  //---------------- MPU6050 ----------------
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado.");
  } else {
    Serial.println("Error: MPU6050 no detectado.");
  }

  //---------------- MOTORES ----------------
  pinMode(motor_A1, OUTPUT);
  pinMode(motor_A2, OUTPUT);
  pinMode(motor_B1, OUTPUT);
  pinMode(motor_B2, OUTPUT);

  //---------------- VL53L0X ----------------
  if (!lox.begin()) {
    Serial.println("Error: VL53L0X no detectado.");
  }

  Serial.println("Listo para recibir comandos.");
}


//=====================================================
// FUNCIONES DE SENSORES
//=====================================================
float leerTemperatura() {
  DHT.read11(pinTemp);
  return DHT.temperature;    // °C reales
}

int leerHumedad() {
  int lectura = analogRead(pinHumedad);
  int humedad = map(lectura, 588, 308, 0, 100);
  return humedad;
}

int leerLuz() {
  int lectura = analogRead(pinLuz);
  int luz = map(lectura, 1023, 0, 0, 100);
  return luz;
}

float leerDistancia() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  return measure.RangeMilliMeter;
}

void leerMPU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

//=====================================================
// FUNCIONES DE MOTORES
//=====================================================
void detenerMotores() {
  digitalWrite(motor_A1, LOW);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, LOW);
  digitalWrite(motor_B2, LOW);
}

void avanzar(int vel) {
  vel = constrain(vel, 0, 255);
  analogWrite(motor_A1, vel);
  analogWrite(motor_B1, vel);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B2, LOW);
}

void retroceder(int vel) {
  vel = constrain(vel, 0, 255);
  analogWrite(motor_A2, vel);
  analogWrite(motor_B2, vel);
  digitalWrite(motor_A1, LOW);
  digitalWrite(motor_B1, LOW);
}

void motoresDerecha(int vel) {
  vel = constrain(vel, -255, 255);

  if (vel > 0) {
    analogWrite(motor_A1, vel);
    digitalWrite(motor_A2, LOW);
  } else if (vel < 0) {
    analogWrite(motor_A2, -vel);
    digitalWrite(motor_A1, LOW);
  } else {
    digitalWrite(motor_A1, LOW);
    digitalWrite(motor_A2, LOW);
  }
}

void motoresIzquierda(int vel) {
  vel = constrain(vel, -255, 255);

  if (vel > 0) {
    analogWrite(motor_B1, vel);
    digitalWrite(motor_B2, LOW);
  } else if (vel < 0) {
    analogWrite(motor_B2, -vel);
    digitalWrite(motor_B1, LOW);
  } else {
    digitalWrite(motor_B1, LOW);
    digitalWrite(motor_B2, LOW);
  }
}


//=====================================================
// MANEJO DE COMANDOS
//=====================================================

void procesarComando(String accion, int valor) {

  //====== SERVOS =========
  if (accion == "servo1") { servo1.write(valor); }
  else if (accion == "servo2") { servo2.write(valor); }
  else if (accion == "servo3") { servo3.write(valor); }
  else if (accion == "servo4") { servo4.write(valor); }

  //====== MOTORES ========
  else if (accion == "avanzar") { avanzar(valor); }
  else if (accion == "retroceder") { retroceder(valor); }
  else if (accion == "der") { motoresDerecha(valor); }
  else if (accion == "izq") { motoresIzquierda(valor); }
  else if (accion == "stop") { detenerMotores(); }

  //====== SENSORES ========
  else if (accion == "temp") {
    Serial.println(leerTemperatura());
  }
  else if (accion == "humedad") {
    Serial.println(leerHumedad());
  }
  else if (accion == "luz") {
    Serial.println(leerLuz());
  }
  else if (accion == "dist") {
    Serial.println(leerDistancia());
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

  else {
    Serial.println("Comando desconocido.");
  }
}


//=====================================================
// LOOP PRINCIPAL
//=====================================================
void loop() {

  if (Serial.available()) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    int espacio = comando.indexOf(' ');

    if (espacio > 0) {
      accion = comando.substring(0, espacio);
      valor  = comando.substring(espacio + 1).toInt();
    } else {
      accion = comando;
      valor = 0;
    }

    procesarComando(accion, valor);

    delay(20);
  }
}
