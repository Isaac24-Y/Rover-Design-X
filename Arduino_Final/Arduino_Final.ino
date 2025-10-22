#include <Servo.h>

Servo servoMotor;
int pos = 0;
String comando;

// Pines de sensores y actuadores
const int pinTemp = A0;   // Sensor de temperatura (LM35)
const int pinLuz  = A1;   // LDR
const int pinDC   = 3;    // Motor DC PWM
const int motor_A = 5;    // Motor adicional (si se quiere control de dirección)
const int motor_B = 6;

// ─ CONFIGURACIÓN INICIAL ─
void setup() {
  servoMotor.attach(9);
  servoMotor.write(pos);

  pinMode(pinDC, OUTPUT);
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Listo para recibir comandos.");
}

// ─ BUCLE PRINCIPAL ─
void loop() {
  if (Serial.available() > 0) {
    comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "servo1") {
      pos += 20;
      if (pos > 180) pos = 0;
      servoMotor.write(pos);
      Serial.println("Servo movido a pos: " + String(pos));

    } 
    else if (comando == "temperatura") {
      int lectura = analogRead(pinTemp);
      float temperatura = lectura * 5.0 / 1023.0 * 100.0; // LM35: 10 mV/°C
      Serial.println(temperatura, 2);

    } 
    else if (comando == "luz") {
      int lectura = analogRead(pinLuz);
      float luz = lectura * 5.0 / 1023.0; // Voltaje LDR
      Serial.println(luz, 2);

    } 
    else if (comando == "dc") {
      analogWrite(pinDC, 255); // Encender motor DC
      Serial.println("Motor DC activado");

    } 
    else if (comando == "stop") {
      analogWrite(pinDC, 0);
      servoMotor.detach();
      digitalWrite(motor_A, 0);
      digitalWrite(motor_B, 0);
      Serial.println("Motores y servo detenidos");

    } 
    else if (comando == "derecha") {
      analogWrite(motor_A, 255);
      digitalWrite(motor_B, 0);
      Serial.println("Motor girando derecha");

    } 
    else if (comando == "izquierda") {
      analogWrite(motor_B, 255);
      digitalWrite(motor_A, 0);
      Serial.println("Motor girando izquierda");

    } 
    else {
      Serial.println("Comando invalido");
    }

    delay(50); // Pausa pequeña para estabilidad
  }
}

