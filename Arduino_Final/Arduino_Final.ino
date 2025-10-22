#include <Servo.h>

// ────────────── Configuración de pines ──────────────
Servo servoMotor;
int pos = 0;

const int pinTemp = A0;   // Sensor LM35
const int pinLuz  = A1;   // LDR
const int pinDC   = 3;    // PWM Motor DC
const int motor_A = 5;
const int motor_B = 6;

String comando;

void setup() {
  Serial.begin(9600);
  servoMotor.attach(9);
  servoMotor.write(pos);

  pinMode(pinDC, OUTPUT);
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  Serial.println("Listo para recibir comandos.");
}

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
      Serial.print(temperatura, 2);
      Serial.println(" °C");
    } 
    else if (comando == "luz") {
      int lectura = analogRead(pinLuz);
      float luz = lectura * 5.0 / 1023.0;
      Serial.print(luz, 2);
      Serial.println(" V");
    } 
    else if (comando == "derecha") {
      analogWrite(motor_A, 255);
      digitalWrite(motor_B, 0);
      Serial.println("Motor girando a la derecha.");
    } 
    else if (comando == "izquierda") {
      analogWrite(motor_B, 255);
      digitalWrite(motor_A, 0);
      Serial.println("Motor girando a la izquierda.");
    } 
    else if (comando == "dc") {
      analogWrite(pinDC, 255);
      Serial.println("Motor DC activado.");
    } 
    else if (comando == "stop") {
      analogWrite(pinDC, 0);
      digitalWrite(motor_A, 0);
      digitalWrite(motor_B, 0);
      Serial.println("Motores detenidos.");
    } 
    else {
      Serial.println("Comando inválido.");
    }
    delay(50); // Pequeña pausa para evitar saturación serial
  }
}
