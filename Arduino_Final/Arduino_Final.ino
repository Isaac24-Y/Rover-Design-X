#include <Servo.h>

Servo servoMotor;
int pos = 0;
String comando;

const int pinTemp = A0;
const int pinLuz = A1;
const int pinDC = 3;
const int motor_A = 5;
const int motor_B = 6;

void setup() {
  servoMotor.attach(9);
  servoMotor.write(pos);

  pinMode(pinDC, OUTPUT);
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  Serial.begin(9600);

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
      Serial.println("Servo movido a la derecha.");
    } 
    else if (comando == "temperatura") {
      int lectura = analogRead(pinTemp);
      float temperatura = lectura * 5.0 / 1023.0 * 100; // LM35
      Serial.println(temperatura, 2);
    } 
    else if (comando == "luz") {
      int lectura = analogRead(pinLuz);
      float luz = lectura * 5.0 / 1023.0;
      Serial.println(luz, 2);
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
    else if (comando == "stop") {
      analogWrite(pinDC, 0);
      digitalWrite(motor_A, 0);
      digitalWrite(motor_B, 0);
      Serial.println("Motores detenidos.");
    } 
    else {
      Serial.println("Comando inválido.");
    }

    delay(200);
  }
}

