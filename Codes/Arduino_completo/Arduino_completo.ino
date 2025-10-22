#include <Servo.h>

Servo servoMotor;
int pos = 0;              // Posición inicial del servo
String comando;            // Almacena el texto recibido

// Pines de sensores y actuadores
const int pinTemp = A0;    // LDR
const int pinLuz  = A1;    // Otro sensor si quieres
const int pinDC   = 3;     // PWM para motor DC

// Variables de lectura
float voltaje;
float luz_val;

void setup() {
  servoMotor.attach(9);
  servoMotor.write(pos);

  pinMode(pinDC, OUTPUT);
  Serial.begin(9600);

  Serial.println("Esperando comando de Raspberry Pi...");
  Serial.println("Comandos válidos: servo1, dc, luz, temp, stop");
}

void loop() {
  if (Serial.available() > 0) {
    comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "servo1") {
      // Mueve el servo en pasos
      pos += 20;
      if(pos >= 180) pos = 0;
      servoMotor.write(pos);
      Serial.println("Servo movido a pos: " + String(pos));

    } else if (comando == "dc") {
      // Motor DC encendido
      analogWrite(pinDC, 255);
      Serial.println("Motor DC activado");

    } else if (comando == "stop") {
      // Apagar todo
      servoMotor.detach();
      analogWrite(pinDC, 0);
      Serial.println("Stop: Servo y DC apagados");

    } else if (comando == "temp") {
      // Lectura temp
      int lectura = analogRead(pinTemp);
      voltaje = lectura * 5.0 / 1023.0;  // Convertir a voltaje
      Serial.println(String(voltaje, 2)); // Enviar solo número para guardar

    } else if (comando == "luz") {
      int lectura = analogRead(pinLuz);
      luz_val = lectura * 5.0 / 1023.0;
      Serial.println(String(luz_val, 2));

    } else {
      Serial.println("Comando inválido");
    }

    delay(50);  // Pequeña pausa
  }
}
